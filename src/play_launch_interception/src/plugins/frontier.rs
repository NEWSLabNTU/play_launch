//! Frontier tracking plugin — tracks per-topic timestamp frontiers and writes
//! events when a frontier advances.
//!
//! **Shared memory mode** (preferred): activated when a `Producer` is provided.
//! Writes `InterceptionEvent` to the SPSC ring buffer.
//!
//! **Socket fallback**: activated when `PLAY_LAUNCH_INTERCEPTION_SOCKET` env
//! var is set. Sends 17-byte `FrontierEvent` datagrams over a Unix socket.
//! Useful for standalone debugging without play_launch.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use parking_lot::{Mutex, RwLock};

use crate::event::{EventKind, InterceptionEvent, monotonic_ns};
use crate::plugin::{InterceptionPlugin, Stamp};
use spsc_shm::Producer;

// ---------------------------------------------------------------------------
// Socket fallback wire format (legacy, for standalone debugging)
// ---------------------------------------------------------------------------

#[repr(C, packed)]
struct FrontierEvent {
    topic_hash: u64,
    stamp_sec: i32,
    stamp_nanosec: u32,
    event_type: u8,
}

const EVENT_PUBLISH: u8 = 0;
const EVENT_TAKE: u8 = 1;

// ---------------------------------------------------------------------------
// Transport: shared memory or socket
// ---------------------------------------------------------------------------

enum Transport {
    /// SPSC ring buffer (shared with other plugins).
    SharedMemory(Arc<Mutex<Producer<InterceptionEvent>>>),
    /// Unix datagram socket (standalone fallback).
    Socket(std::os::unix::net::UnixDatagram),
}

// ---------------------------------------------------------------------------
// FrontierPlugin
// ---------------------------------------------------------------------------

pub(crate) struct FrontierPlugin {
    transport: Transport,
    /// topic_hash -> frontier (leaked, never freed — process lifetime).
    frontiers: RwLock<HashMap<u64, &'static AtomicU64>>,
}

impl FrontierPlugin {
    /// Try to create a FrontierPlugin.
    ///
    /// - If `producer` is `Some`, uses shared memory transport.
    /// - Otherwise, falls back to `PLAY_LAUNCH_INTERCEPTION_SOCKET` env var.
    /// - Returns `None` if neither is available.
    pub fn new(producer: Option<Arc<Mutex<Producer<InterceptionEvent>>>>) -> Option<Self> {
        let transport = if let Some(prod) = producer {
            Transport::SharedMemory(prod)
        } else {
            // Socket fallback
            let socket_path = std::env::var_os("PLAY_LAUNCH_INTERCEPTION_SOCKET")?;
            let sock = std::os::unix::net::UnixDatagram::unbound().ok()?;
            sock.connect(&socket_path).ok()?;
            sock.set_nonblocking(true).ok()?;
            Transport::Socket(sock)
        };
        Some(Self {
            transport,
            frontiers: RwLock::new(HashMap::new()),
        })
    }

    /// Create a FrontierPlugin with a shared memory producer (for testing).
    #[cfg(test)]
    fn with_producer(producer: Arc<Mutex<Producer<InterceptionEvent>>>) -> Self {
        Self {
            transport: Transport::SharedMemory(producer),
            frontiers: RwLock::new(HashMap::new()),
        }
    }

    /// Create a FrontierPlugin connected to a socket path (for testing).
    #[cfg(test)]
    fn connect(socket_path: &std::path::Path) -> Option<Self> {
        let sock = std::os::unix::net::UnixDatagram::unbound().ok()?;
        sock.connect(socket_path).ok()?;
        sock.set_nonblocking(true).ok()?;
        Some(Self {
            transport: Transport::Socket(sock),
            frontiers: RwLock::new(HashMap::new()),
        })
    }

    fn get_or_create_frontier(&self, topic_hash: u64) -> &'static AtomicU64 {
        // Fast path: read lock
        {
            let map = self.frontiers.read();
            if let Some(f) = map.get(&topic_hash) {
                return f;
            }
        }
        // Slow path: write lock + insert
        let mut map = self.frontiers.write();
        map.entry(topic_hash)
            .or_insert_with(|| Box::leak(Box::new(AtomicU64::new(0))))
    }

    fn send_publish(&self, topic_hash: u64, stamp: &Stamp, handle: usize) {
        match &self.transport {
            Transport::SharedMemory(prod) => {
                let event = InterceptionEvent {
                    kind: EventKind::Publish,
                    _pad: [0; 3],
                    topic_hash,
                    stamp_sec: stamp.sec,
                    stamp_nanosec: stamp.nanosec,
                    handle: handle as u64,
                    monotonic_ns: monotonic_ns(),
                };
                let _ = prod.lock().push(&event);
            }
            Transport::Socket(sock) => {
                let event = FrontierEvent {
                    topic_hash,
                    stamp_sec: stamp.sec,
                    stamp_nanosec: stamp.nanosec,
                    event_type: EVENT_PUBLISH,
                };
                let bytes = unsafe {
                    std::slice::from_raw_parts(
                        &event as *const FrontierEvent as *const u8,
                        size_of::<FrontierEvent>(),
                    )
                };
                let _ = sock.send(bytes);
            }
        }
    }

    fn send_take(&self, topic_hash: u64, stamp: &Stamp, handle: usize) {
        match &self.transport {
            Transport::SharedMemory(prod) => {
                let event = InterceptionEvent {
                    kind: EventKind::Take,
                    _pad: [0; 3],
                    topic_hash,
                    stamp_sec: stamp.sec,
                    stamp_nanosec: stamp.nanosec,
                    handle: handle as u64,
                    monotonic_ns: monotonic_ns(),
                };
                let _ = prod.lock().push(&event);
            }
            Transport::Socket(sock) => {
                let event = FrontierEvent {
                    topic_hash,
                    stamp_sec: stamp.sec,
                    stamp_nanosec: stamp.nanosec,
                    event_type: EVENT_TAKE,
                };
                let bytes = unsafe {
                    std::slice::from_raw_parts(
                        &event as *const FrontierEvent as *const u8,
                        size_of::<FrontierEvent>(),
                    )
                };
                let _ = sock.send(bytes);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Frontier tracking (lock-free CAS max-update)
// ---------------------------------------------------------------------------

#[inline]
fn pack(sec: i32, nanosec: u32) -> u64 {
    ((sec as u32 as u64) << 32) | (nanosec as u64)
}

#[inline]
fn frontier_update(frontier: &AtomicU64, sec: i32, nanosec: u32) -> bool {
    let new = pack(sec, nanosec);
    loop {
        let old = frontier.load(Ordering::Relaxed);
        if new <= old {
            return false;
        }
        match frontier.compare_exchange_weak(old, new, Ordering::Release, Ordering::Relaxed) {
            Ok(_) => return true,
            Err(_) => continue,
        }
    }
}

// ---------------------------------------------------------------------------
// InterceptionPlugin impl
// ---------------------------------------------------------------------------

impl InterceptionPlugin for FrontierPlugin {
    fn on_publisher_init(
        &self,
        _handle: usize,
        _topic: &str,
        topic_hash: u64,
        stamp_offset: Option<usize>,
    ) {
        if stamp_offset.is_some() {
            // Pre-create the frontier entry for this topic.
            self.get_or_create_frontier(topic_hash);
        }
    }

    fn on_subscription_init(
        &self,
        _handle: usize,
        _topic: &str,
        _topic_hash: u64,
        _stamp_offset: Option<usize>,
    ) {
        // No-op for subscriptions — frontier is only tracked on publish side.
    }

    fn on_publish(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>) {
        let Some(stamp) = stamp else { return };
        let frontier = self.get_or_create_frontier(topic_hash);
        if frontier_update(frontier, stamp.sec, stamp.nanosec) {
            self.send_publish(topic_hash, &stamp, handle);
        }
    }

    fn on_take(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>) {
        let Some(stamp) = stamp else { return };
        self.send_take(topic_hash, &stamp, handle);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::net::UnixDatagram;

    #[test]
    fn frontier_event_size() {
        assert_eq!(size_of::<FrontierEvent>(), 17);
    }

    #[test]
    fn pack_ordering() {
        assert!(pack(0, 0) < pack(0, 1));
        assert!(pack(0, 999_999_999) < pack(1, 0));
        assert!(pack(1, 0) < pack(2, 0));
    }

    #[test]
    fn update_advances_frontier() {
        let frontier = AtomicU64::new(0);
        assert!(frontier_update(&frontier, 1, 0));
        assert!(frontier_update(&frontier, 1, 500));
        assert!(frontier_update(&frontier, 2, 0));
        assert!(!frontier_update(&frontier, 2, 0)); // same
        assert!(!frontier_update(&frontier, 1, 999)); // older
    }

    // -----------------------------------------------------------------------
    // Shared memory integration tests
    // -----------------------------------------------------------------------

    #[test]
    fn shm_publish_sends_event_when_frontier_advances() {
        let (shm_fd, _event_fd) = spsc_shm::create::<InterceptionEvent>(64).unwrap();
        let producer = unsafe { spsc_shm::Producer::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer = unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_raw_fd(shm_fd) }.unwrap();

        let plugin = FrontierPlugin::with_producer(Arc::new(Mutex::new(producer)));
        let hash = 0xDEAD_BEEF;

        // First publish — frontier advances, event should be written.
        plugin.on_publish(0x100, hash, Some(Stamp { sec: 1, nanosec: 0 }));
        let ev = consumer.pop().expect("should receive publish event");
        assert_eq!(ev.kind, EventKind::Publish);
        assert_eq!(ev.topic_hash, hash);
        assert_eq!(ev.stamp_sec, 1);
        assert_eq!(ev.stamp_nanosec, 0);
        assert_eq!(ev.handle, 0x100);
        assert!(ev.monotonic_ns > 0);

        unsafe {
            libc::close(shm_fd);
            libc::close(_event_fd);
        }
    }

    #[test]
    fn shm_publish_suppressed_when_frontier_does_not_advance() {
        let (shm_fd, event_fd) = spsc_shm::create::<InterceptionEvent>(64).unwrap();
        let producer = unsafe { spsc_shm::Producer::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer = unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_raw_fd(shm_fd) }.unwrap();

        let plugin = FrontierPlugin::with_producer(Arc::new(Mutex::new(producer)));
        let hash = 0xCAFE;

        // Advance frontier to (2, 0).
        plugin.on_publish(0x100, hash, Some(Stamp { sec: 2, nanosec: 0 }));
        let _ = consumer.pop(); // consume

        // Older stamp — suppressed
        plugin.on_publish(0x100, hash, Some(Stamp { sec: 1, nanosec: 999 }));
        assert!(consumer.pop().is_none(), "older stamp should be suppressed");

        // Same stamp — suppressed
        plugin.on_publish(0x100, hash, Some(Stamp { sec: 2, nanosec: 0 }));
        assert!(consumer.pop().is_none(), "same stamp should be suppressed");

        // Newer stamp — should send
        plugin.on_publish(0x100, hash, Some(Stamp { sec: 2, nanosec: 1 }));
        let ev = consumer.pop().expect("newer stamp should send");
        assert_eq!(ev.stamp_sec, 2);
        assert_eq!(ev.stamp_nanosec, 1);

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn shm_take_always_sends_event() {
        let (shm_fd, event_fd) = spsc_shm::create::<InterceptionEvent>(64).unwrap();
        let producer = unsafe { spsc_shm::Producer::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer = unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_raw_fd(shm_fd) }.unwrap();

        let plugin = FrontierPlugin::with_producer(Arc::new(Mutex::new(producer)));
        let hash = 0xBEEF;

        plugin.on_take(0x200, hash, Some(Stamp { sec: 5, nanosec: 100 }));
        let ev = consumer.pop().expect("take should always send");
        assert_eq!(ev.kind, EventKind::Take);
        assert_eq!(ev.topic_hash, hash);
        assert_eq!(ev.stamp_sec, 5);
        assert_eq!(ev.stamp_nanosec, 100);

        // Second take with same stamp — still sends.
        plugin.on_take(0x200, hash, Some(Stamp { sec: 5, nanosec: 100 }));
        assert!(consumer.pop().is_some(), "take should send even with same stamp");

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn shm_none_stamp_is_ignored() {
        let (shm_fd, event_fd) = spsc_shm::create::<InterceptionEvent>(64).unwrap();
        let producer = unsafe { spsc_shm::Producer::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer = unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_raw_fd(shm_fd) }.unwrap();

        let plugin = FrontierPlugin::with_producer(Arc::new(Mutex::new(producer)));

        plugin.on_publish(0x100, 0x1234, None);
        plugin.on_take(0x200, 0x1234, None);
        assert!(consumer.pop().is_none(), "None stamp should not send");

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    // -----------------------------------------------------------------------
    // Socket fallback tests (existing integration tests)
    // -----------------------------------------------------------------------

    use std::sync::atomic::AtomicU32;

    /// Monotonic counter to give each test a unique socket path.
    static TEST_ID: AtomicU32 = AtomicU32::new(0);

    /// Helper: parse a received 17-byte datagram into (topic_hash, sec, nanosec, event_type).
    fn parse_event(buf: &[u8; 17]) -> (u64, i32, u32, u8) {
        let topic_hash = u64::from_ne_bytes(buf[0..8].try_into().unwrap());
        let sec = i32::from_ne_bytes(buf[8..12].try_into().unwrap());
        let nanosec = u32::from_ne_bytes(buf[12..16].try_into().unwrap());
        let event_type = buf[16];
        (topic_hash, sec, nanosec, event_type)
    }

    /// Helper: create a bound receiver + a connected FrontierPlugin (socket mode).
    fn setup_socket() -> (UnixDatagram, FrontierPlugin) {
        let id = TEST_ID.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir();
        let sock_path = dir.join(format!(
            "frontier_test_{}_{}.sock",
            std::process::id(),
            id,
        ));
        let _ = std::fs::remove_file(&sock_path);

        let receiver = UnixDatagram::bind(&sock_path).expect("bind receiver");
        receiver
            .set_read_timeout(Some(std::time::Duration::from_millis(100)))
            .expect("set read timeout");

        let plugin = FrontierPlugin::connect(&sock_path).expect("connect plugin");
        let _ = std::fs::remove_file(&sock_path);

        (receiver, plugin)
    }

    fn recv_event(receiver: &UnixDatagram) -> Option<(u64, i32, u32, u8)> {
        let mut buf = [0u8; 17];
        match receiver.recv(&mut buf) {
            Ok(17) => Some(parse_event(&buf)),
            _ => None,
        }
    }

    #[test]
    fn socket_publish_sends_event_when_frontier_advances() {
        let (receiver, plugin) = setup_socket();
        let hash = 0xDEAD_BEEF;

        plugin.on_publish(0x100, hash, Some(Stamp { sec: 1, nanosec: 0 }));
        let ev = recv_event(&receiver).expect("should receive publish event");
        assert_eq!(ev.0, hash);
        assert_eq!(ev.1, 1);
        assert_eq!(ev.2, 0);
        assert_eq!(ev.3, EVENT_PUBLISH);
    }

    #[test]
    fn socket_take_always_sends_event() {
        let (receiver, plugin) = setup_socket();
        let hash = 0xBEEF;

        plugin.on_take(0x200, hash, Some(Stamp { sec: 5, nanosec: 100 }));
        let ev = recv_event(&receiver).expect("take should always send");
        assert_eq!(ev.0, hash);
        assert_eq!(ev.1, 5);
        assert_eq!(ev.2, 100);
        assert_eq!(ev.3, EVENT_TAKE);
    }

    #[test]
    fn publisher_init_precreates_frontier() {
        let (shm_fd, event_fd) = spsc_shm::create::<InterceptionEvent>(64).unwrap();
        let producer = unsafe { spsc_shm::Producer::from_raw_fd(shm_fd) }.unwrap();
        let plugin = FrontierPlugin::with_producer(Arc::new(Mutex::new(producer)));
        let hash = 0xAAAA;

        // Init with stamp_offset -> frontier should be pre-created.
        plugin.on_publisher_init(0x100, "/chatter", hash, Some(16));
        assert!(plugin.frontiers.read().contains_key(&hash));

        // Init without stamp_offset -> no frontier entry.
        let hash2 = 0xBBBB;
        plugin.on_publisher_init(0x200, "/no_stamp", hash2, None);
        assert!(!plugin.frontiers.read().contains_key(&hash2));

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn multiple_topics_independent_frontiers() {
        let (shm_fd, event_fd) = spsc_shm::create::<InterceptionEvent>(64).unwrap();
        let producer = unsafe { spsc_shm::Producer::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer = unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_raw_fd(shm_fd) }.unwrap();

        let plugin = FrontierPlugin::with_producer(Arc::new(Mutex::new(producer)));
        let hash_a = 0x1111;
        let hash_b = 0x2222;

        // Advance topic A to (10, 0).
        plugin.on_publish(0x100, hash_a, Some(Stamp { sec: 10, nanosec: 0 }));
        let _ = consumer.pop();

        // Advance topic B to (5, 0) — should send (independent frontier).
        plugin.on_publish(0x200, hash_b, Some(Stamp { sec: 5, nanosec: 0 }));
        let ev = consumer.pop().expect("topic B should send independently");
        assert_eq!(ev.topic_hash, hash_b);
        assert_eq!(ev.stamp_sec, 5);

        // Topic A at (9, 0) — should NOT send (behind its own frontier).
        plugin.on_publish(0x100, hash_a, Some(Stamp { sec: 9, nanosec: 0 }));
        assert!(consumer.pop().is_none());

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }
}
