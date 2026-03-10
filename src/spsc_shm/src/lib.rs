//! Zero-copy SPSC ring buffer over Linux shared memory.
//!
//! Provides a typed, single-producer single-consumer ring buffer backed by
//! `memfd_create` + `mmap`, with `eventfd` for async wakeup signaling.
//!
//! # Usage
//!
//! ```no_run
//! // Parent process (consumer):
//! let (shm_fd, event_fd) = spsc_shm::create::<u64>(1024).unwrap();
//! // pass shm_fd and event_fd to child via env vars / fork+exec
//! let mut consumer = unsafe { spsc_shm::Consumer::<u64>::from_raw_fd(shm_fd) }.unwrap();
//! while let Some(val) = consumer.pop() { /* ... */ }
//!
//! // Child process (producer):
//! let mut producer = unsafe { spsc_shm::Producer::<u64>::from_raw_fd(shm_fd) }.unwrap();
//! producer.push(&42u64).unwrap();
//! ```
//!
//! # Safety
//!
//! The ring buffer is **single-producer, single-consumer**. Concurrent access
//! from multiple producers or multiple consumers is undefined behavior. If you
//! need multi-producer access, wrap `Producer` in a `Mutex`.

use std::io;
use std::marker::PhantomData;
use std::os::fd::RawFd;
use std::sync::atomic::{AtomicU64, Ordering};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Magic value stored in the header for validation: "SPSC" as little-endian u64.
const MAGIC: u64 = 0x4350_5350_5343; // arbitrary, just for sanity checks

/// Header is 128 bytes: 2 cache lines to avoid false sharing between producer
/// and consumer indices.
const HEADER_SIZE: usize = 128;

// Field offsets within the header (all u64-aligned).
const WRITE_IDX_OFF: usize = 0;
#[allow(dead_code)]
const READ_IDX_OFF: usize = 64;
const CAPACITY_OFF: usize = 72;
const ELEMENT_SIZE_OFF: usize = 80;
const MAGIC_OFF: usize = 88;

// ---------------------------------------------------------------------------
// Error types
// ---------------------------------------------------------------------------

/// Returned when the ring buffer is full.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Full;

impl std::fmt::Display for Full {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ring buffer full")
    }
}

impl std::error::Error for Full {}

// ---------------------------------------------------------------------------
// Header (runtime view — accessed via raw pointer into mmap'd region)
// ---------------------------------------------------------------------------

/// Ring buffer header in shared memory.
///
/// Cache-line padded: `write_idx` (cache line 0, producer-owned) and
/// `read_idx` (cache line 1, consumer-owned) are on separate 64-byte
/// boundaries to avoid false sharing.
///
/// ```text
/// Offset  0: write_idx (AtomicU64)    -- producer writes, consumer reads
/// Offset  8: [padding 56 bytes]
/// Offset 64: read_idx  (AtomicU64)    -- consumer writes, producer reads
/// Offset 72: capacity  (u64)          -- immutable after creation
/// Offset 80: element_size (u64)       -- immutable after creation
/// Offset 88: magic     (u64)          -- validation tag
/// Offset 96: [padding 32 bytes]
/// Offset 128: slots[0..capacity]
/// ```
#[repr(C)]
struct RingHeader {
    // -- Cache line 0 (written by producer) --
    write_idx: AtomicU64,
    _pad0: [u8; 56],
    // -- Cache line 1 (written by consumer, metadata read-only) --
    read_idx: AtomicU64,
    capacity: u64,
    element_size: u64,
    magic: u64,
    _pad1: [u8; 32],
}

const _: () = assert!(size_of::<RingHeader>() == HEADER_SIZE);

// ---------------------------------------------------------------------------
// create()
// ---------------------------------------------------------------------------

/// Create a shared memory ring buffer and an eventfd for signaling.
///
/// Returns `(shm_fd, event_fd)` as raw file descriptors. The caller owns
/// both fds and is responsible for closing them (or passing them to a child
/// process).
///
/// The shared memory region is sized for `capacity` slots of type `T`.
/// Both indices are initialized to 0 (empty ring).
///
/// # Panics
///
/// Panics if `capacity` is 0 or `size_of::<T>()` is 0.
pub fn create<T: Copy>(capacity: usize) -> io::Result<(RawFd, RawFd)> {
    assert!(capacity > 0, "capacity must be > 0");
    assert!(size_of::<T>() > 0, "element size must be > 0");

    let element_size = size_of::<T>();
    let total_size = HEADER_SIZE + capacity * element_size;

    // 1. memfd_create — anonymous shared memory
    let shm_fd = unsafe { libc::memfd_create(c"spsc_shm".as_ptr(), 0) };
    if shm_fd < 0 {
        return Err(io::Error::last_os_error());
    }

    // 2. ftruncate to the required size
    if unsafe { libc::ftruncate(shm_fd, total_size as libc::off_t) } != 0 {
        unsafe { libc::close(shm_fd) };
        return Err(io::Error::last_os_error());
    }

    // 3. mmap to write the header
    let ptr = unsafe {
        libc::mmap(
            std::ptr::null_mut(),
            total_size,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            shm_fd,
            0,
        )
    };
    if ptr == libc::MAP_FAILED {
        unsafe { libc::close(shm_fd) };
        return Err(io::Error::last_os_error());
    }

    // 4. Initialize header: zero everything, then write metadata
    unsafe {
        std::ptr::write_bytes(ptr as *mut u8, 0, HEADER_SIZE);
        // write_idx and read_idx are 0 (from zero-init)
        let base = ptr as *mut u8;
        std::ptr::write(base.add(CAPACITY_OFF) as *mut u64, capacity as u64);
        std::ptr::write(base.add(ELEMENT_SIZE_OFF) as *mut u64, element_size as u64);
        std::ptr::write(base.add(MAGIC_OFF) as *mut u64, MAGIC);
    }

    // 5. munmap — creator doesn't keep the mapping
    unsafe { libc::munmap(ptr, total_size) };

    // 6. eventfd for async wakeup signaling
    let event_fd = unsafe { libc::eventfd(0, libc::EFD_NONBLOCK) };
    if event_fd < 0 {
        unsafe { libc::close(shm_fd) };
        return Err(io::Error::last_os_error());
    }

    Ok((shm_fd, event_fd))
}

// ---------------------------------------------------------------------------
// Helper: mmap a shm_fd and validate the header
// ---------------------------------------------------------------------------

struct Mapping {
    base: *mut u8,
    map_len: usize,
    capacity: u64,
}

/// Map the shared memory region and validate the header.
unsafe fn map_and_validate<T: Copy>(shm_fd: RawFd) -> io::Result<Mapping> {
    // fstat to get the total size
    let mut stat: libc::stat = unsafe { std::mem::zeroed() };
    if unsafe { libc::fstat(shm_fd, &mut stat) } != 0 {
        return Err(io::Error::last_os_error());
    }
    let total_size = stat.st_size as usize;
    if total_size < HEADER_SIZE {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "shared memory region too small for header",
        ));
    }

    // mmap the entire region
    let ptr = unsafe {
        libc::mmap(
            std::ptr::null_mut(),
            total_size,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            shm_fd,
            0,
        )
    };
    if ptr == libc::MAP_FAILED {
        return Err(io::Error::last_os_error());
    }

    let base = ptr as *mut u8;

    // Validate magic
    let magic = unsafe { std::ptr::read(base.add(MAGIC_OFF) as *const u64) };
    if magic != MAGIC {
        unsafe { libc::munmap(ptr, total_size) };
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid magic in shared memory header",
        ));
    }

    // Validate element size
    let element_size = unsafe { std::ptr::read(base.add(ELEMENT_SIZE_OFF) as *const u64) };
    if element_size != size_of::<T>() as u64 {
        unsafe { libc::munmap(ptr, total_size) };
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "element size mismatch: header says {}, type is {}",
                element_size,
                size_of::<T>()
            ),
        ));
    }

    // Read capacity
    let capacity = unsafe { std::ptr::read(base.add(CAPACITY_OFF) as *const u64) };
    let expected_size = HEADER_SIZE + (capacity as usize) * (element_size as usize);
    if total_size != expected_size {
        unsafe { libc::munmap(ptr, total_size) };
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "size mismatch: file is {} bytes, expected {}",
                total_size, expected_size
            ),
        ));
    }

    Ok(Mapping {
        base,
        map_len: total_size,
        capacity,
    })
}

// ---------------------------------------------------------------------------
// Producer
// ---------------------------------------------------------------------------

/// Write-side handle for the SPSC ring buffer.
///
/// # Thread safety
///
/// `Producer` is `Send` but not `Sync`. Only one thread may call `push()`
/// at a time. For multi-threaded access, wrap in a `Mutex`.
pub struct Producer<T: Copy> {
    base: *mut u8,
    map_len: usize,
    capacity: u64,
    /// Cached consumer read_idx to avoid cross-cache-line reads on every push.
    cached_read: u64,
    _marker: PhantomData<T>,
}

impl<T: Copy> std::fmt::Debug for Producer<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Producer")
            .field("capacity", &self.capacity)
            .finish()
    }
}

// SAFETY: The mmap'd region is process-local. Only one Producer exists per
// ring buffer. Sending to another thread is fine; concurrent access is not
// (enforced by &mut self on push).
unsafe impl<T: Copy> Send for Producer<T> {}

impl<T: Copy> Producer<T> {
    /// Create a producer from a shared memory file descriptor.
    ///
    /// # Safety
    ///
    /// The fd must have been created by [`create::<T>()`](create) with the
    /// same type `T`. The caller must ensure that only one `Producer` exists
    /// for this shared memory region.
    pub unsafe fn from_raw_fd(shm_fd: RawFd) -> io::Result<Self> {
        let mapping = unsafe { map_and_validate::<T>(shm_fd)? };
        Ok(Self {
            base: mapping.base,
            map_len: mapping.map_len,
            capacity: mapping.capacity,
            cached_read: 0,
            _marker: PhantomData,
        })
    }

    /// Push an item into the ring buffer.
    ///
    /// Returns `Err(Full)` if the ring is full. The caller should drop the
    /// event (best-effort semantics).
    pub fn push(&mut self, item: &T) -> Result<(), Full> {
        let w = self.write_idx().load(Ordering::Relaxed);

        // Check if there's space, using cached read_idx first
        if w.wrapping_sub(self.cached_read) >= self.capacity {
            // Re-read actual read_idx (crosses cache line)
            self.cached_read = self.read_idx().load(Ordering::Acquire);
            if w.wrapping_sub(self.cached_read) >= self.capacity {
                return Err(Full);
            }
        }

        // Write the item to the slot
        let slot_idx = (w % self.capacity) as usize;
        unsafe {
            let slot_ptr = self.slot_ptr(slot_idx);
            std::ptr::write(slot_ptr, *item);
        }

        // Publish the write (Release ensures the slot write is visible before
        // the consumer sees the new write_idx)
        self.write_idx().store(w.wrapping_add(1), Ordering::Release);

        Ok(())
    }

    /// Number of slots in the ring buffer.
    pub fn capacity(&self) -> usize {
        self.capacity as usize
    }

    #[inline]
    fn write_idx(&self) -> &AtomicU64 {
        unsafe { &*(self.base.add(WRITE_IDX_OFF) as *const AtomicU64) }
    }

    #[inline]
    fn read_idx(&self) -> &AtomicU64 {
        unsafe { &*(self.base.add(READ_IDX_OFF) as *const AtomicU64) }
    }

    #[inline]
    unsafe fn slot_ptr(&self, idx: usize) -> *mut T {
        unsafe { self.base.add(HEADER_SIZE + idx * size_of::<T>()) as *mut T }
    }
}

impl<T: Copy> Drop for Producer<T> {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.base as *mut libc::c_void, self.map_len);
        }
    }
}

// ---------------------------------------------------------------------------
// Consumer
// ---------------------------------------------------------------------------

/// Read-side handle for the SPSC ring buffer.
///
/// # Thread safety
///
/// `Consumer` is `Send` but not `Sync`. Only one thread may call `pop()`
/// at a time.
pub struct Consumer<T: Copy> {
    base: *mut u8,
    map_len: usize,
    capacity: u64,
    /// Cached producer write_idx to avoid cross-cache-line reads on every pop.
    cached_write: u64,
    _marker: PhantomData<T>,
}

impl<T: Copy> std::fmt::Debug for Consumer<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Consumer")
            .field("capacity", &self.capacity)
            .finish()
    }
}

unsafe impl<T: Copy> Send for Consumer<T> {}

impl<T: Copy> Consumer<T> {
    /// Create a consumer from a shared memory file descriptor.
    ///
    /// # Safety
    ///
    /// The fd must have been created by [`create::<T>()`](create) with the
    /// same type `T`. The caller must ensure that only one `Consumer` exists
    /// for this shared memory region.
    pub unsafe fn from_raw_fd(shm_fd: RawFd) -> io::Result<Self> {
        let mapping = unsafe { map_and_validate::<T>(shm_fd)? };
        Ok(Self {
            base: mapping.base,
            map_len: mapping.map_len,
            capacity: mapping.capacity,
            cached_write: 0,
            _marker: PhantomData,
        })
    }

    /// Pop an item from the ring buffer, or `None` if empty.
    pub fn pop(&mut self) -> Option<T> {
        let r = self.read_idx().load(Ordering::Relaxed);

        // Check if there's data, using cached write_idx first
        if r == self.cached_write {
            // Re-read actual write_idx (crosses cache line)
            self.cached_write = self.write_idx().load(Ordering::Acquire);
            if r == self.cached_write {
                return None;
            }
        }

        // Read the item from the slot
        let slot_idx = (r % self.capacity) as usize;
        let item = unsafe {
            let slot_ptr = self.slot_ptr(slot_idx);
            std::ptr::read(slot_ptr)
        };

        // Publish the read (Release ensures the slot read completed before
        // the producer sees the freed slot)
        self.read_idx().store(r.wrapping_add(1), Ordering::Release);

        Some(item)
    }

    /// Number of slots in the ring buffer.
    pub fn capacity(&self) -> usize {
        self.capacity as usize
    }

    #[inline]
    fn write_idx(&self) -> &AtomicU64 {
        unsafe { &*(self.base.add(WRITE_IDX_OFF) as *const AtomicU64) }
    }

    #[inline]
    fn read_idx(&self) -> &AtomicU64 {
        unsafe { &*(self.base.add(READ_IDX_OFF) as *const AtomicU64) }
    }

    #[inline]
    unsafe fn slot_ptr(&self, idx: usize) -> *const T {
        unsafe { self.base.add(HEADER_SIZE + idx * size_of::<T>()) as *const T }
    }
}

impl<T: Copy> Drop for Consumer<T> {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.base as *mut libc::c_void, self.map_len);
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[repr(C)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct TestEvent {
        kind: u8,
        _pad: [u8; 3],
        value: u32,
        extra: u64,
    }

    #[test]
    fn header_size() {
        assert_eq!(HEADER_SIZE, 128);
        assert_eq!(size_of::<RingHeader>(), 128);
    }

    #[test]
    fn create_returns_valid_fds() {
        let (shm_fd, event_fd) = create::<TestEvent>(16).unwrap();
        assert!(shm_fd >= 0);
        assert!(event_fd >= 0);
        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn producer_consumer_round_trip() {
        let (shm_fd, _event_fd) = create::<u64>(8).unwrap();

        let mut producer = unsafe { Producer::<u64>::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer = unsafe { Consumer::<u64>::from_raw_fd(shm_fd) }.unwrap();

        // Empty ring
        assert!(consumer.pop().is_none());

        // Push and pop single item
        producer.push(&42u64).unwrap();
        assert_eq!(consumer.pop(), Some(42u64));
        assert!(consumer.pop().is_none());

        unsafe {
            libc::close(shm_fd);
            libc::close(_event_fd);
        }
    }

    #[test]
    fn push_pop_sequence() {
        let (shm_fd, event_fd) = create::<u64>(4).unwrap();

        let mut producer = unsafe { Producer::<u64>::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer = unsafe { Consumer::<u64>::from_raw_fd(shm_fd) }.unwrap();

        // Fill and drain multiple times (tests wrapping)
        for round in 0..3 {
            let base = round * 4;
            for i in 0..4 {
                producer.push(&(base + i)).unwrap();
            }
            for i in 0..4 {
                assert_eq!(consumer.pop(), Some(base + i));
            }
            assert!(consumer.pop().is_none());
        }

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn overflow_returns_full() {
        let (shm_fd, event_fd) = create::<u64>(4).unwrap();

        let mut producer = unsafe { Producer::<u64>::from_raw_fd(shm_fd) }.unwrap();

        // Fill the ring
        for i in 0..4 {
            producer.push(&i).unwrap();
        }

        // Next push should fail
        assert_eq!(producer.push(&99), Err(Full));

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn typed_events() {
        let (shm_fd, event_fd) = create::<TestEvent>(16).unwrap();

        let mut producer = unsafe { Producer::<TestEvent>::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer = unsafe { Consumer::<TestEvent>::from_raw_fd(shm_fd) }.unwrap();

        let event = TestEvent {
            kind: 2,
            _pad: [0; 3],
            value: 0xDEAD,
            extra: 0xCAFE_BABE,
        };
        producer.push(&event).unwrap();

        let got = consumer.pop().unwrap();
        assert_eq!(got, event);

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn element_size_mismatch_rejected() {
        let (shm_fd, event_fd) = create::<u64>(8).unwrap();

        // Try to open as u32 — should fail with size mismatch
        let result = unsafe { Producer::<u32>::from_raw_fd(shm_fd) };
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.to_string().contains("element size mismatch"));

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn capacity_reported_correctly() {
        let (shm_fd, event_fd) = create::<u64>(256).unwrap();

        let producer = unsafe { Producer::<u64>::from_raw_fd(shm_fd) }.unwrap();
        let consumer = unsafe { Consumer::<u64>::from_raw_fd(shm_fd) }.unwrap();

        assert_eq!(producer.capacity(), 256);
        assert_eq!(consumer.capacity(), 256);

        // Need to drop before closing fd (unmap first)
        drop(producer);
        drop(consumer);
        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn concurrent_producer_consumer() {
        let (shm_fd, event_fd) = create::<u64>(1024).unwrap();

        let mut producer = unsafe { Producer::<u64>::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer = unsafe { Consumer::<u64>::from_raw_fd(shm_fd) }.unwrap();

        let n = 10_000u64;

        // Producer thread
        let handle = std::thread::spawn(move || {
            for i in 0..n {
                // Spin-retry on Full
                while producer.push(&i).is_err() {
                    std::thread::yield_now();
                }
            }
        });

        // Consumer in main thread
        let mut received = Vec::with_capacity(n as usize);
        while received.len() < n as usize {
            if let Some(val) = consumer.pop() {
                received.push(val);
            } else {
                std::thread::yield_now();
            }
        }

        handle.join().unwrap();

        // Verify order
        for (i, &val) in received.iter().enumerate() {
            assert_eq!(val, i as u64, "mismatch at index {}", i);
        }

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }
}
