//! Reading declared scalar fields on publish (play_launch phase 102 M0).
//!
//! A contract states predicates over message fields (a hazard's or a value
//! function's `when:`, and phase 100's `modes.<m>.commands`). Checking one
//! at run time needs the field's value in each message, and nothing else of
//! the message. play_launch names the fields it needs in
//! `PLAY_LAUNCH_INTERCEPTION_FIELDS`, one `topic<TAB>field.path` per line;
//! the line's position is the field's index, which the consumer reads back
//! from the same list. At publisher init the dotted path is resolved once by
//! rosidl introspection to an offset and a type, exactly as `tf.rs` resolves
//! a `TFMessage`; on publish each resolved field is one load.
//!
//! Reach: a scalar at a dotted path through nested messages (no sequences or
//! arrays). A numeric, bool or char field travels as its value; a string
//! travels as the FNV-1a hash of its bytes, which is enough for `equals` and
//! `not_equals`. A path that does not resolve to such a field is skipped:
//! the contract checker already reports a field the `.msg` does not have.

use std::{
    collections::HashMap,
    ffi::{CStr, c_char},
    sync::{
        OnceLock,
        atomic::{AtomicBool, Ordering},
    },
};

use parking_lot::RwLock;
use rcl_interception_sys::*;

/// The environment variable naming the fields to read.
pub const ENV: &str = "PLAY_LAUNCH_INTERCEPTION_FIELDS";

/// How a value is encoded in an event's 64 value bits.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueKind {
    /// `f64::to_bits`; float32 is widened.
    Float = 1,
    /// `i64 as u64`; every signed integer is widened.
    Int = 2,
    /// Every unsigned integer, char and octet, widened.
    Uint = 3,
    /// 0 or 1.
    Bool = 4,
    /// FNV-1a of the string's bytes.
    StrHash = 5,
}

/// Where one field is, and what it is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FieldLayout {
    /// The field's index in `PLAY_LAUNCH_INTERCEPTION_FIELDS`.
    pub index: u8,
    pub offset: usize,
    /// The rosidl introspection type id.
    pub type_id: u8,
}

/// One requested field, in list order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Request {
    pub topic: String,
    pub path: String,
}

/// Parse the variable's text: one `topic<TAB>field.path` per line. A
/// malformed line keeps its index (so the consumer's numbering still
/// matches) and resolves to nothing. At most 256 entries, the index being
/// one byte.
pub fn parse(text: &str) -> Vec<Request> {
    text.lines()
        .take(256)
        .map(|line| {
            let (topic, path) = line.split_once('\t').unwrap_or(("", ""));
            Request {
                topic: topic.trim().to_string(),
                path: path.trim().to_string(),
            }
        })
        .collect()
}

fn requests() -> &'static [Request] {
    static REQUESTS: OnceLock<Vec<Request>> = OnceLock::new();
    REQUESTS.get_or_init(|| std::env::var(ENV).map(|t| parse(&t)).unwrap_or_default())
}

unsafe fn members_of(ts: *const rosidl_message_type_support_t) -> Option<&'static MessageMembers> {
    if ts.is_null() {
        return None;
    }
    for id in [
        TYPESUPPORT_INTROSPECTION_C_IDENTIFIER,
        TYPESUPPORT_INTROSPECTION_CPP_IDENTIFIER,
    ] {
        let func = unsafe { (*ts).func }?;
        let handle = unsafe { func(ts, id.as_ptr() as *const c_char) };
        if !handle.is_null() && !unsafe { (*handle).data }.is_null() {
            return Some(unsafe { &*((*handle).data as *const MessageMembers) });
        }
    }
    None
}

fn member<'a>(m: &'a MessageMembers, name: &str) -> Option<&'a MessageMember> {
    if m.members_.is_null() {
        return None;
    }
    let slice = unsafe { std::slice::from_raw_parts(m.members_, m.member_count_ as usize) };
    slice.iter().find(|f| {
        !f.name_.is_null() && unsafe { CStr::from_ptr(f.name_) }.to_bytes() == name.as_bytes()
    })
}

/// Resolve a dotted path within `msg` to a scalar field's offset and type.
///
/// # Safety
///
/// `msg` must be valid introspection data, and so must every nested
/// message type support it names.
pub unsafe fn resolve(msg: &MessageMembers, path: &str) -> Option<(usize, u8)> {
    let mut members = msg;
    let mut offset = 0usize;
    let mut parts = path.split('.').peekable();
    while let Some(name) = parts.next() {
        let field = member(members, name)?;
        if field.is_array_ {
            return None;
        }
        offset += field.offset_ as usize;
        if parts.peek().is_none() {
            return value_kind(field.type_id_).map(|_| (offset, field.type_id_));
        }
        if field.type_id_ != ROS_TYPE_MESSAGE {
            return None;
        }
        members = unsafe { members_of(field.members_) }?;
    }
    None
}

/// How a rosidl type id travels, or `None` for one that is not a scalar.
pub fn value_kind(type_id: u8) -> Option<ValueKind> {
    Some(match type_id {
        1 | 2 => ValueKind::Float,
        9 | 11 | 13 | 15 => ValueKind::Int,
        4 | 7 | 8 | 10 | 12 | 14 => ValueKind::Uint,
        6 => ValueKind::Bool,
        16 => ValueKind::StrHash,
        _ => return None,
    })
}

fn fnv(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

/// Read one field: its encoding and its 64 value bits.
///
/// # Safety
///
/// `msg` must be a message of the type `layout` was resolved from.
pub unsafe fn read(layout: &FieldLayout, msg: *const u8) -> Option<(ValueKind, u64)> {
    let kind = value_kind(layout.type_id)?;
    let at = unsafe { msg.add(layout.offset) };
    let bits = unsafe {
        match layout.type_id {
            1 => (*(at as *const f32) as f64).to_bits(),
            2 => (*(at as *const f64)).to_bits(),
            9 => *(at as *const i8) as i64 as u64,
            11 => *(at as *const i16) as i64 as u64,
            13 => *(at as *const i32) as i64 as u64,
            15 => *(at as *const i64) as u64,
            4 | 7 | 8 => *at as u64,
            10 => *(at as *const u16) as u64,
            12 => *(at as *const u32) as u64,
            14 => *(at as *const u64),
            6 => u64::from(*at != 0),
            // Pointer then length, in both C and C++ messages (see tf.rs).
            16 => {
                let ptr = *(at as *const *const u8);
                let len = *(at.add(8) as *const usize);
                if ptr.is_null() || len > 4096 {
                    fnv(b"")
                } else {
                    fnv(std::slice::from_raw_parts(ptr, len))
                }
            }
            _ => return None,
        }
    };
    Some((kind, bits))
}

static ANY: AtomicBool = AtomicBool::new(false);
static PUBLISHERS: RwLock<Option<HashMap<usize, Vec<FieldLayout>>>> = RwLock::new(None);

/// At publisher init: resolve every requested field of `topic` and
/// remember them. Nothing is done for a topic no request names.
///
/// # Safety
///
/// `ts` must be the publisher's valid message type support.
pub unsafe fn register(publisher: usize, topic: &str, ts: *const rosidl_message_type_support_t) {
    let wanted: Vec<(usize, &Request)> = requests()
        .iter()
        .enumerate()
        .filter(|(_, r)| r.topic == topic && !r.path.is_empty())
        .collect();
    if wanted.is_empty() {
        return;
    }
    let Some(msg) = (unsafe { members_of(ts) }) else {
        return;
    };
    let layouts: Vec<FieldLayout> = wanted
        .into_iter()
        .filter_map(|(index, r)| {
            let (offset, type_id) = unsafe { resolve(msg, &r.path) }?;
            Some(FieldLayout {
                index: index as u8,
                offset,
                type_id,
            })
        })
        .collect();
    if layouts.is_empty() {
        return;
    }
    PUBLISHERS
        .write()
        .get_or_insert_with(HashMap::new)
        .insert(publisher, layouts);
    ANY.store(true, Ordering::Release);
}

/// Call `f` with each field to read on `publisher`, under the read lock and
/// without allocating. One load in a process that reads none.
pub fn for_each(publisher: usize, mut f: impl FnMut(&FieldLayout)) {
    if !ANY.load(Ordering::Acquire) {
        return;
    }
    if let Some(layouts) = PUBLISHERS.read().as_ref().and_then(|m| m.get(&publisher)) {
        layouts.iter().for_each(&mut f);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_keeps_line_positions() {
        let r = parse("/a\tdata\nmalformed\n/b\tstatus.mode");
        assert_eq!(r.len(), 3);
        assert_eq!(r[0].topic, "/a");
        assert_eq!(r[0].path, "data");
        assert_eq!(r[1].path, "");
        assert_eq!(r[2].path, "status.mode");
    }

    // A nested message type support for the tests: `func` returns a handle
    // whose data is `INNER`.
    static mut INNER: Option<MessageMembers> = None;
    static mut INNER_HANDLE: Option<rosidl_message_type_support_t> = None;

    unsafe extern "C" fn inner_func(
        _ts: *const rosidl_message_type_support_t,
        _id: *const c_char,
    ) -> *const rosidl_message_type_support_t {
        #[allow(static_mut_refs)]
        unsafe {
            INNER_HANDLE.as_ref().unwrap() as *const _
        }
    }

    #[test]
    fn resolves_nested_scalars_and_refuses_the_rest() {
        let mode = c"mode";
        let speed = c"speed";
        let status = c"status";
        let flag = c"flag";
        let list = c"list";
        unsafe {
            let mut inner: [MessageMember; 2] = std::mem::zeroed();
            inner[0].name_ = mode.as_ptr();
            inner[0].type_id_ = 16;
            inner[0].offset_ = 8;
            inner[1].name_ = speed.as_ptr();
            inner[1].type_id_ = 2;
            inner[1].offset_ = 40;
            let mut inner_m: MessageMembers = std::mem::zeroed();
            inner_m.member_count_ = 2;
            inner_m.members_ = inner.as_ptr();
            INNER = Some(inner_m);
            let mut handle: rosidl_message_type_support_t = std::mem::zeroed();
            #[allow(static_mut_refs)]
            {
                handle.data = INNER.as_ref().unwrap() as *const _ as *const std::ffi::c_void;
            }
            INNER_HANDLE = Some(handle);
            let mut ts: rosidl_message_type_support_t = std::mem::zeroed();
            ts.func = Some(inner_func);

            let mut outer: [MessageMember; 3] = std::mem::zeroed();
            outer[0].name_ = flag.as_ptr();
            outer[0].type_id_ = 6;
            outer[0].offset_ = 0;
            outer[1].name_ = status.as_ptr();
            outer[1].type_id_ = ROS_TYPE_MESSAGE;
            outer[1].offset_ = 16;
            outer[1].members_ = &ts;
            outer[2].name_ = list.as_ptr();
            outer[2].type_id_ = 2;
            outer[2].is_array_ = true;
            outer[2].offset_ = 100;
            let mut outer_m: MessageMembers = std::mem::zeroed();
            outer_m.member_count_ = 3;
            outer_m.members_ = outer.as_ptr();

            assert_eq!(resolve(&outer_m, "flag"), Some((0, 6)));
            assert_eq!(resolve(&outer_m, "status.mode"), Some((24, 16)));
            assert_eq!(resolve(&outer_m, "status.speed"), Some((56, 2)));
            assert_eq!(
                resolve(&outer_m, "status"),
                None,
                "a message is not a scalar"
            );
            assert_eq!(resolve(&outer_m, "list"), None, "arrays are out of reach");
            assert_eq!(resolve(&outer_m, "flag.x"), None);
            assert_eq!(resolve(&outer_m, "missing"), None);
        }
    }

    #[test]
    fn reads_each_encoding() {
        #[repr(C)]
        struct Msg {
            flag: bool,
            _p: [u8; 7],
            speed: f64,
            count: i32,
            small: f32,
            name_ptr: *const u8,
            name_len: usize,
        }
        let name = b"MANUAL";
        let m = Msg {
            flag: true,
            _p: [0; 7],
            speed: -1.5,
            count: -3,
            small: 0.25,
            name_ptr: name.as_ptr(),
            name_len: name.len(),
        };
        let base = &m as *const Msg as *const u8;
        let at = |offset: usize, type_id: u8| unsafe {
            read(
                &FieldLayout {
                    index: 0,
                    offset,
                    type_id,
                },
                base,
            )
        };
        assert_eq!(at(0, 6), Some((ValueKind::Bool, 1)));
        assert_eq!(at(8, 2), Some((ValueKind::Float, (-1.5f64).to_bits())));
        assert_eq!(at(16, 13), Some((ValueKind::Int, (-3i64) as u64)));
        assert_eq!(at(20, 1), Some((ValueKind::Float, 0.25f64.to_bits())));
        assert_eq!(at(24, 16), Some((ValueKind::StrHash, fnv(b"MANUAL"))));
    }

    /// The hot-path cost, printed: one resolved field read per publish.
    #[test]
    fn read_cost() {
        let v = 3.0f64;
        let layout = FieldLayout {
            index: 0,
            offset: 0,
            type_id: 2,
        };
        let n = 1_000_000u64;
        let t = std::time::Instant::now();
        let mut acc = 0u64;
        for _ in 0..n {
            acc ^= unsafe { read(&layout, std::hint::black_box(&v as *const f64 as *const u8)) }
                .unwrap()
                .1;
        }
        std::hint::black_box(acc);
        eprintln!(
            "field read: {:.1} ns",
            t.elapsed().as_nanos() as f64 / n as f64
        );
    }

    /// What the publish hook adds for one declared field: the publisher
    /// lookup, the read, the event and the ring push (`cargo test --release
    /// -- --ignored field_hook_cost --nocapture`). Phase 102 M0 measured it.
    #[test]
    #[ignore]
    fn field_hook_cost() {
        use std::sync::Arc;

        use parking_lot::Mutex;

        use crate::{event::InterceptionEvent, plugin::InterceptionPlugin};

        let (shm_fd, _event_fd) = spsc_shm::create::<InterceptionEvent>(1 << 16).unwrap();
        let producer = unsafe { spsc_shm::Producer::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer =
            unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_raw_fd(shm_fd) }.unwrap();
        let plugin = crate::plugins::tf::TfPlugin::new(Arc::new(Mutex::new(producer)));
        // 64 publishers, one field each, as `register` would leave them.
        for id in 0..64usize {
            PUBLISHERS.write().get_or_insert_with(HashMap::new).insert(
                0x1000 + id,
                vec![FieldLayout {
                    index: 0,
                    offset: 0,
                    type_id: 2,
                }],
            );
        }
        ANY.store(true, Ordering::Release);
        let v = 3.0f64;
        let msg = &v as *const f64 as *const u8;
        let n = 200_000u64;
        let mut total = std::time::Duration::ZERO;
        for round in 0..(n / 1000) {
            let t = std::time::Instant::now();
            for i in 0..1000u64 {
                let id = 0x1000 + ((round * 1000 + i) % 64) as usize;
                for_each(id, |layout| {
                    if let Some((kind, bits)) = unsafe { read(layout, msg) } {
                        plugin.on_field(id, 9, layout.index, kind, bits, None);
                    }
                });
            }
            total += t.elapsed();
            while consumer.pop().is_some() {}
        }
        eprintln!(
            "field hook overhead: {:.1} ns per field",
            total.as_nanos() as f64 / n as f64
        );
    }
}
