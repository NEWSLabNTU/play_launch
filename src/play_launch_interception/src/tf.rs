//! Reading `tf2_msgs/msg/TFMessage` on publish (big-system-observability
//! §H): which frame pairs a publisher sends, how often, and whether the
//! transform moves. Resolved once per publisher from rosidl introspection,
//! so the same code reads a C message (rclpy, rclc) and a C++ one (rclcpp):
//! the sequence goes through the member's own `size_function` /
//! `get_const_function`, and a string is read as its first two words,
//! pointer then length, which is the layout of both `rosidl_runtime_c__String`
//! and libstdc++'s `std::string`.

use std::{
    collections::HashMap,
    ffi::{CStr, c_char, c_void},
    sync::atomic::{AtomicBool, Ordering},
};

use parking_lot::RwLock;
use rcl_interception_sys::*;

/// Where a `TFMessage`'s pieces are.
#[derive(Debug, Clone, Copy)]
pub struct TfLayout {
    transforms_offset: usize,
    size_fn: unsafe extern "C" fn(*const c_void) -> usize,
    get_fn: unsafe extern "C" fn(*const c_void, usize) -> *const c_void,
    /// Within one `TransformStamped`.
    stamp_offset: usize,
    frame_id_offset: usize,
    child_offset: usize,
    transform_offset: usize,
}

/// One transform as published.
#[derive(Debug, Clone, Copy)]
pub struct TfSample<'a> {
    pub parent: &'a str,
    pub child: &'a str,
    pub sec: i32,
    pub nanosec: u32,
    /// FNV-1a over the transform's seven doubles.
    pub value_hash: u64,
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

fn member<'a>(m: &'a MessageMembers, name: &[u8]) -> Option<&'a MessageMember> {
    if m.members_.is_null() {
        return None;
    }
    let slice = unsafe { std::slice::from_raw_parts(m.members_, m.member_count_ as usize) };
    slice
        .iter()
        .find(|f| !f.name_.is_null() && unsafe { CStr::from_ptr(f.name_) }.to_bytes() == name)
}

/// The layout of a `TFMessage` type support, or `None` for any other type.
///
/// # Safety
///
/// `ts` must be a valid message type support.
pub unsafe fn tf_layout(ts: *const rosidl_message_type_support_t) -> Option<TfLayout> {
    let msg = unsafe { members_of(ts) }?;
    let transforms = member(msg, b"transforms")?;
    if !transforms.is_array_ || transforms.type_id_ != ROS_TYPE_MESSAGE {
        return None;
    }
    let stamped = unsafe { members_of(transforms.members_) }?;
    let header = member(stamped, b"header")?;
    let header_m = unsafe { members_of(header.members_) }?;
    let stamp = member(header_m, b"stamp")?;
    let frame_id = member(header_m, b"frame_id")?;
    let child = member(stamped, b"child_frame_id")?;
    let transform = member(stamped, b"transform")?;
    Some(TfLayout {
        transforms_offset: transforms.offset_ as usize,
        size_fn: transforms.size_function?,
        get_fn: transforms.get_const_function?,
        stamp_offset: header.offset_ as usize + stamp.offset_ as usize,
        frame_id_offset: header.offset_ as usize + frame_id.offset_ as usize,
        child_offset: child.offset_ as usize,
        transform_offset: transform.offset_ as usize,
    })
}

/// A string member: pointer then length, in both C and C++ messages.
unsafe fn read_str<'a>(at: *const u8) -> &'a str {
    let ptr = unsafe { *(at as *const *const u8) };
    let len = unsafe { *(at.add(8) as *const usize) };
    if ptr.is_null() || len > 4096 {
        return "";
    }
    std::str::from_utf8(unsafe { std::slice::from_raw_parts(ptr, len) }).unwrap_or("")
}

fn fnv(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

/// Every transform in `msg`, in order. The strings borrow from `msg`.
///
/// # Safety
///
/// `msg` must be a message of the type `layout` was resolved from, and
/// outlive the result.
pub unsafe fn samples<'a>(layout: &TfLayout, msg: *const c_void) -> Vec<TfSample<'a>> {
    let mut out = Vec::new();
    let seq = unsafe { (msg as *const u8).add(layout.transforms_offset) } as *const c_void;
    let n = unsafe { (layout.size_fn)(seq) }.min(1024);
    for i in 0..n {
        let t = unsafe { (layout.get_fn)(seq, i) } as *const u8;
        if t.is_null() {
            continue;
        }
        let values = unsafe { std::slice::from_raw_parts(t.add(layout.transform_offset), 7 * 8) };
        out.push(TfSample {
            parent: unsafe { read_str(t.add(layout.frame_id_offset)) },
            child: unsafe { read_str(t.add(layout.child_offset)) },
            sec: unsafe { *(t.add(layout.stamp_offset) as *const i32) },
            nanosec: unsafe { *(t.add(layout.stamp_offset + 4) as *const u32) },
            value_hash: fnv(values),
        });
    }
    out
}

static ANY: AtomicBool = AtomicBool::new(false);
static PUBLISHERS: RwLock<Option<HashMap<usize, (TfLayout, bool)>>> = RwLock::new(None);

/// Remember a TF publisher; `is_static` when its topic is a `tf_static`.
pub fn register(publisher: usize, layout: TfLayout, is_static: bool) {
    PUBLISHERS
        .write()
        .get_or_insert_with(HashMap::new)
        .insert(publisher, (layout, is_static));
    ANY.store(true, Ordering::Release);
}

/// The layout of a TF publisher. One relaxed load for every other
/// publisher in a process that has none.
pub fn lookup(publisher: usize) -> Option<(TfLayout, bool)> {
    if !ANY.load(Ordering::Acquire) {
        return None;
    }
    PUBLISHERS.read().as_ref()?.get(&publisher).copied()
}
