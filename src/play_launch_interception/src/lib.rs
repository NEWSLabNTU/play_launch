//! LD_PRELOAD interceptor for ROS 2 `rcl_publish` / `rcl_take`.
//!
//! When loaded via `LD_PRELOAD`, this library hooks four rcl functions to
//! extract `header.stamp` from published/received messages and report
//! per-topic timestamp frontiers to play_launch over a Unix datagram socket.
//!
//! # Inert mode
//!
//! The library is inert (all hooks pass through with zero extra work) when:
//! - `PLAY_LAUNCH_INTERCEPTION_SOCKET` is not set, OR
//! - the process doesn't link rcl (dlsym can't find the originals)

mod channel;
mod frontier;
mod introspection;
mod registry;

use std::ffi::{CStr, c_char, c_void};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;

use rcl_interception_sys::*;

use channel::{EVENT_PUBLISH, EVENT_TAKE, FrontierEvent};

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

/// When true, all hooks are pure pass-through (no socket, no registry).
static INERT: AtomicBool = AtomicBool::new(true);

struct Originals {
    publisher_init: FnRclPublisherInit,
    publish: FnRclPublish,
    subscription_init: FnRclSubscriptionInit,
    take: FnRclTake,
}

static ORIGINALS: OnceLock<Originals> = OnceLock::new();

/// Try to resolve original function pointers via `dlsym(RTLD_NEXT, ...)`.
///
/// Returns `None` if any symbol is missing (the process doesn't link rcl).
fn try_resolve_originals() -> Option<Originals> {
    unsafe {
        let publisher_init = libc::dlsym(libc::RTLD_NEXT, c"rcl_publisher_init".as_ptr());
        let publish = libc::dlsym(libc::RTLD_NEXT, c"rcl_publish".as_ptr());
        let subscription_init =
            libc::dlsym(libc::RTLD_NEXT, c"rcl_subscription_init".as_ptr());
        let take = libc::dlsym(libc::RTLD_NEXT, c"rcl_take".as_ptr());

        if publisher_init.is_null()
            || publish.is_null()
            || subscription_init.is_null()
            || take.is_null()
        {
            return None;
        }

        Some(Originals {
            publisher_init: std::mem::transmute::<*mut c_void, FnRclPublisherInit>(publisher_init),
            publish: std::mem::transmute::<*mut c_void, FnRclPublish>(publish),
            subscription_init: std::mem::transmute::<*mut c_void, FnRclSubscriptionInit>(
                subscription_init,
            ),
            take: std::mem::transmute::<*mut c_void, FnRclTake>(take),
        })
    }
}

/// Get resolved originals. Returns `None` if rcl was not found at init.
#[inline(always)]
fn originals() -> Option<&'static Originals> {
    ORIGINALS.get()
}

// ---------------------------------------------------------------------------
// Constructor — runs when the .so is loaded
// ---------------------------------------------------------------------------

#[ctor::ctor]
fn init() {
    // 1. Always try to resolve rcl originals — our exported symbols override
    //    the real ones regardless of mode, so hooks must be able to call through.
    if let Some(orig) = try_resolve_originals() {
        let _ = ORIGINALS.set(orig);
    }
    // If dlsym failed, ORIGINALS stays empty. Hooks return error (1), which
    // is fine — this only happens in non-ROS processes that don't call rcl.

    // 2. Activate frontier tracking only if the socket channel is ready AND
    //    we successfully resolved the originals.
    if channel::init() && ORIGINALS.get().is_some() {
        INERT.store(false, Ordering::Release);
        eprintln!("[play_launch_interception] active — socket configured");
    }
}

// ---------------------------------------------------------------------------
// Hooks
// ---------------------------------------------------------------------------

/// # Safety
///
/// Called by the dynamic linker as an override for `rcl_publisher_init`.
/// All pointer arguments must satisfy the same preconditions as the real
/// `rcl_publisher_init`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_publisher_init(
    publisher: *mut rcl_publisher_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const c_char,
    options: *const rcl_publisher_options_t,
) -> rcl_ret_t {
    let Some(orig) = originals() else {
        return 1;
    };

    let ret =
        unsafe { (orig.publisher_init)(publisher, node, type_support, topic_name, options) };

    if ret == 0 && !INERT.load(Ordering::Acquire) {
        let topic = unsafe { CStr::from_ptr(topic_name) }
            .to_string_lossy()
            .into_owned();
        let stamp_offset = unsafe { introspection::find_stamp_offset(type_support) };
        registry::register_publisher(publisher as usize, &topic, stamp_offset);
    }

    ret
}

/// # Safety
///
/// Called by the dynamic linker as an override for `rcl_publish`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_publish(
    publisher: *const rcl_publisher_t,
    ros_message: *const c_void,
    allocation: *mut rmw_publisher_allocation_t,
) -> rcl_ret_t {
    let Some(orig) = originals() else {
        return 1;
    };

    if !INERT.load(Ordering::Acquire)
        && let Some(info) = registry::lookup_publisher(publisher as usize)
    {
        let stamp_ptr =
            unsafe { (ros_message as *const u8).add(info.stamp_offset) as *const BuiltinTime };
        let stamp = unsafe { &*stamp_ptr };

        if frontier::update(info.frontier, stamp.sec, stamp.nanosec) {
            channel::send(&FrontierEvent {
                topic_hash: info.topic_hash,
                stamp_sec: stamp.sec,
                stamp_nanosec: stamp.nanosec,
                event_type: EVENT_PUBLISH,
            });
        }
    }

    unsafe { (orig.publish)(publisher, ros_message, allocation) }
}

/// # Safety
///
/// Called by the dynamic linker as an override for `rcl_subscription_init`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_subscription_init(
    subscription: *mut rcl_subscription_t,
    node: *const rcl_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const c_char,
    options: *const rcl_subscription_options_t,
) -> rcl_ret_t {
    let Some(orig) = originals() else {
        return 1;
    };

    let ret = unsafe {
        (orig.subscription_init)(subscription, node, type_support, topic_name, options)
    };

    if ret == 0 && !INERT.load(Ordering::Acquire) {
        let topic = unsafe { CStr::from_ptr(topic_name) }
            .to_string_lossy()
            .into_owned();
        let stamp_offset = unsafe { introspection::find_stamp_offset(type_support) };
        registry::register_subscription(subscription as usize, &topic, stamp_offset);
    }

    ret
}

/// # Safety
///
/// Called by the dynamic linker as an override for `rcl_take`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_take(
    subscription: *const rcl_subscription_t,
    ros_message: *mut c_void,
    message_info: *mut rmw_message_info_t,
    allocation: *mut rmw_subscription_allocation_t,
) -> rcl_ret_t {
    let Some(orig) = originals() else {
        return 1;
    };

    let ret = unsafe { (orig.take)(subscription, ros_message, message_info, allocation) };

    if ret == 0
        && !INERT.load(Ordering::Acquire)
        && let Some(info) = registry::lookup_subscription(subscription as usize)
    {
        let stamp_ptr = unsafe {
            (ros_message as *const u8).add(info.stamp_offset) as *const BuiltinTime
        };
        let stamp = unsafe { &*stamp_ptr };

        channel::send(&FrontierEvent {
            topic_hash: info.topic_hash,
            stamp_sec: stamp.sec,
            stamp_nanosec: stamp.nanosec,
            event_type: EVENT_TAKE,
        });
    }

    ret
}
