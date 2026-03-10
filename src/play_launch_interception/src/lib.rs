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
//!
//! # Python / rclpy support
//!
//! Python nodes load `librcl.so` lazily via `_rclpy_pybind11.so` with
//! `RTLD_LOCAL`, so `dlsym(RTLD_NEXT)` can't find it at ctor time. We use
//! lazy resolution: on first hook invocation, try `RTLD_NEXT` (C++ nodes),
//! then fall back to `dlopen("librcl.so", RTLD_NOLOAD)` + `dlsym(handle)`
//! to find the already-loaded library in its local scope.

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

/// When true, frontier tracking is disabled (no socket, no registry).
/// Set to false in `#[ctor]` if `PLAY_LAUNCH_INTERCEPTION_SOCKET` is set.
/// The hooks still call through to originals regardless — INERT only
/// controls whether we inspect messages and send events.
static CHANNEL_READY: AtomicBool = AtomicBool::new(false);

struct Originals {
    publisher_init: FnRclPublisherInit,
    publish: FnRclPublish,
    subscription_init: FnRclSubscriptionInit,
    take: FnRclTake,
}

/// Lazily resolved original function pointers. Initialized on first hook
/// invocation, not in `#[ctor]`, because Python nodes load `librcl.so`
/// after the constructor runs.
static ORIGINALS: OnceLock<Option<Originals>> = OnceLock::new();

/// Resolve all 4 rcl symbols from `source`, returning `None` if any are
/// missing.
///
/// # Safety
///
/// `source` must be a valid handle for `dlsym` (`RTLD_NEXT` or a dlopen'd
/// handle).
unsafe fn resolve_from(source: *mut c_void) -> Option<Originals> {
    let publisher_init = unsafe { libc::dlsym(source, c"rcl_publisher_init".as_ptr()) };
    let publish = unsafe { libc::dlsym(source, c"rcl_publish".as_ptr()) };
    let subscription_init =
        unsafe { libc::dlsym(source, c"rcl_subscription_init".as_ptr()) };
    let take = unsafe { libc::dlsym(source, c"rcl_take".as_ptr()) };

    if publisher_init.is_null()
        || publish.is_null()
        || subscription_init.is_null()
        || take.is_null()
    {
        return None;
    }

    Some(Originals {
        publisher_init: unsafe {
            std::mem::transmute::<*mut c_void, FnRclPublisherInit>(publisher_init)
        },
        publish: unsafe { std::mem::transmute::<*mut c_void, FnRclPublish>(publish) },
        subscription_init: unsafe {
            std::mem::transmute::<*mut c_void, FnRclSubscriptionInit>(subscription_init)
        },
        take: unsafe { std::mem::transmute::<*mut c_void, FnRclTake>(take) },
    })
}

/// Try to resolve original rcl function pointers.
///
/// 1. `RTLD_NEXT` — works for C++ nodes where `librcl.so` is in the global
///    symbol scope (loaded as a DT_NEEDED dependency of the main binary).
/// 2. `dlopen("librcl.so", RTLD_NOLOAD)` — works for Python nodes where
///    `librcl.so` was loaded with `RTLD_LOCAL` by `_rclpy_pybind11.so`.
///    `RTLD_NOLOAD` finds the already-loaded library without loading a new
///    copy, and handle-based `dlsym` searches its symbol table directly
///    (bypassing scope restrictions, no recursion risk).
fn try_resolve_originals() -> Option<Originals> {
    // Strategy 1: RTLD_NEXT (global scope — C++ nodes).
    if let Some(orig) = unsafe { resolve_from(libc::RTLD_NEXT) } {
        return Some(orig);
    }

    // Strategy 2: dlopen RTLD_NOLOAD (local scope — Python/rclpy nodes).
    let handle = unsafe { libc::dlopen(c"librcl.so".as_ptr(), libc::RTLD_NOW | libc::RTLD_NOLOAD) };
    if handle.is_null() {
        return None;
    }
    // Don't dlclose — we need the symbols to stay valid for the process lifetime.
    unsafe { resolve_from(handle) }
}

/// Get resolved originals, initializing lazily on first call.
///
/// Returns `None` if `librcl.so` was not found via either strategy.
#[inline(always)]
fn originals() -> Option<&'static Originals> {
    ORIGINALS
        .get_or_init(|| {
            let result = try_resolve_originals();
            if result.is_some() && CHANNEL_READY.load(Ordering::Acquire) {
                eprintln!("[play_launch_interception] active — socket configured");
            }
            result
        })
        .as_ref()
}

// ---------------------------------------------------------------------------
// Constructor — runs when the .so is loaded
// ---------------------------------------------------------------------------

#[ctor::ctor]
fn init() {
    // Initialize the socket channel early. This only checks the env var and
    // opens a socket — no dependency on librcl.so.
    //
    // Originals resolution is deferred to the first hook invocation because
    // Python nodes haven't loaded librcl.so yet at ctor time.
    if channel::init() {
        CHANNEL_READY.store(true, Ordering::Release);
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

    if ret == 0 && CHANNEL_READY.load(Ordering::Acquire) {
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

    if CHANNEL_READY.load(Ordering::Acquire)
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

    if ret == 0 && CHANNEL_READY.load(Ordering::Acquire) {
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
        && CHANNEL_READY.load(Ordering::Acquire)
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
