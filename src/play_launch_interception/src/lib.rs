//! LD_PRELOAD interceptor for ROS 2 `rcl_publish` / `rcl_take`.
//!
//! When loaded via `LD_PRELOAD`, this library hooks four rcl functions to
//! extract `header.stamp` from published/received messages and dispatch events
//! to compiled-in plugins.
//!
//! # Compiled-in plugins
//!
//! All plugins are compiled into the interception `.so`:
//! - **FrontierPlugin** — per-topic timestamp frontier tracking
//! - **StatsPlugin** — message counts, rates, latency (every publish/take)
//!
//! Both plugins share a single `spsc_shm` ring buffer (activated when
//! `PLAY_LAUNCH_INTERCEPTION_SHM_FD` is set). FrontierPlugin also supports
//! a standalone socket fallback via `PLAY_LAUNCH_INTERCEPTION_SOCKET`.
//!
//! # Inert mode
//!
//! The library is inert (all hooks pass through with zero extra work) when:
//! - the process doesn't link rcl (dlsym can't find the originals), OR
//! - originals found but no plugins activated (no relevant env vars set)
//!
//! # Python / rclpy support
//!
//! Python nodes load `librcl.so` lazily via `_rclpy_pybind11.so` with
//! `RTLD_LOCAL`, so `dlsym(RTLD_NEXT)` can't find it at ctor time. We use
//! lazy resolution: on first hook invocation, try `RTLD_NEXT` (C++ nodes),
//! then fall back to `dlopen("librcl.so", RTLD_NOLOAD)` + `dlsym(handle)`
//! to find the already-loaded library in its local scope.

mod event;
mod introspection;
mod plugin;
mod plugin_dispatch;
mod plugins;
mod registry;

use std::ffi::{CStr, c_char, c_void};
use std::os::fd::RawFd;
use std::sync::{Arc, OnceLock};

use parking_lot::Mutex;
use plugin::{InterceptionPlugin, Stamp};
use rcl_interception_sys::*;

use event::InterceptionEvent;
use spsc_shm::Producer;

// ---------------------------------------------------------------------------
// Runtime — replaces the old global state
// ---------------------------------------------------------------------------

struct Originals {
    publisher_init: FnRclPublisherInit,
    publish: FnRclPublish,
    subscription_init: FnRclSubscriptionInit,
    take: FnRclTake,
}

struct Runtime {
    originals: Originals,
    /// Compiled-in plugins. Empty if no plugins activated (inert mode).
    plugins: Vec<Box<dyn InterceptionPlugin>>,
}

/// Lazily initialized on first hook invocation.
static RUNTIME: OnceLock<Option<Runtime>> = OnceLock::new();

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
    let handle =
        unsafe { libc::dlopen(c"librcl.so".as_ptr(), libc::RTLD_NOW | libc::RTLD_NOLOAD) };
    if handle.is_null() {
        return None;
    }
    // Don't dlclose — we need the symbols to stay valid for the process lifetime.
    unsafe { resolve_from(handle) }
}

/// Try to open the shared memory ring buffer from `PLAY_LAUNCH_INTERCEPTION_SHM_FD`.
fn try_open_producer() -> Option<Arc<Mutex<Producer<InterceptionEvent>>>> {
    let fd_str = std::env::var("PLAY_LAUNCH_INTERCEPTION_SHM_FD").ok()?;
    let fd: RawFd = fd_str.parse().ok()?;
    let producer = unsafe { Producer::<InterceptionEvent>::from_raw_fd(fd) }.ok()?;
    Some(Arc::new(Mutex::new(producer)))
}

/// Build the list of compiled-in plugins, activating each based on env vars.
///
/// If `PLAY_LAUNCH_INTERCEPTION_SHM_FD` is set, both FrontierPlugin and
/// StatsPlugin are activated sharing a single ring buffer. FrontierPlugin
/// also supports standalone socket fallback via `PLAY_LAUNCH_INTERCEPTION_SOCKET`.
fn build_plugins() -> Vec<Box<dyn InterceptionPlugin>> {
    let mut plugins: Vec<Box<dyn InterceptionPlugin>> = Vec::new();

    let producer = try_open_producer();

    // FrontierPlugin: prefers shared memory, falls back to socket
    if let Some(frontier) = plugins::frontier::FrontierPlugin::new(producer.clone()) {
        plugins.push(Box::new(frontier));
    }

    // StatsPlugin: requires shared memory
    if let Some(ref prod) = producer {
        plugins.push(Box::new(plugins::stats::StatsPlugin::new(prod.clone())));
    }

    plugins
}

/// Get the runtime, initializing lazily on first call.
///
/// Returns `None` if `librcl.so` was not found via either strategy.
#[inline(always)]
fn runtime() -> Option<&'static Runtime> {
    RUNTIME
        .get_or_init(|| {
            let originals = try_resolve_originals()?;
            let plugins = build_plugins();

            if !plugins.is_empty() {
                eprintln!(
                    "[play_launch_interception] active — {} plugin(s)",
                    plugins.len()
                );
            }

            Some(Runtime {
                originals,
                plugins,
            })
        })
        .as_ref()
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
    let Some(rt) = runtime() else {
        return 1;
    };

    let ret =
        unsafe { (rt.originals.publisher_init)(publisher, node, type_support, topic_name, options) };

    if ret == 0 && !rt.plugins.is_empty() {
        let topic = unsafe { CStr::from_ptr(topic_name) }
            .to_string_lossy()
            .into_owned();
        let stamp_offset = unsafe { introspection::find_stamp_offset(type_support) };

        // Register in the shared registry (used for hot-path lookups).
        registry::register_publisher(publisher as usize, &topic, stamp_offset);

        let topic_hash = registry::fnv1a(topic.as_bytes());
        plugin_dispatch::dispatch_publisher_init(
            &rt.plugins,
            publisher as usize,
            &topic,
            topic_hash,
            stamp_offset,
        );
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
    let Some(rt) = runtime() else {
        return 1;
    };

    if !rt.plugins.is_empty() {
        if let Some(info) = registry::lookup_publisher(publisher as usize) {
            let stamp_ptr = unsafe {
                (ros_message as *const u8).add(info.stamp_offset) as *const BuiltinTime
            };
            let stamp = unsafe { &*stamp_ptr };
            plugin_dispatch::dispatch_publish(
                &rt.plugins,
                publisher as usize,
                info.topic_hash,
                Some(Stamp {
                    sec: stamp.sec,
                    nanosec: stamp.nanosec,
                }),
            );
        }
    }

    unsafe { (rt.originals.publish)(publisher, ros_message, allocation) }
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
    let Some(rt) = runtime() else {
        return 1;
    };

    let ret = unsafe {
        (rt.originals.subscription_init)(subscription, node, type_support, topic_name, options)
    };

    if ret == 0 && !rt.plugins.is_empty() {
        let topic = unsafe { CStr::from_ptr(topic_name) }
            .to_string_lossy()
            .into_owned();
        let stamp_offset = unsafe { introspection::find_stamp_offset(type_support) };

        // Register in the shared registry.
        registry::register_subscription(subscription as usize, &topic, stamp_offset);

        let topic_hash = registry::fnv1a(topic.as_bytes());
        plugin_dispatch::dispatch_subscription_init(
            &rt.plugins,
            subscription as usize,
            &topic,
            topic_hash,
            stamp_offset,
        );
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
    let Some(rt) = runtime() else {
        return 1;
    };

    let ret = unsafe { (rt.originals.take)(subscription, ros_message, message_info, allocation) };

    if ret == 0 && !rt.plugins.is_empty() {
        if let Some(info) = registry::lookup_subscription(subscription as usize) {
            let stamp_ptr = unsafe {
                (ros_message as *const u8).add(info.stamp_offset) as *const BuiltinTime
            };
            let stamp = unsafe { &*stamp_ptr };
            plugin_dispatch::dispatch_take(
                &rt.plugins,
                subscription as usize,
                info.topic_hash,
                Some(Stamp {
                    sec: stamp.sec,
                    nanosec: stamp.nanosec,
                }),
            );
        }
    }

    ret
}
