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

mod allowlist;
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
    /// Node accessors for topic-name expansion (Phase 36 polish).
    /// Optional because resolution can fail in non-ROS processes.
    node_get_name: Option<FnRclNodeGetName>,
    node_get_namespace: Option<FnRclNodeGetNamespace>,
    /// RMW-layer originals (Phase 36.1). Optional — resolved separately
    /// from rcl symbols because the rmw symbols live in
    /// `librmw_implementation.so`, which may not be loaded under
    /// `RTLD_NEXT` scope (e.g. in Python processes that lazy-load the
    /// rmw layer). When any rmw symbol is missing, all rmw hooks pass
    /// through with no dispatch.
    rmw: Option<RmwOriginals>,
}

struct RmwOriginals {
    create_publisher: FnRmwCreatePublisher,
    create_subscription: FnRmwCreateSubscription,
    publish: FnRmwPublish,
    take_with_info: FnRmwTakeWithInfo,
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

    // Optional node accessors — may be missing in non-ROS processes
    // (in that case topic-name expansion silently skips).
    let node_get_name =
        unsafe { libc::dlsym(source, c"rcl_node_get_name".as_ptr()) };
    let node_get_namespace =
        unsafe { libc::dlsym(source, c"rcl_node_get_namespace".as_ptr()) };
    let node_get_name = if node_get_name.is_null() {
        None
    } else {
        Some(unsafe { std::mem::transmute::<*mut c_void, FnRclNodeGetName>(node_get_name) })
    };
    let node_get_namespace = if node_get_namespace.is_null() {
        None
    } else {
        Some(unsafe {
            std::mem::transmute::<*mut c_void, FnRclNodeGetNamespace>(node_get_namespace)
        })
    };

    Some(Originals {
        publisher_init: unsafe {
            std::mem::transmute::<*mut c_void, FnRclPublisherInit>(publisher_init)
        },
        publish: unsafe { std::mem::transmute::<*mut c_void, FnRclPublish>(publish) },
        subscription_init: unsafe {
            std::mem::transmute::<*mut c_void, FnRclSubscriptionInit>(subscription_init)
        },
        take: unsafe { std::mem::transmute::<*mut c_void, FnRclTake>(take) },
        node_get_name,
        node_get_namespace,
        rmw: None, // resolved separately in try_resolve_originals
    })
}

/// Expand a topic name to its fully-qualified form.
///
/// Mirrors the ROS 2 topic-name resolution rules (simplified — we
/// don't support advanced `{var}` substitutions, which are rare and
/// would require parsing the full rcl substitution map):
///
/// - Absolute (`/...`) — return unchanged.
/// - `~` prefix — replace with `<node_ns>/<node_name>`.
/// - Relative — prepend `<node_ns>/`.
///
/// Falls back to the raw `topic` string if the node accessors are
/// missing or fail.
fn expand_topic_name(originals: &Originals, node: *const rcl_node_t, topic: &str) -> String {
    if topic.starts_with('/') {
        return topic.to_string();
    }
    let (Some(get_name), Some(get_ns)) =
        (originals.node_get_name, originals.node_get_namespace)
    else {
        return topic.to_string();
    };
    if node.is_null() {
        return topic.to_string();
    }
    let name_ptr = unsafe { get_name(node) };
    let ns_ptr = unsafe { get_ns(node) };
    if name_ptr.is_null() || ns_ptr.is_null() {
        return topic.to_string();
    }
    let name = unsafe { CStr::from_ptr(name_ptr) }.to_string_lossy();
    let ns = unsafe { CStr::from_ptr(ns_ptr) }.to_string_lossy();

    // ~ -> <ns>/<name>/<rest>
    if let Some(rest) = topic.strip_prefix('~').and_then(|r| r.strip_prefix('/')) {
        let ns = ns.trim_end_matches('/');
        return if ns.is_empty() {
            format!("/{name}/{rest}")
        } else {
            format!("{ns}/{name}/{rest}")
        };
    }
    if topic == "~" {
        let ns = ns.trim_end_matches('/');
        return if ns.is_empty() {
            format!("/{name}")
        } else {
            format!("{ns}/{name}")
        };
    }

    // Relative — prepend node namespace.
    let ns = ns.trim_end_matches('/');
    if ns.is_empty() {
        format!("/{topic}")
    } else {
        format!("{ns}/{topic}")
    }
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
///
/// rmw symbols are resolved separately. They may be missing in
/// non-ROS processes or when LD_PRELOAD'd into Python rclpy before the
/// rmw layer loads — in those cases `Originals.rmw` is `None` and the
/// rmw hooks pass through with zero dispatch.
fn try_resolve_originals() -> Option<Originals> {
    // Strategy 1: RTLD_NEXT (global scope — C++ nodes).
    let mut originals = unsafe { resolve_from(libc::RTLD_NEXT) };

    // Strategy 2: dlopen RTLD_NOLOAD (local scope — Python/rclpy nodes).
    if originals.is_none() {
        let handle =
            unsafe { libc::dlopen(c"librcl.so".as_ptr(), libc::RTLD_NOW | libc::RTLD_NOLOAD) };
        if handle.is_null() {
            return None;
        }
        // Don't dlclose — we need the symbols to stay valid for the process lifetime.
        originals = unsafe { resolve_from(handle) };
    }

    let mut originals = originals?;
    originals.rmw = try_resolve_rmw_originals();
    Some(originals)
}

/// Try to resolve the rmw symbols. Returns `None` if any rmw symbol is
/// missing — the rmw hooks then degrade to pass-through.
///
/// `librmw_implementation.so` is a thin forwarder that the active RMW
/// (e.g. `rmw_fastrtps_cpp`) is loaded behind. The symbols
/// `rmw_create_publisher`, `rmw_publish`, etc. live in
/// `librmw_implementation.so` regardless of the active RMW backend, so
/// dlsym there gives the correct function pointers.
fn try_resolve_rmw_originals() -> Option<RmwOriginals> {
    // Strategy 1: RTLD_NEXT.
    if let Some(rmw) = unsafe { resolve_rmw_from(libc::RTLD_NEXT) } {
        return Some(rmw);
    }
    // Strategy 2: dlopen RTLD_NOLOAD.
    let handle = unsafe {
        libc::dlopen(
            c"librmw_implementation.so".as_ptr(),
            libc::RTLD_NOW | libc::RTLD_NOLOAD,
        )
    };
    if handle.is_null() {
        return None;
    }
    unsafe { resolve_rmw_from(handle) }
}

/// Resolve all rmw symbols from `source`.
///
/// # Safety
///
/// `source` must be a valid handle for `dlsym` (`RTLD_NEXT` or a
/// dlopen'd handle).
unsafe fn resolve_rmw_from(source: *mut c_void) -> Option<RmwOriginals> {
    let create_publisher = unsafe { libc::dlsym(source, c"rmw_create_publisher".as_ptr()) };
    let create_subscription =
        unsafe { libc::dlsym(source, c"rmw_create_subscription".as_ptr()) };
    let publish = unsafe { libc::dlsym(source, c"rmw_publish".as_ptr()) };
    let take_with_info = unsafe { libc::dlsym(source, c"rmw_take_with_info".as_ptr()) };

    if create_publisher.is_null()
        || create_subscription.is_null()
        || publish.is_null()
        || take_with_info.is_null()
    {
        return None;
    }

    Some(RmwOriginals {
        create_publisher: unsafe {
            std::mem::transmute::<*mut c_void, FnRmwCreatePublisher>(create_publisher)
        },
        create_subscription: unsafe {
            std::mem::transmute::<*mut c_void, FnRmwCreateSubscription>(create_subscription)
        },
        publish: unsafe { std::mem::transmute::<*mut c_void, FnRmwPublish>(publish) },
        take_with_info: unsafe {
            std::mem::transmute::<*mut c_void, FnRmwTakeWithInfo>(take_with_info)
        },
    })
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

    // QosNegotiationPlugin (Phase 36.2): requires shared memory.
    // Activated alongside stats — both rely on the same producer and
    // consumer pipeline.
    if let Some(ref prod) = producer {
        plugins.push(Box::new(
            plugins::qos_negotiation::QosNegotiationPlugin::new(prod.clone()),
        ));
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

    // Phase 36.7: blocking enforcement — when the allowlist is loaded
    // and the topic isn't on it, refuse to create the publisher.
    // Returns `RCL_RET_TOPIC_INVALID` (1004) so the caller sees a
    // recognizable error from `rcl_publisher_init`.
    if !topic_name.is_null() {
        let topic = unsafe { CStr::from_ptr(topic_name) }.to_string_lossy();
        let topic_hash = registry::fnv1a(topic.as_bytes());
        if !allowlist::is_allowed(topic_hash) {
            eprintln!(
                "[play_launch_interception] BLOCKED publisher init for '{}' (not in allowlist)",
                topic
            );
            return 1004;
        }
    }

    let ret =
        unsafe { (rt.originals.publisher_init)(publisher, node, type_support, topic_name, options) };

    if ret == 0 && !rt.plugins.is_empty() {
        let raw_topic = unsafe { CStr::from_ptr(topic_name) }
            .to_string_lossy()
            .into_owned();
        // Phase 36 polish: expand to FQN so the consumer-side
        // RuleEngine's topic_hash_to_fqn map matches manifest-declared
        // absolute names. Without this, bare names like "chatter" never
        // match `/<ns>/chatter` declarations.
        let topic = expand_topic_name(&rt.originals, node, &raw_topic);
        let stamp_offset = unsafe { introspection::find_stamp_offset(type_support) };
        let type_hash = unsafe { introspection::find_type_identity(type_support) }
            .map(|id| registry::fnv1a(id.as_bytes()));

        // Register in the shared registry (used for hot-path lookups).
        registry::register_publisher(publisher as usize, &topic, stamp_offset);

        let topic_hash = registry::fnv1a(topic.as_bytes());
        plugin_dispatch::dispatch_publisher_init(
            &rt.plugins,
            publisher as usize,
            &topic,
            topic_hash,
            stamp_offset,
            type_hash,
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
        // Try stamp-aware lookup first; fall back to full lookup for no-stamp messages.
        let dispatch_info = if let Some(info) = registry::lookup_publisher(publisher as usize) {
            let stamp_ptr = unsafe {
                (ros_message as *const u8).add(info.stamp_offset) as *const BuiltinTime
            };
            let stamp = unsafe { &*stamp_ptr };
            Some((
                info.topic_hash,
                Some(Stamp {
                    sec: stamp.sec,
                    nanosec: stamp.nanosec,
                }),
            ))
        } else {
            // No stamp_offset — still dispatch with stamp=None for StatsPlugin.
            registry::lookup_publisher_full(publisher as usize)
                .map(|(hash, _)| (hash, None))
        };

        if let Some((topic_hash, stamp)) = dispatch_info {
            plugin_dispatch::dispatch_publish(
                &rt.plugins,
                publisher as usize,
                topic_hash,
                stamp,
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

    // Phase 36.7: same blocking enforcement on the subscription side.
    if !topic_name.is_null() {
        let topic = unsafe { CStr::from_ptr(topic_name) }.to_string_lossy();
        let topic_hash = registry::fnv1a(topic.as_bytes());
        if !allowlist::is_allowed(topic_hash) {
            eprintln!(
                "[play_launch_interception] BLOCKED subscription init for '{}' (not in allowlist)",
                topic
            );
            return 1004;
        }
    }

    let ret = unsafe {
        (rt.originals.subscription_init)(subscription, node, type_support, topic_name, options)
    };

    if ret == 0 && !rt.plugins.is_empty() {
        let raw_topic = unsafe { CStr::from_ptr(topic_name) }
            .to_string_lossy()
            .into_owned();
        let topic = expand_topic_name(&rt.originals, node, &raw_topic);
        let stamp_offset = unsafe { introspection::find_stamp_offset(type_support) };
        let type_hash = unsafe { introspection::find_type_identity(type_support) }
            .map(|id| registry::fnv1a(id.as_bytes()));

        // Register in the shared registry.
        registry::register_subscription(subscription as usize, &topic, stamp_offset);

        let topic_hash = registry::fnv1a(topic.as_bytes());
        plugin_dispatch::dispatch_subscription_init(
            &rt.plugins,
            subscription as usize,
            &topic,
            topic_hash,
            stamp_offset,
            type_hash,
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
        let dispatch_info = if let Some(info) = registry::lookup_subscription(subscription as usize) {
            let stamp_ptr = unsafe {
                (ros_message as *const u8).add(info.stamp_offset) as *const BuiltinTime
            };
            let stamp = unsafe { &*stamp_ptr };
            Some((
                info.topic_hash,
                Some(Stamp {
                    sec: stamp.sec,
                    nanosec: stamp.nanosec,
                }),
            ))
        } else {
            registry::lookup_subscription_full(subscription as usize)
                .map(|(hash, _)| (hash, None))
        };

        if let Some((topic_hash, stamp)) = dispatch_info {
            plugin_dispatch::dispatch_take(
                &rt.plugins,
                subscription as usize,
                topic_hash,
                stamp,
            );
        }
    }

    ret
}

// ---------------------------------------------------------------------------
// RMW-layer hooks (Phase 36.1)
// ---------------------------------------------------------------------------
//
// These wrap rmw_create_publisher / rmw_create_subscription / rmw_publish /
// rmw_take_with_info. Like the rcl hooks they're inert when no plugins are
// active or when the rmw symbols couldn't be resolved (Originals.rmw is None).
//
// Hot path: pass-through to original + dispatch via plugin trait's rmw
// defaults (no-op for FrontierPlugin/StatsPlugin, real work for the upcoming
// QosNegotiation/DropMonitor plugins in 36.2).

/// Hash the raw QoS profile bytes so plugins can distinguish profiles
/// without exposing the distro-specific struct layout. The size is
/// stable enough across Humble / Jazzy that a fixed 64-byte read is
/// safe (the struct is always larger). Hash via fnv1a — same scheme
/// already used for topic hashes.
unsafe fn hash_qos_profile(qos: *const rmw_qos_profile_t) -> u64 {
    if qos.is_null() {
        return 0;
    }
    // 64 bytes covers reliability, durability, history, depth, deadline,
    // lifespan, and liveliness fields on every supported distro. Any
    // padding bytes hash deterministically because the struct is
    // memcpy-initialized by RMW.
    let bytes = unsafe { std::slice::from_raw_parts(qos as *const u8, 64) };
    registry::fnv1a(bytes)
}

#[inline]
fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: clock_gettime with a valid clock id and writable pointer.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (ts.tv_sec as u64) * 1_000_000_000 + ts.tv_nsec as u64
}

/// # Safety
///
/// Called by the dynamic linker as an override for `rmw_create_publisher`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmw_create_publisher(
    node: *const rmw_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const c_char,
    qos_profile: *const rmw_qos_profile_t,
    options: *const rmw_publisher_options_t,
) -> *mut rmw_publisher_t {
    let Some(rt) = runtime() else {
        return std::ptr::null_mut();
    };
    let Some(ref rmw) = rt.originals.rmw else {
        // rmw symbols unresolved — can't even forward. Returning null
        // would crash callers; safe fallback is to return null and let
        // them surface the error. In practice this branch is
        // unreachable when LD_PRELOAD'd into a real ROS node.
        return std::ptr::null_mut();
    };

    let ret = unsafe {
        (rmw.create_publisher)(node, type_support, topic_name, qos_profile, options)
    };

    if !ret.is_null() && !rt.plugins.is_empty() {
        let topic = unsafe { CStr::from_ptr(topic_name) }
            .to_string_lossy()
            .into_owned();
        let topic_hash = registry::fnv1a(topic.as_bytes());
        let qos_hash = unsafe { hash_qos_profile(qos_profile) };
        let parsed_qos = unsafe { parse_qos_profile(qos_profile) };
        for p in &rt.plugins {
            p.on_rmw_publisher_created(
                ret as usize,
                &topic,
                topic_hash,
                qos_hash,
                parsed_qos.as_ref(),
            );
        }
    }

    ret
}

/// # Safety
///
/// Called by the dynamic linker as an override for `rmw_create_subscription`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmw_create_subscription(
    node: *const rmw_node_t,
    type_support: *const rosidl_message_type_support_t,
    topic_name: *const c_char,
    qos_profile: *const rmw_qos_profile_t,
    options: *const rmw_subscription_options_t,
) -> *mut rmw_subscription_t {
    let Some(rt) = runtime() else {
        return std::ptr::null_mut();
    };
    let Some(ref rmw) = rt.originals.rmw else {
        return std::ptr::null_mut();
    };

    let ret = unsafe {
        (rmw.create_subscription)(node, type_support, topic_name, qos_profile, options)
    };

    if !ret.is_null() && !rt.plugins.is_empty() {
        let topic = unsafe { CStr::from_ptr(topic_name) }
            .to_string_lossy()
            .into_owned();
        let topic_hash = registry::fnv1a(topic.as_bytes());
        let qos_hash = unsafe { hash_qos_profile(qos_profile) };
        let parsed_qos = unsafe { parse_qos_profile(qos_profile) };
        for p in &rt.plugins {
            p.on_rmw_subscription_created(
                ret as usize,
                &topic,
                topic_hash,
                qos_hash,
                parsed_qos.as_ref(),
            );
        }
    }

    ret
}

/// Parse a `rmw_qos_profile_t` pointer into the layout-aware
/// `RmwQosProfile` view. Returns `None` if `qos` is null.
///
/// # Safety
///
/// `qos` must point to a valid `rmw_qos_profile_t` struct (matches
/// the C layout from `rmw/types.h`). The struct must remain valid for
/// the duration of the read. We copy the struct out by value rather
/// than borrow; the caller can drop the original safely after this
/// returns.
unsafe fn parse_qos_profile(
    qos: *const rmw_qos_profile_t,
) -> Option<rcl_interception_sys::qos::RmwQosProfile> {
    if qos.is_null() {
        return None;
    }
    // Cast to the layout-aware view and copy out by value.
    let p = qos as *const rcl_interception_sys::qos::RmwQosProfile;
    Some(unsafe { std::ptr::read(p) })
}

/// # Safety
///
/// Called by the dynamic linker as an override for `rmw_publish`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmw_publish(
    publisher: *const rmw_publisher_t,
    ros_message: *const c_void,
    allocation: *mut rmw_publisher_allocation_t,
) -> rcl_ret_t {
    let Some(rt) = runtime() else {
        return 1;
    };
    let Some(ref rmw) = rt.originals.rmw else {
        return 1;
    };

    if !rt.plugins.is_empty() {
        let ts = monotonic_ns();
        for p in &rt.plugins {
            p.on_rmw_publish(publisher as usize, ts);
        }
    }

    unsafe { (rmw.publish)(publisher, ros_message, allocation) }
}

/// # Safety
///
/// Called by the dynamic linker as an override for `rmw_take_with_info`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rmw_take_with_info(
    subscription: *const rmw_subscription_t,
    ros_message: *mut c_void,
    taken: *mut bool,
    message_info: *mut rmw_message_info_t,
    allocation: *mut rmw_subscription_allocation_t,
) -> rcl_ret_t {
    let Some(rt) = runtime() else {
        return 1;
    };
    let Some(ref rmw) = rt.originals.rmw else {
        return 1;
    };

    let ret = unsafe {
        (rmw.take_with_info)(subscription, ros_message, taken, message_info, allocation)
    };

    if ret == 0 && !rt.plugins.is_empty() {
        let was_taken = if !taken.is_null() {
            unsafe { *taken }
        } else {
            false
        };
        let ts = monotonic_ns();
        for p in &rt.plugins {
            p.on_rmw_take(subscription as usize, ts, was_taken);
        }
    }

    ret
}
