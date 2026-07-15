//! Linux scheduling syscall layer — the apply-layer for the shared
//! scheduling spec (`ros-launch-manifest-sched`).
//!
//! The scheduling spec has two stages:
//! - **validate-now** (`play_launch check --sched <file>`): parses and
//!   resolves the tier table for the `posix` platform, checking that the
//!   schema is internally consistent. No syscalls, no process interaction.
//! - **apply** (this module, phase 2): given an already-spawned PID and a
//!   pre-resolved [`AppliedTier`], actually sets the Linux scheduling policy,
//!   real-time priority, and CPU affinity via `sched_setscheduler(2)` /
//!   `sched_setaffinity(2)`.
//!
//! This module is intentionally self-contained: it does not know about
//! `ResolvedTierTable`, launch dumps, or the actor system. It is pure,
//! PID-in / result-out, and unit-testable in isolation. Higher-level phase-38
//! work (plan building, actor wiring) is layered on top in the `play_launch`
//! binary's `execution` module tree.
//!
//! This module lives in the **lib** (rather than the bin's `execution`
//! module tree) so that a future `src/bin/*` helper binary (phase 38.10, the
//! `CAP_SYS_NICE`-holding RT helper) can reuse the syscall core without
//! depending on the whole `play_launch` binary (ROS, clap, etc). See
//! `docs/superpowers/specs/2026-07-14-rt-helper-design.md` §3.3/§4.1.
//!
//! Linux scheduling attributes are per-THREAD, not per-process:
//! `sched_setscheduler(pid, ...)` only affects the thread whose TID equals
//! `pid` (conventionally the main thread) — sibling threads are untouched
//! unless they were created after the call (which inherit via glibc's
//! default `PTHREAD_INHERIT_SCHED`). A ROS node/composable can go from 1
//! thread to ~11 threads within half a second of exec (DDS spawns most of
//! them), so applying scheduling to only the "pid" misses every thread
//! spawned before the call. [`apply_tier`] therefore sweeps every TID under
//! `/proc/<pid>/task/` and applies to each individually.
//!
//! Public items here are not yet called outside this module (later phase-38
//! work — `sched_plan`, actor wiring — are the consumers), hence the
//! blanket `dead_code` allow.
#![allow(dead_code)]

use serde::{Deserialize, Serialize};

/// Linux scheduling policy derived from a tier's `sched_class`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SchedPolicy {
    Fifo,
    Rr,
    Other,
}

impl SchedPolicy {
    /// Map a tier `sched_class` string. Unknown / None → Other.
    pub fn from_sched_class(s: Option<&str>) -> Self {
        match s {
            Some("SCHED_FIFO") => SchedPolicy::Fifo,
            Some("SCHED_RR") => SchedPolicy::Rr,
            _ => SchedPolicy::Other,
        }
    }

    fn as_libc(self) -> libc::c_int {
        match self {
            SchedPolicy::Fifo => libc::SCHED_FIFO,
            SchedPolicy::Rr => libc::SCHED_RR,
            SchedPolicy::Other => libc::SCHED_OTHER,
        }
    }
}

/// Pre-resolved, platform-lowered knobs for one node.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AppliedTier {
    pub policy: SchedPolicy,
    /// RT priority for Fifo/Rr; ignored for Other.
    pub priority: i32,
    pub core: Option<u32>,
    /// Diagnostics only (tier name from the resolved spec).
    pub tier_name: String,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error, Serialize, Deserialize)]
pub enum SchedApplyError {
    #[error("permission denied setting scheduling for pid {pid} (need CAP_SYS_NICE or root)")]
    PermissionDenied { pid: u32 },
    #[error("invalid RT priority {priority} for pid {pid} (must be 1..=99)")]
    InvalidPriority { pid: u32, priority: i32 },
    #[error("{call} failed for pid {pid}: errno {errno}")]
    Syscall {
        pid: u32,
        // NOTE: `String`, not `&'static str` — a raw `&'static str` field
        // breaks serde's derived `Deserialize` when this type is nested
        // inside another derived container (e.g. `ipc::sched_protocol`'s
        // `SchedResponse::Applied(Result<(), SchedApplyError>)`): the
        // blanket `Deserialize` impl for `&'a str` requires `'de: 'a`, and
        // with `'a = 'static` that forces `'de: 'static` on the *caller's*
        // generic impl too, which serde_derive only threads through for
        // fields with an explicit lifetime parameter it can see — not an
        // opaque nested type. Owning the string sidesteps the whole issue.
        call: String,
        errno: i32,
    },
}

/// All thread IDs of a process, from `/proc/<pid>/task/`. Empty if the
/// process is gone (or `/proc` is unreadable). Sorted for deterministic
/// iteration order (not otherwise meaningful).
pub fn thread_ids(pid: u32) -> Vec<u32> {
    let mut tids = Vec::new();
    if let Ok(entries) = std::fs::read_dir(format!("/proc/{pid}/task")) {
        for e in entries.flatten() {
            if let Some(tid) = e.file_name().to_str().and_then(|s| s.parse::<u32>().ok()) {
                tids.push(tid);
            }
        }
    }
    tids.sort_unstable();
    tids
}

/// starttime (clock ticks since boot) of a PID, from `/proc/<pid>/stat` field
/// 22. `None` if the process is gone or the file is unparsable. Field 2
/// (`comm`) can contain spaces/parens — parse from after the LAST `')'`.
pub fn proc_start_time(pid: u32) -> Option<u64> {
    let content = std::fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
    let close_paren = content.rfind(')')?;
    // Fields after ')' start at field 3 (state). starttime is field 22
    // overall, i.e. the 20th field counting from field 3.
    let mut fields = content[close_paren + 1..].split_whitespace();
    // Skip fields 3..21 (indices 0..18, 19 fields) to land on field 22
    // (starttime, index 19).
    let starttime = fields.nth(19)?;
    starttime.parse::<u64>().ok()
}

/// Map the current `errno` (via `std::io::Error::last_os_error()`) to a
/// [`SchedApplyError`] for a given syscall name. `priority` is the value
/// that was attempted (used only to annotate `InvalidPriority` on `EINVAL`).
///
/// Only appropriate for the policy/priority syscall (`sched_setscheduler`):
/// an `EINVAL` there really does mean "priority rejected". Affinity errors
/// go through [`map_affinity_errno`] instead, since `EINVAL` there means
/// something else entirely (bad core id / cpu mask), not a bad priority.
fn map_errno(pid: u32, call: &'static str, priority: i32) -> SchedApplyError {
    let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
    match errno {
        libc::EPERM => SchedApplyError::PermissionDenied { pid },
        libc::EINVAL => SchedApplyError::InvalidPriority { pid, priority },
        _ => SchedApplyError::Syscall {
            pid,
            call: call.to_string(),
            errno,
        },
    }
}

/// Map the current `errno` for a failed `sched_setaffinity` call. Unlike
/// [`map_errno`], `EINVAL` here is NOT a priority problem (affinity has no
/// priority concept) — it means the given CPU mask was invalid (e.g. no bits
/// corresponding to an online CPU), so it falls through to `Syscall` like
/// any other unrecognized errno.
fn map_affinity_errno(pid: u32) -> SchedApplyError {
    let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
    match errno {
        libc::EPERM => SchedApplyError::PermissionDenied { pid },
        _ => SchedApplyError::Syscall {
            pid,
            call: "sched_setaffinity".to_string(),
            errno,
        },
    }
}

/// Apply policy + priority + CPU affinity to a single TID.
///
/// Order: RT policy/priority first (if `Fifo`/`Rr`), then affinity (applied
/// for ALL policies, including `Other`). A failure in either step returns
/// early with the mapped error.
///
/// `tid` is passed to `sched_setscheduler`/`sched_setaffinity` as the target
/// `pid` argument — per `sched_setscheduler(2)`, passing a TID (not just the
/// thread-group leader's PID) targets that specific thread.
fn apply_to_tid(tid: u32, tier: &AppliedTier) -> Result<(), SchedApplyError> {
    match tier.policy {
        SchedPolicy::Fifo | SchedPolicy::Rr => {
            let sched_param = libc::sched_param {
                sched_priority: tier.priority,
            };

            // SAFETY: sched_param is a plain-old-data struct with a single
            // `sched_priority` field we've fully initialized; tid is passed
            // by value.
            let ret = unsafe {
                libc::sched_setscheduler(tid as libc::pid_t, tier.policy.as_libc(), &sched_param)
            };

            if ret == -1 {
                return Err(map_errno(tid, "sched_setscheduler", tier.priority));
            }
        }
        SchedPolicy::Other => {
            if tier.priority != 0 {
                tracing::debug!(
                    tid,
                    priority = tier.priority,
                    "priority ignored for SCHED_OTHER"
                );
            }
        }
    }

    if let Some(cpu) = tier.core {
        // SAFETY: cpu_set_t is a plain fixed-size bitmask type; CPU_ZERO/
        // CPU_SET operate on a valid `&mut` reference to it.
        unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            libc::CPU_ZERO(&mut set);
            libc::CPU_SET(cpu as usize, &mut set);

            let ret = libc::sched_setaffinity(
                tid as libc::pid_t,
                std::mem::size_of::<libc::cpu_set_t>(),
                &set,
            );

            if ret == -1 {
                return Err(map_affinity_errno(tid));
            }
        }
    }

    Ok(())
}

/// Apply policy + priority + CPU affinity to every thread of an
/// already-spawned process (`pid` is the thread-group leader / main TID).
///
/// Linux scheduling attributes are per-thread, so this enumerates
/// `/proc/<pid>/task/` and applies [`apply_to_tid`] to each TID found there.
/// A thread that exits mid-sweep (`ESRCH`) is skipped rather than failing
/// the whole apply, since threads legitimately come and go while a process
/// is initializing. Any other error (e.g. `EPERM`) aborts the sweep and is
/// returned immediately — the caller (actor hook) decides whether to
/// warn-and-continue or abort based on `SchedApplyMode`.
///
/// Priority is validated before any syscall (including before the
/// `/proc` walk), so an invalid priority never touches the filesystem or
/// the kernel.
pub fn apply_tier(pid: u32, tier: &AppliedTier) -> Result<(), SchedApplyError> {
    if matches!(tier.policy, SchedPolicy::Fifo | SchedPolicy::Rr) && !(1..=99).contains(&tier.priority) {
        return Err(SchedApplyError::InvalidPriority {
            pid,
            priority: tier.priority,
        });
    }

    let tids = thread_ids(pid);
    if tids.is_empty() {
        return Err(SchedApplyError::Syscall {
            pid,
            call: "thread_ids".to_string(),
            errno: libc::ESRCH,
        });
    }

    for tid in tids {
        match apply_to_tid(tid, tier) {
            Ok(()) => {}
            Err(SchedApplyError::Syscall {
                errno: libc::ESRCH, ..
            }) => {
                // Thread exited between enumeration and apply — legitimate
                // race, not a failure of the overall sweep.
                tracing::debug!(pid, tid, "thread exited mid-sweep, skipping");
            }
            Err(e) => return Err(e),
        }
    }

    Ok(())
}

/// Preflight: can this process set RT scheduling at all?
///
/// `true` if running as root (`euid == 0`), or if `CAP_SYS_NICE` is present
/// in the effective capability set (`/proc/self/status` `CapEff:` line, bit
/// 23). Any read/parse failure is treated as "no privilege".
pub fn has_sched_privilege() -> bool {
    // SAFETY: geteuid() takes no arguments and cannot fail.
    if unsafe { libc::geteuid() } == 0 {
        return true;
    }

    const CAP_SYS_NICE: u64 = 23;

    let Ok(status) = std::fs::read_to_string("/proc/self/status") else {
        return false;
    };

    for line in status.lines() {
        if let Some(hex) = line.strip_prefix("CapEff:") {
            let hex = hex.trim();
            if let Ok(mask) = u64::from_str_radix(hex, 16) {
                return (mask & (1 << CAP_SYS_NICE)) != 0;
            }
            return false;
        }
    }

    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::process::Command;

    fn other_tier(core: Option<u32>) -> AppliedTier {
        AppliedTier {
            policy: SchedPolicy::Other,
            priority: 0,
            core,
            tier_name: "test".to_string(),
        }
    }

    fn fifo_tier(priority: i32) -> AppliedTier {
        AppliedTier {
            policy: SchedPolicy::Fifo,
            priority,
            core: None,
            tier_name: "test".to_string(),
        }
    }

    /// Spawn a throwaway `sleep` child, run `f` with its PID, then kill+wait
    /// it so no orphans leak regardless of test outcome.
    fn with_sleep_child<F: FnOnce(u32)>(f: F) {
        let mut child = Command::new("sleep")
            .arg("30")
            .spawn()
            .expect("failed to spawn sleep child");
        let pid = child.id();

        // Give the kernel a moment to finish exec() before we poke at it.
        std::thread::sleep(std::time::Duration::from_millis(50));

        f(pid);

        let _ = child.kill();
        let _ = child.wait();
    }

    #[test]
    fn priority_out_of_range_rejected_without_syscall() {
        // Bogus pid: if the syscall were reached it would fail with ESRCH
        // (Syscall variant), not InvalidPriority. Asserting InvalidPriority
        // proves validation happened before any syscall (including before
        // the /proc/<pid>/task walk).
        let bogus_pid = u32::MAX;

        let err = apply_tier(bogus_pid, &fifo_tier(0)).unwrap_err();
        assert_eq!(
            err,
            SchedApplyError::InvalidPriority {
                pid: bogus_pid,
                priority: 0,
            }
        );

        let err = apply_tier(bogus_pid, &fifo_tier(100)).unwrap_err();
        assert_eq!(
            err,
            SchedApplyError::InvalidPriority {
                pid: bogus_pid,
                priority: 100,
            }
        );
    }

    #[test]
    fn sched_class_string_maps_to_policy() {
        assert_eq!(
            SchedPolicy::from_sched_class(Some("SCHED_FIFO")),
            SchedPolicy::Fifo
        );
        assert_eq!(
            SchedPolicy::from_sched_class(Some("SCHED_RR")),
            SchedPolicy::Rr
        );
        assert_eq!(
            SchedPolicy::from_sched_class(Some("SCHED_OTHER")),
            SchedPolicy::Other
        );
        assert_eq!(SchedPolicy::from_sched_class(None), SchedPolicy::Other);
        assert_eq!(
            SchedPolicy::from_sched_class(Some("bogus")),
            SchedPolicy::Other
        );
    }

    #[test]
    fn privileged_self_apply_roundtrip() {
        with_sleep_child(|pid| {
            let tier = fifo_tier(10);
            let result = apply_tier(pid, &tier);

            if has_sched_privilege() {
                result.expect("apply_tier should succeed when privileged");

                // SAFETY: pid is a valid child pid we just spawned.
                let policy = unsafe { libc::sched_getscheduler(pid as libc::pid_t) };
                assert_eq!(policy, libc::SCHED_FIFO);

                let mut param = libc::sched_param { sched_priority: 0 };
                // SAFETY: param is a valid out-pointer for sched_getparam.
                let ret = unsafe { libc::sched_getparam(pid as libc::pid_t, &mut param) };
                assert_eq!(ret, 0);
                assert_eq!(param.sched_priority, 10);
            } else {
                assert_eq!(result, Err(SchedApplyError::PermissionDenied { pid }));
            }
        });
    }

    #[test]
    fn other_policy_applies_affinity_only() {
        with_sleep_child(|pid| {
            let tier = other_tier(Some(0));
            let result = apply_tier(pid, &tier);
            assert_eq!(result, Ok(()));
        });
    }

    #[test]
    fn affinity_failure_never_reports_invalid_priority() {
        // `AppliedTier` is constructed directly (not via `SchedPlan::build`)
        // because the FIX-1 build-time validation added in `sched_plan.rs`
        // would reject an out-of-range core before this could reach
        // `apply_tier` at all. This test targets `apply_tier` in isolation.
        //
        // `core = CPU_SETSIZE - 1` (1023) is within the fixed-size
        // `cpu_set_t` bitmask (so `CPU_SET` itself can't panic on an
        // out-of-bounds array index) but is not a CPU that exists on any
        // real machine, so the kernel rejects the resulting mask (no bits
        // corresponding to an online CPU) with `EINVAL`.
        with_sleep_child(|pid| {
            let tier = AppliedTier {
                policy: SchedPolicy::Other,
                priority: 0,
                core: Some(1023),
                tier_name: "test".to_string(),
            };

            let err = apply_tier(pid, &tier)
                .expect_err("setting affinity to a nonexistent CPU should fail");

            match err {
                SchedApplyError::Syscall { call, .. } => {
                    assert_eq!(call, "sched_setaffinity");
                }
                SchedApplyError::PermissionDenied { .. } => {
                    // Acceptable: an unprivileged environment could plausibly
                    // deny the affinity syscall outright before EINVAL comes
                    // into play. Either way, never `InvalidPriority`.
                }
                SchedApplyError::InvalidPriority { .. } => {
                    panic!(
                        "affinity failure must never be reported as InvalidPriority, got {err:?}"
                    );
                }
            }
        });
    }

    #[test]
    fn thread_ids_finds_multiple_threads() {
        let main_tid = std::process::id();

        // Spawn a thread and keep it alive with a barrier so the /proc
        // enumeration is guaranteed to see it.
        let barrier = std::sync::Arc::new(std::sync::Barrier::new(2));
        let b2 = barrier.clone();
        let handle = std::thread::spawn(move || {
            b2.wait();
        });

        // Give the new thread a moment to actually show up under
        // /proc/<pid>/task/ (creation and /proc visibility are not
        // perfectly synchronous).
        std::thread::sleep(std::time::Duration::from_millis(50));

        let tids = thread_ids(main_tid);
        assert!(
            tids.len() >= 2,
            "expected at least 2 TIDs (main + spawned), got {tids:?}"
        );
        assert!(
            tids.contains(&main_tid),
            "expected thread_ids to contain the main TID {main_tid}, got {tids:?}"
        );

        barrier.wait();
        handle.join().expect("spawned thread panicked");
    }

    #[test]
    fn per_tid_apply_reaches_every_thread_unprivileged() {
        if has_sched_privilege() {
            // This test asserts the unprivileged (PermissionDenied) path;
            // under privilege the syscall would actually succeed instead,
            // so there's nothing meaningful to assert here.
            eprintln!(
                "skipping per_tid_apply_reaches_every_thread_unprivileged: running with CAP_SYS_NICE/root"
            );
            return;
        }

        with_sleep_child(|pid| {
            // `sleep` is single-threaded, but the point of this test is that
            // apply_tier reaches the per-TID syscall path (not that it
            // no-ops for a bogus/gone target): a real, live TID must yield
            // PermissionDenied, proving the sweep actually dispatched a
            // syscall for that TID rather than silently skipping it.
            let tids = thread_ids(pid);
            assert!(
                !tids.is_empty(),
                "expected at least one TID for live child pid {pid}"
            );

            let result = apply_tier(pid, &fifo_tier(10));
            assert_eq!(result, Err(SchedApplyError::PermissionDenied { pid }));
        });
    }

    #[test]
    fn proc_start_time_own_pid_is_some_and_positive() {
        let pid = std::process::id();
        let start_time = proc_start_time(pid);
        assert!(
            matches!(start_time, Some(t) if t > 0),
            "expected Some(>0) starttime for own pid {pid}, got {start_time:?}"
        );
    }

    #[test]
    fn proc_start_time_bogus_pid_is_none() {
        // PID 0 is never a real process (reserved), and /proc/0/stat doesn't
        // exist, so this reliably exercises the "gone/unreadable" path.
        assert_eq!(proc_start_time(0), None);
    }
}
