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
//! work (plan building, actor wiring) is layered on top in separate modules.
//!
//! Public items here are not yet called outside this module (later phase-38
//! tasks — `sched_plan`, actor wiring — are the consumers), hence the
//! blanket `dead_code` allow.
#![allow(dead_code)]

use clap::ValueEnum;

/// How aggressively to apply scheduling.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, ValueEnum)]
pub enum SchedApplyMode {
    /// Resolve + report only; no syscalls.
    Off,
    /// Apply per process; log a warning and continue on failure.
    #[default]
    Warn,
    /// Any capability/apply failure aborts the run.
    Strict,
}

/// Linux scheduling policy derived from a tier's `sched_class`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AppliedTier {
    pub policy: SchedPolicy,
    /// RT priority for Fifo/Rr; ignored for Other.
    pub priority: i32,
    pub core: Option<u32>,
    /// Diagnostics only (tier name from the resolved spec).
    pub tier_name: String,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SchedApplyError {
    #[error("permission denied setting scheduling for pid {pid} (need CAP_SYS_NICE or root)")]
    PermissionDenied { pid: u32 },
    #[error("invalid RT priority {priority} for pid {pid} (must be 1..=99)")]
    InvalidPriority { pid: u32, priority: i32 },
    #[error("{call} failed for pid {pid}: errno {errno}")]
    Syscall {
        pid: u32,
        call: &'static str,
        errno: i32,
    },
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
        _ => SchedApplyError::Syscall { pid, call, errno },
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
            call: "sched_setaffinity",
            errno,
        },
    }
}

/// Apply policy + priority + CPU affinity to an already-spawned PID.
///
/// Order: RT policy/priority first (if `Fifo`/`Rr`), then affinity (applied
/// for ALL policies, including `Other`). A failure in either step returns
/// early with the mapped error; the caller (actor hook) decides whether to
/// warn-and-continue or abort based on [`SchedApplyMode`].
pub fn apply_tier(pid: u32, tier: &AppliedTier) -> Result<(), SchedApplyError> {
    match tier.policy {
        SchedPolicy::Fifo | SchedPolicy::Rr => {
            if !(1..=99).contains(&tier.priority) {
                return Err(SchedApplyError::InvalidPriority {
                    pid,
                    priority: tier.priority,
                });
            }

            let sched_param = libc::sched_param {
                sched_priority: tier.priority,
            };

            // SAFETY: sched_param is a plain-old-data struct with a single
            // `sched_priority` field we've fully initialized; pid is passed
            // by value.
            let ret = unsafe {
                libc::sched_setscheduler(pid as libc::pid_t, tier.policy.as_libc(), &sched_param)
            };

            if ret == -1 {
                return Err(map_errno(pid, "sched_setscheduler", tier.priority));
            }
        }
        SchedPolicy::Other => {
            if tier.priority != 0 {
                tracing::debug!(
                    pid,
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
                pid as libc::pid_t,
                std::mem::size_of::<libc::cpu_set_t>(),
                &set,
            );

            if ret == -1 {
                return Err(map_affinity_errno(pid));
            }
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
        // proves validation happened before any syscall.
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
                let ret =
                    unsafe { libc::sched_getparam(pid as libc::pid_t, &mut param) };
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
}
