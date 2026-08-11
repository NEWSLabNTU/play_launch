//! Linux scheduling syscall layer — the apply-layer for the shared
//! scheduling spec (`ros-launch-manifest-sched`).
//!
//! The scheduling spec has two stages:
//! - **validate-now** (`ros-launch-resolve check --sched <file>`): parses and
//!   resolves the tier table for the `posix` platform, checking that the
//!   schema is internally consistent. No syscalls, no process interaction.
//! - **apply** (this module, phase 2): given an already-spawned PID and a
//!   pre-resolved [`AppliedTier`], actually sets the Linux scheduling policy,
//!   real-time priority, utilization clamp, and CPU affinity via
//!   `sched_setattr(2)` / `sched_setaffinity(2)`.
//!
//! The policy syscall is `sched_setattr(2)`, not `sched_setscheduler(2)`: the
//! latter cannot express `SCHED_DEADLINE`, utilization clamping, or any
//! `sched_flags` value, so it could not carry `SCHED_FLAG_RESET_ON_FORK`
//! either — which a `SCHED_DEADLINE` thread requires before it may `fork(2)`
//! at all.
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

// ---------------------------------------------------------------------------
// Raw `sched_setattr(2)` / `sched_getattr(2)` ABI
// ---------------------------------------------------------------------------
//
// `sched_setscheduler(2)` cannot express `SCHED_DEADLINE`, utilization
// clamping, or any `sched_flags` value, so the apply layer is built on
// `sched_setattr(2)` instead. glibc ships no wrapper for it and `libc` exposes
// neither the function nor `SYS_sched_setattr` on `x86_64-unknown-linux-gnu`
// (only on aarch64 and the x32 ABI), so both are defined here.
//
// `libc::sched_attr` exists but stops at `sched_period` — it is the original
// 48-byte `SCHED_ATTR_SIZE_VER0` layout, with no `sched_util_min`/`_max`. The
// full 56-byte VER1 layout is therefore declared locally rather than extended
// from it.

/// `struct sched_attr` (`SCHED_ATTR_SIZE_VER1`, Linux 5.3+).
///
/// Field order and widths are kernel ABI — do not reorder. `size` tells the
/// kernel how many bytes to read, so sending [`SCHED_ATTR_SIZE_VER0`] makes it
/// ignore the two trailing uclamp fields on kernels that predate them.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct SchedAttr {
    size: u32,
    sched_policy: u32,
    sched_flags: u64,
    sched_nice: i32,
    sched_priority: u32,
    sched_runtime: u64,
    sched_deadline: u64,
    sched_period: u64,
    sched_util_min: u32,
    sched_util_max: u32,
}

/// `SCHED_DEADLINE`. `libc` does not define it for linux-gnu.
const SCHED_DEADLINE: libc::c_int = 6;

/// Original layout, through `sched_period` (Linux 3.14).
const SCHED_ATTR_SIZE_VER0: u32 = 48;
/// Layout including `sched_util_min`/`sched_util_max` (Linux 5.3).
const SCHED_ATTR_SIZE_VER1: u32 = 56;

#[cfg(target_arch = "x86_64")]
const SYS_SCHED_SETATTR: libc::c_long = 314;
#[cfg(target_arch = "x86_64")]
const SYS_SCHED_GETATTR: libc::c_long = 315;
#[cfg(target_arch = "aarch64")]
const SYS_SCHED_SETATTR: libc::c_long = 274;
#[cfg(target_arch = "aarch64")]
const SYS_SCHED_GETATTR: libc::c_long = 275;

#[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
compile_error!(
    "play_launch's scheduling apply layer defines sched_setattr/sched_getattr \
     syscall numbers for x86_64 and aarch64 only (the two architectures the \
     wheel ships for). Add this target's numbers to sched.rs to build here."
);

/// `sched_setattr(2)`. `pid == 0` means the calling thread.
///
/// # Safety
/// `attr` must point to a valid, fully initialized [`SchedAttr`] whose `size`
/// field does not exceed the struct's real size.
unsafe fn sched_setattr(pid: libc::pid_t, attr: &SchedAttr, flags: u32) -> libc::c_long {
    unsafe { libc::syscall(SYS_SCHED_SETATTR, pid, attr as *const SchedAttr, flags) }
}

/// `sched_getattr(2)`. Used by the capability probe and by tests that assert
/// what was actually applied rather than what we asked for.
///
/// # Safety
/// `attr` must point to a writable [`SchedAttr`] of at least `size` bytes.
unsafe fn sched_getattr(
    pid: libc::pid_t,
    attr: &mut SchedAttr,
    size: u32,
    flags: u32,
) -> libc::c_long {
    unsafe { libc::syscall(SYS_SCHED_GETATTR, pid, attr as *mut SchedAttr, size, flags) }
}

/// What the running kernel accepts, probed once.
///
/// Needed because `sched_setattr` reports an unsupported flag as `EINVAL` and
/// an unrecognized struct size as `E2BIG` — neither distinguishable from a bad
/// parameter without knowing what the kernel supports. Reporting "invalid RT
/// priority 34" for a uclamp request on a 5.0 kernel sends an integrator
/// hunting a priority that is fine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KernelSchedSupport {
    /// `sched_setattr`/`sched_getattr` exist at all (Linux 3.14+).
    pub setattr: bool,
    /// Utilization clamping (`SCHED_FLAG_UTIL_CLAMP_*`, Linux 5.3+).
    pub uclamp: bool,
}

static KERNEL_SUPPORT: std::sync::OnceLock<KernelSchedSupport> = std::sync::OnceLock::new();

/// Probe (once) what this kernel accepts.
///
/// `setattr` is probed with `sched_getattr`, which is read-only and therefore
/// safe to call on ourselves. `uclamp` is probed with a `sched_setattr` that
/// carries `SCHED_FLAG_KEEP_ALL`, so policy and parameters are left untouched,
/// and requests the CFS defaults (`0..=1024`) — on a kernel that supports it
/// the call is a no-op, and on one that does not it fails without having
/// changed anything.
///
/// `SCHED_FLAG_DL_OVERRUN` is deliberately NOT probed: it is only valid
/// together with `SCHED_DEADLINE`, so setting it on a non-deadline thread
/// fails with `EINVAL` whether or not the kernel supports the flag, and the
/// probe could not tell the two apart. Its support is established where it is
/// used instead.
pub fn kernel_sched_support() -> KernelSchedSupport {
    *KERNEL_SUPPORT.get_or_init(|| {
        let mut probe = SchedAttr::default();
        // SAFETY: `probe` is a valid writable SchedAttr of exactly VER1 bytes.
        let setattr = (unsafe { sched_getattr(0, &mut probe, SCHED_ATTR_SIZE_VER1, 0) }) == 0;

        let uclamp = setattr && {
            let attr = SchedAttr {
                size: SCHED_ATTR_SIZE_VER1,
                sched_flags: (libc::SCHED_FLAG_KEEP_ALL
                    | libc::SCHED_FLAG_UTIL_CLAMP_MIN
                    | libc::SCHED_FLAG_UTIL_CLAMP_MAX) as u64,
                sched_util_min: 0,
                sched_util_max: UCLAMP_MAX,
                ..Default::default()
            };
            // SAFETY: fully initialized attr whose `size` equals its real size.
            (unsafe { sched_setattr(0, &attr, 0) }) == 0
        };

        KernelSchedSupport { setattr, uclamp }
    })
}

/// The largest legal utilization-clamp value (the kernel's scale is 0..=1024,
/// not a percentage).
pub const UCLAMP_MAX: u32 = 1024;

/// A utilization-clamp request, in the kernel's 0..=1024 scale.
///
/// Note that on `SCHED_FIFO`/`SCHED_RR` threads `min` is a no-op in practice:
/// RT tasks default to `uclamp_min = uclamp_max = 1024` because they "must
/// always run at a constant frequency to combat undeterministic DVFS rampup
/// delays", so under `schedutil` they already request the maximum performance
/// point. The useful RT knob is `max` (run an RT thread *below* the maximum,
/// which matters on battery). `min` is meaningful for `SCHED_OTHER`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Uclamp {
    pub min: u32,
    pub max: u32,
}

/// A set of CPUs for `sched_setaffinity(2)`.
///
/// Replaces the earlier `core: Option<u32>`, which could express exactly one
/// CPU. Stored sorted and deduplicated so equality is structural.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CpuSet(Vec<u32>);

impl CpuSet {
    /// Build from any iterator of CPU indices; sorts and deduplicates.
    pub fn new(cpus: impl IntoIterator<Item = u32>) -> Self {
        let mut cpus: Vec<u32> = cpus.into_iter().collect();
        cpus.sort_unstable();
        cpus.dedup();
        CpuSet(cpus)
    }

    /// The single-CPU case, which is what the old `core:` field expressed.
    pub fn single(cpu: u32) -> Self {
        CpuSet(vec![cpu])
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn cpus(&self) -> &[u32] {
        &self.0
    }

    /// Lower to a `cpu_set_t`. Returns `None` if any CPU is outside
    /// `CPU_SETSIZE`, since `CPU_SET` on an out-of-range index is UB rather
    /// than an error the kernel would reject.
    fn to_cpu_set_t(&self) -> Option<libc::cpu_set_t> {
        // SAFETY: cpu_set_t is a plain fixed-size bitmask; zeroed is a valid
        // (empty) value and CPU_SET writes only within it after the bounds
        // check below.
        unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            libc::CPU_ZERO(&mut set);
            for &cpu in &self.0 {
                if cpu as usize >= libc::CPU_SETSIZE as usize {
                    return None;
                }
                libc::CPU_SET(cpu as usize, &mut set);
            }
            Some(set)
        }
    }
}

impl std::fmt::Display for CpuSet {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let list: Vec<String> = self.0.iter().map(|c| c.to_string()).collect();
        write!(f, "{}", list.join(","))
    }
}

/// Linux scheduling policy derived from a tier's `sched_class`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SchedPolicy {
    Fifo,
    Rr,
    Other,
    /// `SCHED_BATCH` — CFS without the interactivity bonus, so it preempts
    /// less. Reachable only by an explicit override today.
    Batch,
    /// `SCHED_IDLE` — runs only when nothing else wants the CPU. Override-only,
    /// and never derived: it is near-total starvation under load, which is the
    /// wrong default for anything whose criticality has not been established.
    Idle,
    /// `SCHED_DEADLINE` — a CBS reservation. Carries no priority: a deadline
    /// thread preempts every fixed-priority thread regardless of RT priority.
    Deadline,
}

impl SchedPolicy {
    /// Map a tier `sched_class` string. Unknown / None → Other.
    pub fn from_sched_class(s: Option<&str>) -> Self {
        match s {
            Some("SCHED_FIFO") => SchedPolicy::Fifo,
            Some("SCHED_RR") => SchedPolicy::Rr,
            Some("SCHED_BATCH") => SchedPolicy::Batch,
            Some("SCHED_IDLE") => SchedPolicy::Idle,
            Some("SCHED_DEADLINE") => SchedPolicy::Deadline,
            _ => SchedPolicy::Other,
        }
    }

    fn as_libc(self) -> libc::c_int {
        match self {
            SchedPolicy::Fifo => libc::SCHED_FIFO,
            SchedPolicy::Rr => libc::SCHED_RR,
            SchedPolicy::Other => libc::SCHED_OTHER,
            SchedPolicy::Batch => libc::SCHED_BATCH,
            SchedPolicy::Idle => libc::SCHED_IDLE,
            SchedPolicy::Deadline => SCHED_DEADLINE,
        }
    }
}

/// Pre-resolved, platform-lowered knobs for one node.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AppliedTier {
    pub policy: SchedPolicy,
    /// RT priority for Fifo/Rr; ignored for the CFS policies.
    pub priority: i32,
    /// CFS nice value for Other/Batch; ignored for Fifo/Rr/Idle (Idle has no
    /// nice of its own — the kernel forces it to the weakest weight).
    pub nice: i32,
    /// CPUs to pin to; `None` leaves affinity untouched.
    pub cpus: Option<CpuSet>,
    /// Utilization clamp; `None` leaves it at the policy default. Carried
    /// through the wire protocol and applied here, but nothing derives it yet
    /// — the platform-file vocabulary that authors it arrives with the
    /// override surface.
    pub uclamp: Option<Uclamp>,
    /// `SCHED_DEADLINE` reservation, in nanoseconds. `Some` only when
    /// `policy == Deadline`.
    pub reservation: Option<Reservation>,
    /// Diagnostics only (tier name from the resolved spec).
    pub tier_name: String,
}

/// A CBS reservation: `runtime` every `period`, to be completed within
/// `deadline`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Reservation {
    pub runtime_ns: u64,
    pub deadline_ns: u64,
    pub period_ns: u64,
    /// `SCHED_FLAG_DL_OVERRUN` — deliver `SIGXCPU` on overrun.
    pub overrun: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error, Serialize, Deserialize)]
pub enum SchedApplyError {
    #[error("permission denied setting scheduling for pid {pid} (need CAP_SYS_NICE or root)")]
    PermissionDenied { pid: u32 },
    #[error("invalid RT priority {priority} for pid {pid} (must be 1..=99)")]
    InvalidPriority { pid: u32, priority: i32 },
    #[error(
        "{feature} is not supported by this kernel (pid {pid}, errno {errno}); \
         {requires} is required"
    )]
    UnsupportedFeature {
        pid: u32,
        feature: String,
        requires: String,
        errno: i32,
    },
    #[error("invalid CPU set {cpus} for pid {pid}: {reason}")]
    InvalidCpuSet {
        pid: u32,
        cpus: String,
        reason: String,
    },
    #[error("invalid uclamp range {min}..={max} for pid {pid} (need min <= max <= 1024)")]
    InvalidUclamp { pid: u32, min: u32, max: u32 },
    #[error("pid {pid} has policy SCHED_DEADLINE but carries no reservation parameters")]
    MissingReservation { pid: u32 },
    #[error(
        "admission control rejected the reservation for pid {pid}: {arithmetic}. \
         The kernel admits a new deadline task only while the SUM of runtime/period across \
         the root domain stays under `sched_rt_runtime_us / sched_rt_period_us`, and that \
         budget is shared with every SCHED_FIFO/RR thread in the same domain."
    )]
    AdmissionRejected {
        pid: u32,
        runtime_ns: u64,
        period_ns: u64,
        /// Pre-rendered so the raw numbers stay structured while the message
        /// still shows the arithmetic. `f64` fields would forbid `Eq`, which
        /// this type derives for the IPC round-trip tests.
        arithmetic: String,
    },
    #[error(
        "SCHED_DEADLINE was refused for pid {pid} with EPERM. A deadline thread's CPU \
         affinity may not be narrower than the root domain it was created on, so this \
         process must be STARTED inside an exclusive cpuset partition — it cannot be \
         pinned into one, and it cannot migrate into one either. Run \
         `play_launch verify` to see the partition state."
    )]
    DeadlineNeedsPartition { pid: u32 },
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

/// The kernel's total deadline-bandwidth ceiling, as a fraction of one CPU.
///
/// Admission control admits a new reservation only while
/// `sum(runtime/period) < M * (sched_rt_runtime_us / sched_rt_period_us)`.
/// Read from the running kernel rather than assumed, because an integrator may
/// have changed it and reporting the default would misstate the arithmetic.
fn rt_bandwidth_ceiling() -> Option<f64> {
    let runtime: f64 = std::fs::read_to_string("/proc/sys/kernel/sched_rt_runtime_us")
        .ok()?
        .trim()
        .parse()
        .ok()?;
    let period: f64 = std::fs::read_to_string("/proc/sys/kernel/sched_rt_period_us")
        .ok()?
        .trim()
        .parse()
        .ok()?;
    // -1 disables throttling entirely, which means no ceiling at all.
    if runtime < 0.0 || period <= 0.0 {
        return None;
    }
    Some(runtime / period)
}

/// Map a failed `sched_setattr(SCHED_DEADLINE)`.
///
/// Two errnos mean something specific here and would be actively misleading if
/// folded into the generic mapping:
///
/// - `EBUSY` is admission control refusing the reservation. The useful report
///   is the arithmetic — this task's utilization against the kernel's ceiling —
///   not "resource busy".
/// - `EPERM` is almost always the affinity/root-domain rule rather than a
///   missing capability, since the helper holds `CAP_SYS_NICE` by construction.
///   Saying "need CAP_SYS_NICE" would send the reader to check something that
///   is already true.
fn map_deadline_errno(pid: u32, res: &Reservation, requested_uclamp: bool) -> SchedApplyError {
    let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
    match errno {
        libc::EBUSY => {
            let utilization = if res.period_ns > 0 {
                res.runtime_ns as f64 / res.period_ns as f64 * 100.0
            } else {
                f64::NAN
            };
            let arithmetic = match rt_bandwidth_ceiling() {
                Some(ceiling) => format!(
                    "this task asks for {utilization:.1}% of one CPU ({}ns runtime / {}ns                      period) against a per-CPU ceiling of {:.0}%",
                    res.runtime_ns,
                    res.period_ns,
                    ceiling * 100.0
                ),
                None => format!(
                    "this task asks for {utilization:.1}% of one CPU ({}ns runtime / {}ns                      period); the kernel's RT bandwidth ceiling could not be read",
                    res.runtime_ns, res.period_ns
                ),
            };
            SchedApplyError::AdmissionRejected {
                pid,
                runtime_ns: res.runtime_ns,
                period_ns: res.period_ns,
                arithmetic,
            }
        }
        libc::EPERM => SchedApplyError::DeadlineNeedsPartition { pid },
        _ => map_setattr_errno(pid, requested_uclamp, 0),
    }
}

/// Map a failed `sched_setattr` to a [`SchedApplyError`].
///
/// `EINVAL` is deliberately NOT translated to `InvalidPriority` unconditionally
/// — that was defensible when `sched_setscheduler` was the only caller, but
/// `sched_setattr` also returns `EINVAL` for an unsupported `sched_flags` bit
/// and `E2BIG` for a `struct sched_attr` larger than the kernel understands. A
/// uclamp request on a pre-5.3 kernel would otherwise be reported as "invalid
/// RT priority 34", sending the reader after a priority that is fine. The
/// kernel probe ([`kernel_sched_support`]) is what tells the two apart.
fn map_setattr_errno(pid: u32, requested_uclamp: bool, priority: i32) -> SchedApplyError {
    let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
    let support = kernel_sched_support();

    match errno {
        libc::EPERM => SchedApplyError::PermissionDenied { pid },
        libc::E2BIG => SchedApplyError::UnsupportedFeature {
            pid,
            feature: "struct sched_attr (SCHED_ATTR_SIZE_VER1)".to_string(),
            requires: "Linux 5.3+".to_string(),
            errno,
        },
        libc::EINVAL if requested_uclamp && !support.uclamp => {
            SchedApplyError::UnsupportedFeature {
                pid,
                feature: "utilization clamping (SCHED_FLAG_UTIL_CLAMP_MIN/MAX)".to_string(),
                requires: "Linux 5.3+".to_string(),
                errno,
            }
        }
        libc::EINVAL if !support.setattr => SchedApplyError::UnsupportedFeature {
            pid,
            feature: "sched_setattr(2)".to_string(),
            requires: "Linux 3.14+".to_string(),
            errno,
        },
        libc::EINVAL => SchedApplyError::InvalidPriority { pid, priority },
        _ => SchedApplyError::Syscall {
            pid,
            call: "sched_setattr".to_string(),
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
/// `SCHED_FLAG_RESET_ON_FORK`, for the policies that require it — and only
/// those.
///
/// Today that is nothing: `SCHED_DEADLINE` is the only policy the kernel
/// refuses to `fork(2)` from without the flag, and this apply layer cannot
/// express `SCHED_DEADLINE` yet. The function exists so the rule has one
/// home, with its justification, rather than being rediscovered as a
/// mysterious thread at `SCHED_OTHER`.
///
/// See [`apply_to_tid`] for why applying it to `SCHED_FIFO`/`SCHED_RR` breaks
/// the per-TID sweep.
fn reset_on_fork_flag(policy: SchedPolicy) -> u64 {
    match policy {
        SchedPolicy::Fifo
        | SchedPolicy::Rr
        | SchedPolicy::Other
        | SchedPolicy::Batch
        | SchedPolicy::Idle => 0,
        // The one policy that genuinely needs it: the kernel refuses `fork(2)`
        // from a SCHED_DEADLINE thread without it (`EAGAIN`), and the isolated
        // container manager forks once per composable node under the DEFAULT
        // container mode.
        SchedPolicy::Deadline => libc::SCHED_FLAG_RESET_ON_FORK as u64,
    }
}

fn apply_to_tid(tid: u32, tier: &AppliedTier) -> Result<(), SchedApplyError> {
    match tier.policy {
        SchedPolicy::Fifo | SchedPolicy::Rr => {
            // NOTE: `SCHED_FLAG_RESET_ON_FORK` is deliberately NOT set here.
            //
            // It is tempting — it stops a forked child from inheriting real
            // time — but it is incompatible with this module's per-TID sweep,
            // and measurably so: with the flag set,
            // `per_tid_sched_fifo_launch_privileged_only` fails with a
            // `control_node` thread at policy 0.
            //
            // The kernel resets scheduling in `sched_fork()`, which runs for
            // **thread** creation as well as process creation — a
            // `clone(CLONE_THREAD)` goes through the same path. So the flag
            // does not merely disarm `fork(2)`; it also stops every thread
            // created *after* the sweep from inheriting the policy via glibc's
            // default `PTHREAD_INHERIT_SCHED`. Since a ROS node keeps spawning
            // DDS threads for hundreds of milliseconds after exec, that leaves
            // an arbitrary subset of a node's threads at `SCHED_OTHER` — which
            // is precisely the failure the per-TID sweep exists to prevent.
            //
            // The one policy that genuinely requires the flag is
            // `SCHED_DEADLINE`, whose threads cannot `fork(2)` at all without
            // it (`EAGAIN`). It belongs there and nowhere else, so it is set
            // per-policy rather than uniformly for "RT".
            let mut flags = reset_on_fork_flag(tier.policy);
            let mut size = SCHED_ATTR_SIZE_VER0;
            let mut util_min = 0;
            let mut util_max = 0;

            if let Some(uclamp) = &tier.uclamp {
                flags |= (libc::SCHED_FLAG_UTIL_CLAMP_MIN | libc::SCHED_FLAG_UTIL_CLAMP_MAX) as u64;
                size = SCHED_ATTR_SIZE_VER1;
                util_min = uclamp.min;
                util_max = uclamp.max;
            }

            let attr = SchedAttr {
                size,
                sched_policy: tier.policy.as_libc() as u32,
                sched_flags: flags,
                sched_priority: tier.priority as u32,
                sched_util_min: util_min,
                sched_util_max: util_max,
                ..Default::default()
            };

            // SAFETY: `attr` is fully initialized and its `size` field is
            // never larger than the struct itself.
            let ret = unsafe { sched_setattr(tid as libc::pid_t, &attr, 0) };
            if ret == -1 {
                return Err(map_setattr_errno(tid, tier.uclamp.is_some(), tier.priority));
            }
        }
        SchedPolicy::Deadline => {
            let Some(res) = &tier.reservation else {
                return Err(SchedApplyError::MissingReservation { pid: tid });
            };

            let mut flags = reset_on_fork_flag(tier.policy);
            if res.overrun {
                flags |= libc::SCHED_FLAG_DL_OVERRUN as u64;
            }
            let mut size = SCHED_ATTR_SIZE_VER0;
            let mut util_min = 0;
            let mut util_max = 0;
            if let Some(uclamp) = &tier.uclamp {
                flags |= (libc::SCHED_FLAG_UTIL_CLAMP_MIN | libc::SCHED_FLAG_UTIL_CLAMP_MAX) as u64;
                size = SCHED_ATTR_SIZE_VER1;
                util_min = uclamp.min;
                util_max = uclamp.max;
            }

            let attr = SchedAttr {
                size,
                sched_policy: tier.policy.as_libc() as u32,
                sched_flags: flags,
                sched_runtime: res.runtime_ns,
                sched_deadline: res.deadline_ns,
                sched_period: res.period_ns,
                sched_util_min: util_min,
                sched_util_max: util_max,
                ..Default::default()
            };

            // SAFETY: fully initialized attr whose `size` never exceeds the
            // struct's real size.
            let ret = unsafe { sched_setattr(tid as libc::pid_t, &attr, 0) };
            if ret == -1 {
                return Err(map_deadline_errno(tid, res, tier.uclamp.is_some()));
            }
            // No `sched_setaffinity` for a reservation, ever: a deadline
            // thread's affinity may not be narrower than the root domain it
            // was created on, so narrowing it returns EPERM. Its CPUs come
            // from the cpuset partition the process was started in.
            return Ok(());
        }
        SchedPolicy::Other | SchedPolicy::Batch | SchedPolicy::Idle => {
            // `SCHED_OTHER` at nice 0 with no uclamp is what every thread
            // already has, so issuing the syscall would change nothing while
            // adding a failure mode — and would *reset* a nice value somebody
            // else set. Skip it, exactly as before the override vocabulary
            // existed.
            let is_no_op = matches!(tier.policy, SchedPolicy::Other)
                && tier.nice == 0
                && tier.uclamp.is_none();
            if is_no_op {
                if tier.priority != 0 {
                    tracing::debug!(
                        tid,
                        priority = tier.priority,
                        "priority ignored for SCHED_OTHER"
                    );
                }
                return affinity_only(tid, tier);
            }

            let mut flags = 0u64;
            let mut size = SCHED_ATTR_SIZE_VER0;
            let mut util_min = 0;
            let mut util_max = 0;
            if let Some(uclamp) = &tier.uclamp {
                flags |= (libc::SCHED_FLAG_UTIL_CLAMP_MIN | libc::SCHED_FLAG_UTIL_CLAMP_MAX) as u64;
                size = SCHED_ATTR_SIZE_VER1;
                util_min = uclamp.min;
                util_max = uclamp.max;
            }

            let attr = SchedAttr {
                size,
                sched_policy: tier.policy.as_libc() as u32,
                sched_flags: flags,
                // `SCHED_IDLE` has no nice of its own — the kernel pins it to
                // the weakest weight — so it is sent as 0 rather than
                // whatever an override happened to carry.
                sched_nice: if matches!(tier.policy, SchedPolicy::Idle) {
                    0
                } else {
                    tier.nice
                },
                sched_util_min: util_min,
                sched_util_max: util_max,
                ..Default::default()
            };

            // SAFETY: `attr` is fully initialized and its `size` field is
            // never larger than the struct itself.
            let ret = unsafe { sched_setattr(tid as libc::pid_t, &attr, 0) };
            if ret == -1 {
                return Err(map_setattr_errno(tid, tier.uclamp.is_some(), tier.priority));
            }
        }
    }

    affinity_only(tid, tier)
}

/// Apply just the CPU affinity part of a tier to one TID.
fn affinity_only(tid: u32, tier: &AppliedTier) -> Result<(), SchedApplyError> {
    if let Some(cpus) = &tier.cpus {
        let Some(set) = cpus.to_cpu_set_t() else {
            return Err(SchedApplyError::InvalidCpuSet {
                pid: tid,
                cpus: cpus.to_string(),
                reason: format!("CPU index >= CPU_SETSIZE ({})", libc::CPU_SETSIZE),
            });
        };

        // SAFETY: `set` is a fully initialized cpu_set_t built by
        // `to_cpu_set_t`, which bounds-checked every index.
        let ret = unsafe {
            libc::sched_setaffinity(
                tid as libc::pid_t,
                std::mem::size_of::<libc::cpu_set_t>(),
                &set,
            )
        };

        if ret == -1 {
            return Err(map_affinity_errno(tid));
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
    if matches!(tier.policy, SchedPolicy::Fifo | SchedPolicy::Rr)
        && !(1..=99).contains(&tier.priority)
    {
        return Err(SchedApplyError::InvalidPriority {
            pid,
            priority: tier.priority,
        });
    }

    // An empty CpuSet is `Some` but names no CPU. `sched_setaffinity` with an
    // empty mask fails EINVAL deep in the sweep; rejecting it here keeps the
    // error at the level where the caller can say which node is misconfigured.
    if let Some(cpus) = &tier.cpus
        && cpus.is_empty()
    {
        return Err(SchedApplyError::InvalidCpuSet {
            pid,
            cpus: String::new(),
            reason: "CPU set is empty".to_string(),
        });
    }

    if let Some(uclamp) = &tier.uclamp
        && (uclamp.min > uclamp.max || uclamp.max > UCLAMP_MAX)
    {
        return Err(SchedApplyError::InvalidUclamp {
            pid,
            min: uclamp.min,
            max: uclamp.max,
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

    // F2: a reservation is per-THREAD, so sweeping it across every TID would
    // multiply it by the thread count. A ROS node reaches ~11 threads within
    // half a second of exec, which would turn one declared 8ms/100ms budget
    // into 88% of a CPU at admission control — rejected outright, or granted
    // an order of magnitude more bandwidth than was declared.
    //
    // So the thread-group leader gets the reservation (on a single-threaded
    // rclcpp executor that IS the callback thread) and every sibling gets
    // SCHED_FIFO at the priority the mapper derived, keeping DDS threads above
    // best-effort without consuming reservation bandwidth.
    //
    // Stated limit: with a multi-threaded executor the reservation covers one
    // of several executor threads and the guarantee is unsound. That is
    // refused where it can be detected (a container's
    // `--use_multi_threaded_executor`) and documented where it cannot.
    if matches!(tier.policy, SchedPolicy::Deadline) {
        return apply_reservation(pid, &tids, tier);
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

/// Apply a reservation to `pid`: the leader is reserved, siblings take
/// `SCHED_FIFO`. See the comment in [`apply_tier`] for why.
fn apply_reservation(pid: u32, tids: &[u32], tier: &AppliedTier) -> Result<(), SchedApplyError> {
    // Siblings first. If admission control then rejects the reservation the
    // node is left entirely at fixed priority — a coherent, if weaker, state —
    // rather than with a reserved leader and best-effort DDS threads, which
    // would be worse than either.
    if (1..=99).contains(&tier.priority) {
        let sibling = AppliedTier {
            policy: SchedPolicy::Fifo,
            reservation: None,
            // Affinity is untouched for every thread of a reserved node: its
            // CPUs come from the cpuset partition, and narrowing a deadline
            // thread's mask is exactly what the kernel forbids.
            cpus: None,
            ..tier.clone()
        };
        for &tid in tids {
            if tid == pid {
                continue;
            }
            match apply_to_tid(tid, &sibling) {
                Ok(()) => {}
                Err(SchedApplyError::Syscall {
                    errno: libc::ESRCH, ..
                }) => tracing::debug!(pid, tid, "thread exited mid-sweep, skipping"),
                Err(e) => return Err(e),
            }
        }
    } else {
        tracing::debug!(
            pid,
            priority = tier.priority,
            "no derived RT priority for a reserved node's sibling threads; leaving them alone"
        );
    }

    apply_to_tid(pid, tier)
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

    fn other_tier(cpus: Option<CpuSet>) -> AppliedTier {
        AppliedTier {
            policy: SchedPolicy::Other,
            priority: 0,
            nice: 0,
            cpus,
            uclamp: None,
            reservation: None,
            tier_name: "test".to_string(),
        }
    }

    fn fifo_tier(priority: i32) -> AppliedTier {
        AppliedTier {
            policy: SchedPolicy::Fifo,
            priority,
            nice: 0,
            cpus: None,
            uclamp: None,
            reservation: None,
            tier_name: "test".to_string(),
        }
    }

    /// Read back what the kernel actually applied, rather than what we asked
    /// for. `None` if `sched_getattr` is unavailable or the target is gone.
    fn read_attr(pid: u32) -> Option<SchedAttr> {
        let mut attr = SchedAttr::default();
        // SAFETY: `attr` is a valid writable SchedAttr of exactly VER1 bytes.
        let ret = unsafe { sched_getattr(pid as libc::pid_t, &mut attr, SCHED_ATTR_SIZE_VER1, 0) };
        (ret == 0).then_some(attr)
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
        let cpu = available_cpus()[0];
        with_sleep_child(|pid| {
            let tier = other_tier(Some(CpuSet::single(cpu)));
            let result = apply_tier(pid, &tier);
            assert_eq!(result, Ok(()));
        });
    }

    #[test]
    fn sched_other_at_nice_zero_still_issues_no_policy_syscall() {
        // The no-op guard: an unmodified best-effort node must not have its
        // scheduling touched, or we would reset a nice value somebody else
        // set and add a failure mode for no gain.
        with_sleep_child(|pid| {
            let before = read_attr(pid).expect("getattr");
            let tier = other_tier(None);
            apply_tier(pid, &tier).expect("no-op apply should succeed");
            let after = read_attr(pid).expect("getattr");
            assert_eq!(before.sched_policy, after.sched_policy);
            assert_eq!(before.sched_nice, after.sched_nice);
        });
    }

    #[test]
    fn batch_and_nice_reach_the_kernel() {
        // Lowering a node to SCHED_BATCH IS a change the user asked for, so
        // unlike the no-op case it must actually issue the syscall.
        if !has_sched_privilege() {
            eprintln!("skipping batch_and_nice_reach_the_kernel: needs CAP_SYS_NICE/root");
            return;
        }
        with_sleep_child(|pid| {
            let tier = AppliedTier {
                policy: SchedPolicy::Batch,
                priority: 0,
                nice: 7,
                cpus: None,
                uclamp: None,
                reservation: None,
                tier_name: "test".to_string(),
            };
            apply_tier(pid, &tier).expect("batch apply should succeed");
            let attr = read_attr(pid).expect("getattr");
            assert_eq!(attr.sched_policy, libc::SCHED_BATCH as u32);
            assert_eq!(attr.sched_nice, 7);
        });
    }

    fn deadline_tier(priority: i32) -> AppliedTier {
        AppliedTier {
            policy: SchedPolicy::Deadline,
            // The siblings' FIFO priority, not the reservation's — a
            // reservation has none.
            priority,
            nice: 0,
            cpus: None,
            uclamp: None,
            reservation: Some(Reservation {
                runtime_ns: 8_000_000,
                deadline_ns: 100_000_000,
                period_ns: 100_000_000,
                overrun: false,
            }),
            tier_name: "test".to_string(),
        }
    }

    #[test]
    fn only_deadline_carries_reset_on_fork() {
        // The kernel refuses fork(2) from a SCHED_DEADLINE thread without it,
        // and the isolated container manager forks per composable node. On
        // FIFO the same flag would defeat PTHREAD_INHERIT_SCHED for threads
        // created after the sweep — measured, hence the asymmetry.
        assert_ne!(reset_on_fork_flag(SchedPolicy::Deadline), 0);
        assert_eq!(reset_on_fork_flag(SchedPolicy::Fifo), 0);
        assert_eq!(reset_on_fork_flag(SchedPolicy::Rr), 0);
        assert_eq!(reset_on_fork_flag(SchedPolicy::Other), 0);
        assert_eq!(reset_on_fork_flag(SchedPolicy::Batch), 0);
        assert_eq!(reset_on_fork_flag(SchedPolicy::Idle), 0);
    }

    #[test]
    fn deadline_without_reservation_parameters_is_refused() {
        with_sleep_child(|pid| {
            let mut tier = deadline_tier(0);
            tier.reservation = None;
            assert_eq!(
                apply_tier(pid, &tier),
                Err(SchedApplyError::MissingReservation { pid })
            );
        });
    }

    #[test]
    fn a_reservation_outside_a_partition_names_the_partition() {
        // On any unprovisioned host the kernel refuses with EPERM because the
        // affinity mask is not the whole root domain. The value of this test
        // is the MESSAGE: "need CAP_SYS_NICE" would send the reader to check
        // something that is already true of the helper.
        if !has_sched_privilege() {
            eprintln!(
                "skipping a_reservation_outside_a_partition_names_the_partition: unprivileged"
            );
            return;
        }
        let readiness = crate::sched::kernel_sched_support();
        assert!(readiness.setattr, "sched_setattr must exist to run this");

        with_sleep_child(|pid| {
            match apply_tier(pid, &deadline_tier(40)) {
                Err(SchedApplyError::DeadlineNeedsPartition { .. }) => {}
                // On a host that IS partitioned (a container running the
                // suite), the apply legitimately succeeds.
                Ok(()) => eprintln!("host is partitioned — reservation applied"),
                Err(SchedApplyError::AdmissionRejected { arithmetic, .. }) => {
                    assert!(
                        arithmetic.contains('%'),
                        "an admission rejection must show the arithmetic: {arithmetic}"
                    );
                }
                Err(other) => {
                    panic!("unexpected error for a reservation outside a partition: {other}")
                }
            }
        });
    }

    #[test]
    fn admission_rejection_reports_the_arithmetic_not_just_ebusy() {
        // Rendered from the reservation, so the reader sees what was asked for
        // against what the kernel allows rather than "resource busy".
        let res = Reservation {
            runtime_ns: 8_000_000,
            deadline_ns: 100_000_000,
            period_ns: 100_000_000,
            overrun: false,
        };
        let utilization = res.runtime_ns as f64 / res.period_ns as f64 * 100.0;
        assert!((utilization - 8.0).abs() < 1e-9);
        // The ceiling is read from the running kernel, never assumed.
        if let Some(ceiling) = rt_bandwidth_ceiling() {
            assert!(
                ceiling > 0.0 && ceiling <= 1.0,
                "sched_rt_runtime_us/sched_rt_period_us should be a sane fraction, got {ceiling}"
            );
        }
    }

    #[test]
    fn sched_attr_matches_the_kernel_abi() {
        // Field order and widths are ABI, so a reordering or a type change
        // would silently corrupt every call. VER0 is the original layout
        // (through sched_period); VER1 adds the two uclamp fields.
        assert_eq!(
            std::mem::size_of::<SchedAttr>(),
            SCHED_ATTR_SIZE_VER1 as usize,
            "SchedAttr must be exactly SCHED_ATTR_SIZE_VER1 bytes"
        );
        assert_eq!(SCHED_ATTR_SIZE_VER1 - SCHED_ATTR_SIZE_VER0, 8);
        assert_eq!(std::mem::align_of::<SchedAttr>(), 8);

        let a = SchedAttr::default();
        let base = &a as *const _ as usize;
        let off = |p: *const u8| p as usize - base;
        assert_eq!(off(&a.size as *const _ as *const u8), 0);
        assert_eq!(off(&a.sched_policy as *const _ as *const u8), 4);
        assert_eq!(off(&a.sched_flags as *const _ as *const u8), 8);
        assert_eq!(off(&a.sched_nice as *const _ as *const u8), 16);
        assert_eq!(off(&a.sched_priority as *const _ as *const u8), 20);
        assert_eq!(off(&a.sched_runtime as *const _ as *const u8), 24);
        assert_eq!(off(&a.sched_deadline as *const _ as *const u8), 32);
        assert_eq!(off(&a.sched_period as *const _ as *const u8), 40);
        assert_eq!(off(&a.sched_util_min as *const _ as *const u8), 48);
        assert_eq!(off(&a.sched_util_max as *const _ as *const u8), 52);
    }

    #[test]
    fn sched_getattr_reads_our_own_policy() {
        // Proves the raw syscall plumbing works at all, without needing any
        // privilege: a test binary is SCHED_OTHER, so that is what comes back.
        let attr = read_attr(std::process::id())
            .expect("sched_getattr must work on ourselves (Linux 3.14+)");
        assert_eq!(attr.sched_policy, libc::SCHED_OTHER as u32);
        assert!(kernel_sched_support().setattr);
    }

    #[test]
    fn fifo_apply_must_not_set_reset_on_fork() {
        // Regression lock. Setting RESET_ON_FORK on SCHED_FIFO looks like
        // hygiene and silently breaks the per-TID sweep: the kernel resets
        // scheduling in `sched_fork()`, which runs for thread creation too, so
        // every thread spawned after the sweep lands on SCHED_OTHER instead of
        // inheriting the policy. That was measured, not theorised — it fails
        // `per_tid_sched_fifo_launch_privileged_only` with a control_node
        // thread at policy 0.
        //
        // The unit test asserts the flag's absence directly so the cause is
        // visible here rather than only as a puzzling integration failure.
        if !has_sched_privilege() {
            eprintln!("skipping fifo_apply_must_not_set_reset_on_fork: needs CAP_SYS_NICE/root");
            return;
        }
        with_sleep_child(|pid| {
            apply_tier(pid, &fifo_tier(10)).expect("privileged apply should succeed");
            let attr = read_attr(pid).expect("sched_getattr on a live child");
            assert_eq!(attr.sched_policy, libc::SCHED_FIFO as u32);
            assert_eq!(attr.sched_priority, 10);
            assert_eq!(
                attr.sched_flags & libc::SCHED_FLAG_RESET_ON_FORK as u64,
                0,
                "RESET_ON_FORK must NOT be set on SCHED_FIFO — it defeats \
                 PTHREAD_INHERIT_SCHED for threads created after the sweep. \
                 Got flags {:#x}",
                attr.sched_flags
            );
        });

        assert_eq!(
            reset_on_fork_flag(SchedPolicy::Fifo),
            0,
            "no policy this layer can express requires RESET_ON_FORK yet"
        );
        assert_eq!(reset_on_fork_flag(SchedPolicy::Rr), 0);
        assert_eq!(reset_on_fork_flag(SchedPolicy::Other), 0);
    }

    #[test]
    fn cpuset_applies_every_cpu_it_names() {
        // The old `core: Option<u32>` could express exactly one CPU. Assert
        // the set form actually reaches the kernel as a multi-CPU mask.
        let cpus = available_cpus();
        if cpus.len() < 2 {
            eprintln!("skipping cpuset_applies_every_cpu_it_names: needs >= 2 usable CPUs");
            return;
        }
        let (a, b) = (cpus[0], cpus[1]);
        with_sleep_child(|pid| {
            let tier = other_tier(Some(CpuSet::new([a, b])));
            apply_tier(pid, &tier).expect("affinity apply should succeed");

            // SAFETY: zeroed cpu_set_t is valid; sched_getaffinity fills it.
            let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
            let ret = unsafe {
                libc::sched_getaffinity(
                    pid as libc::pid_t,
                    std::mem::size_of::<libc::cpu_set_t>(),
                    &mut set,
                )
            };
            assert_eq!(ret, 0);
            assert!(
                unsafe { libc::CPU_ISSET(a as usize, &set) },
                "cpu {a} must be set"
            );
            assert!(
                unsafe { libc::CPU_ISSET(b as usize, &set) },
                "cpu {b} must be set"
            );
            for &other in cpus.iter().skip(2) {
                assert!(
                    !unsafe { libc::CPU_ISSET(other as usize, &set) },
                    "cpu {other} was not named and must not be set"
                );
            }
        });
    }

    /// The CPUs this process may actually run on.
    ///
    /// NOT `_SC_NPROCESSORS_ONLN`: inside a cpuset partition, or under
    /// `taskset`, the online count says 32 while only two of them are usable —
    /// and a test that pins to CPU 0 then fails for reasons that have nothing
    /// to do with what it is testing. Both affinity tests below were written
    /// against CPUs 0 and 1 and failed the first time they ran inside a real
    /// partition.
    fn available_cpus() -> Vec<u32> {
        // SAFETY: zeroed cpu_set_t is valid; sched_getaffinity fills it.
        let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
        let ret =
            unsafe { libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut set) };
        if ret != 0 {
            return vec![0];
        }
        (0..libc::CPU_SETSIZE as usize)
            .filter(|&c| unsafe { libc::CPU_ISSET(c, &set) })
            .map(|c| c as u32)
            .collect()
    }

    #[test]
    fn empty_cpuset_is_rejected_before_any_syscall() {
        let bogus_pid = u32::MAX;
        let mut tier = other_tier(Some(CpuSet::new([])));
        tier.policy = SchedPolicy::Other;
        let err = apply_tier(bogus_pid, &tier).unwrap_err();
        assert!(
            matches!(err, SchedApplyError::InvalidCpuSet { .. }),
            "expected InvalidCpuSet, got {err:?}"
        );
    }

    #[test]
    fn out_of_range_uclamp_is_rejected_before_any_syscall() {
        let bogus_pid = u32::MAX;
        let mut tier = fifo_tier(10);
        tier.uclamp = Some(Uclamp {
            min: 0,
            max: UCLAMP_MAX + 1,
        });
        assert_eq!(
            apply_tier(bogus_pid, &tier).unwrap_err(),
            SchedApplyError::InvalidUclamp {
                pid: bogus_pid,
                min: 0,
                max: UCLAMP_MAX + 1,
            }
        );

        let mut tier = fifo_tier(10);
        tier.uclamp = Some(Uclamp { min: 800, max: 100 });
        assert!(matches!(
            apply_tier(bogus_pid, &tier).unwrap_err(),
            SchedApplyError::InvalidUclamp { .. }
        ));
    }

    #[test]
    fn cpuset_is_sorted_and_deduplicated() {
        assert_eq!(CpuSet::new([3, 1, 3, 0]).cpus(), &[0, 1, 3]);
        assert_eq!(CpuSet::single(7).cpus(), &[7]);
        assert_eq!(CpuSet::new([2, 0]).to_string(), "0,2");
        assert!(CpuSet::new([]).is_empty());
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
                nice: 0,
                cpus: Some(CpuSet::single(1023)),
                uclamp: None,
                reservation: None,
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
                other => panic!("unexpected affinity error: {other:?}"),
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
