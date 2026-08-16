//! Admission control for process startup.
//!
//! # What this does and does not fix
//!
//! Before this module, every member actor spawned its process the moment it
//! reached `Pending`, and every actor reached `Pending` at once. Measured on a
//! 12-core AGX Orin bringing up a 144-process Autoware launch, that put **442
//! tasks in the runnable queue with load1 at 203** while the machine sat at
//! ~10 of its 12 cores for the whole ~50 s startup. A desktop session sharing
//! that machine was starved, and the report that prompted this work had the
//! OOM killer take GNOME.
//!
//! The obvious inference — that the storm is contention, and that pacing the
//! spawns will fix it — **was measured and is wrong.** Same binary, same
//! launch, pacing on versus off:
//!
//! | | pacing off | 12 at a time |
//! |---|---|---|
//! | spawn → startup complete | 10.6 s | 23.8 s |
//! | peak runnable tasks | 525 | 471 |
//! | peak load1 | 181 | 184 |
//! | MemAvailable consumed | 3.8 GiB | 3.6 GiB |
//!
//! Pacing doubled the startup and bought about ten percent. The reason is that
//! the storm is not contention over a fixed amount of work — it is the work.
//! Those 144 processes are 84 `component_node` forks created by
//! `--container-mode isolated`, and their cost is incurred by *existing*, not
//! by starting simultaneously. Serialising the spawns stretches the same CPU
//! over a longer window. The lever that does move it is the process count
//! itself: `--container-mode observable` runs the same nodes as threads inside
//! their containers and brings the same system up on **3.9 cores instead of
//! 10.2** — peak load1 45 against 190, 1.4 GiB against 3.5, and it finishes
//! FASTER (8.4 s against 10.7 s) with the same 84/84 composables loaded.
//!
//! So the concurrency limit is **off by default**. What stays on is the memory
//! floor, which is not a throughput control at all: it costs nothing while
//! memory is plentiful — it never blocks — and only serialises admissions once
//! `MemAvailable` falls through the floor, which is precisely the condition
//! that ended with a dead desktop. Pair it with the `oom_score_adj` bias in
//! `node_cmdline`, which decides *who* dies if the floor is crossed anyway.
//!
//! # How a permit is released
//!
//! The subtle part is not acquiring a permit but releasing it. `spawn()`
//! returns in microseconds, so releasing on spawn would admit every node
//! immediately and change nothing. The permit is instead held until the child
//! has *finished initialising*, which is detected by watching its CPU usage
//! decay: a ROS node burns CPU hard while it links, parses parameters and
//! builds its DDS participant, then drops to near-idle. `settle_threshold_pct`
//! and `settle_samples` define "near-idle"; `max_settle` caps the wait so a
//! node that legitimately runs hot forever cannot wedge the queue.
//!
//! This feedback form was chosen over a fixed inter-spawn delay because a
//! fixed delay has to be tuned per machine and per launch file to be either
//! safe or fast, and is silently wrong when it is neither.
//!
//! # Deadlock is not an option
//!
//! Every gate here has a bypass. A launch that refuses to start is worse than
//! a launch that starts badly, so when the memory floor or the load ceiling
//! cannot be satisfied within `max_gate_wait`, the node is admitted anyway
//! with a warning naming the gate. The governor degrades to the old behaviour
//! under pressure rather than stalling.

use std::{
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};

use tokio::sync::{OwnedSemaphorePermit, Semaphore};
use tracing::{debug, info, warn};

/// Resolved admission-control limits.
///
/// Construct with [`StartupLimits::auto`] to derive them from the machine, or
/// build one directly when the user has stated what they want.
#[derive(Debug, Clone)]
pub struct StartupLimits {
    /// Maximum processes allowed to be starting at once. `usize::MAX` — the
    /// DEFAULT — means unlimited.
    ///
    /// Off by default because it was measured and did not pay: see the module
    /// documentation for the table. It doubled a 10.6 s startup for a ~10%
    /// reduction in peak runnable tasks, because the cost of a wide launch is
    /// the processes existing rather than their starting together. Kept
    /// because it is the right control on a machine that must stay responsive
    /// to something else during a launch, where trading startup latency for
    /// headroom is the point.
    pub max_concurrent: usize,

    /// Refuse to admit another process while `MemAvailable` is below this.
    /// Zero disables the memory gate.
    pub min_available_kb: u64,

    /// Refuse to admit another process while the number of runnable tasks
    /// exceeds `max_runnable_factor * ncpu`. Zero disables the gate, and zero
    /// is the default — see below.
    ///
    /// It reads `procs_running` from `/proc/stat` rather than loadavg, because
    /// load1 is a one-minute exponential average and lags a startup transient
    /// by most of the transient's duration.
    ///
    /// **Off by default, and measured that way.** A ceiling of `2 * ncpu` was
    /// tried first and made startup dramatically worse: it cannot tell "the
    /// machine is busy because we are starting things" from "the machine is
    /// busy because the things we started are running". Nodes that had already
    /// come up kept `procs_running` above the ceiling, so no further node could
    /// be admitted until the 30 s bypass fired — turning a 49 s startup into
    /// one still unfinished at 190 s, with a HIGHER peak load1 than the
    /// unpaced baseline (217 vs 183). Left available for an operator sharing a
    /// machine with something else, but it is not a default anyone should
    /// want — and `max_concurrent` is the better knob for that job, since it
    /// bounds what this run is doing rather than what the whole machine is.
    pub max_runnable_factor: f64,

    /// CPU usage (percent of one core) below which a starting process counts
    /// as settled.
    pub settle_threshold_pct: f64,

    /// Consecutive under-threshold samples required before releasing a permit.
    pub settle_samples: u32,

    /// How often to sample a starting child's CPU usage.
    pub settle_poll: Duration,

    /// Hard cap on how long one permit may be held.
    pub max_settle: Duration,

    /// Hard cap on how long the memory/load gates may block one admission
    /// before it is let through anyway.
    pub max_gate_wait: Duration,

    /// Maximum composable-node loads in flight across ALL containers.
    ///
    /// Separate from `max_concurrent` because the two have different release
    /// semantics: a process slot is released when the child settles, a load
    /// slot when the LoadNode call returns — which, under the existing retry
    /// budgets, can be minutes. Sharing one semaphore would let a single stuck
    /// container stall every plain node still waiting to spawn.
    ///
    /// This is what `composable_node_loading.max_concurrent_load_node_spawn`
    /// has always claimed to do. Until Phase 61 nothing read it:
    /// `dispatch_pending_loads` drained its queue and spawned a task per
    /// request, so all 84 composables of a 144-process launch were dispatched
    /// at once.
    pub max_concurrent_loads: usize,
}

impl StartupLimits {
    /// Admission control disabled — spawn everything at once, as before.
    pub fn unlimited() -> Self {
        Self {
            max_concurrent: usize::MAX,
            min_available_kb: 0,
            max_runnable_factor: 0.0,
            settle_threshold_pct: 0.0,
            settle_samples: 0,
            settle_poll: Duration::from_millis(200),
            max_settle: Duration::ZERO,
            max_gate_wait: Duration::ZERO,
            max_concurrent_loads: usize::MAX,
        }
    }

    /// Derive limits from the machine.
    ///
    /// Only the memory floor is derived and only the memory floor is on.
    ///
    /// It is an ABSOLUTE quantity, not a proportion of RAM. What the floor
    /// guards against is one more process allocating before the next sample
    /// notices, and a process needs what it needs regardless of how much
    /// memory the machine happens to have: an Autoware node wanting 250 MB
    /// wants 250 MB on a 4 GiB board and on a 64 GiB one. Measured on the golf
    /// cart stack, the largest launch-owned process peaked at **274 MiB** and
    /// the 99th percentile across every process on the machine was 116 MiB —
    /// so 1 GiB is roughly four times the largest single demand observed, with
    /// room for the CUDA/TensorRT allocations a sensor-attached run adds.
    ///
    /// An earlier version scaled it at 10% of RAM, clamped to
    /// [512 MiB, 4 GiB]. That was wrong in the direction that matters: it gave
    /// the 64 GiB box a 4 GiB cushion it did not need and the 4 GiB board
    /// 512 MiB — less than two of the processes it was meant to protect
    /// against, on the machine least able to absorb the miss.
    ///
    /// The one proportional term left is a CAP, so the floor cannot demand a
    /// quarter of a small board and spend every startup bouncing off its own
    /// gate. That is proportionality used to stay sane on tiny machines, not
    /// to size the reserve.
    ///
    /// `ncpu` is still taken because a caller that DOES want a concurrency
    /// limit almost always wants it expressed in cores, and because the
    /// runnable-task ceiling is scaled by it when enabled.
    pub fn auto(ncpu: usize, mem_total_kb: u64) -> Self {
        /// Headroom for the next process to allocate into. Absolute — see above.
        const FLOOR_KB: u64 = 1024 * 1024; // 1 GiB
        /// …but never more than this share of the machine.
        const MAX_FLOOR_FRACTION: u64 = 4;

        let floor = if mem_total_kb == 0 {
            FLOOR_KB
        } else {
            FLOOR_KB.min(mem_total_kb / MAX_FLOOR_FRACTION)
        };

        let _ = ncpu;
        Self {
            // See the field docs: measured, and not worth its cost by default.
            max_concurrent: usize::MAX,
            min_available_kb: floor,
            // Off: see the field's documentation for the measurement that
            // rejected a 2.0 ceiling.
            max_runnable_factor: 0.0,
            settle_threshold_pct: 20.0,
            settle_samples: 2,
            settle_poll: Duration::from_millis(250),
            max_settle: Duration::from_secs(15),
            max_gate_wait: Duration::from_secs(30),
            // Overwritten from `composable_node_loading
            // .max_concurrent_load_node_spawn` by the caller; this is the
            // same default that field already carried.
            max_concurrent_loads: 10,
        }
    }

    /// Whether any gate is active. A governor with none is a no-op and says
    /// so, rather than paying for a semaphore it never contends.
    pub fn is_enabled(&self) -> bool {
        self.max_concurrent != usize::MAX
            || self.min_available_kb > 0
            || self.max_runnable_factor > 0.0
            || self.max_concurrent_loads != usize::MAX
    }
}

/// Shared admission controller. One per run; every actor holds an `Arc`.
///
/// `Debug` is derived because `ActorConfig` is `Debug` and holds one; the
/// semaphores print as opaque handles, which is all a config dump wants.
#[derive(Debug)]
pub struct StartupGovernor {
    limits: StartupLimits,
    sem: Arc<Semaphore>,
    load_sem: Arc<Semaphore>,
    ncpu: usize,
    /// Processes admitted so far, for the progress line.
    admitted: AtomicU64,
    /// How many members will ask for admission, so the log can say "12/144"
    /// rather than a bare count. Zero when unknown.
    expected: u64,
}

impl StartupGovernor {
    pub fn new(limits: StartupLimits, expected: u64) -> Self {
        let permits = limits.max_concurrent.min(Semaphore::MAX_PERMITS);
        let load_permits = limits.max_concurrent_loads.min(Semaphore::MAX_PERMITS);
        Self {
            sem: Arc::new(Semaphore::new(permits)),
            load_sem: Arc::new(Semaphore::new(load_permits)),
            ncpu: std::thread::available_parallelism()
                .map(|n| n.get())
                .unwrap_or(1),
            limits,
            admitted: AtomicU64::new(0),
            expected,
        }
    }

    /// A governor that admits everything immediately.
    pub fn disabled() -> Self {
        Self::new(StartupLimits::unlimited(), 0)
    }

    pub fn limits(&self) -> &StartupLimits {
        &self.limits
    }

    /// Wait until it is this member's turn to spawn.
    ///
    /// Returns a permit that must be handed to [`StartupPermit::hold_until_settled`]
    /// once the child's pid is known. Dropping it early simply releases the
    /// slot, which is the right behaviour when the spawn failed.
    pub async fn admit(&self, who: &str) -> StartupPermit {
        if !self.limits.is_enabled() {
            return StartupPermit::inert();
        }

        let waited = Instant::now();

        // With no concurrency limit there is no slot to take, but the resource
        // gates still apply — that combination (unbounded parallelism, bounded
        // by memory pressure alone) is the default, so it must not allocate a
        // permit or spawn a settle-watcher per node for nothing.
        if self.limits.max_concurrent == usize::MAX {
            self.wait_for_resources(who, waited).await;
            return StartupPermit::inert();
        }

        let permit = self
            .sem
            .clone()
            .acquire_owned()
            .await
            .expect("startup semaphore is never closed");

        self.wait_for_resources(who, waited).await;

        let n = self.admitted.fetch_add(1, Ordering::Relaxed) + 1;
        if self.expected > 0 {
            debug!("startup: admitting {who} ({n}/{} )", self.expected);
        } else {
            debug!("startup: admitting {who} ({n})");
        }

        StartupPermit {
            inner: Some(permit),
            limits: self.limits.clone(),
        }
    }

    /// Wait for a composable-node load slot.
    ///
    /// The returned permit is released by dropping it, which the caller does
    /// by letting it fall out of scope when the LoadNode call returns. There
    /// is no settle phase: "loaded" is already the completion signal a process
    /// spawn has to infer from CPU decay.
    pub async fn admit_load(&self, who: &str) -> Option<OwnedSemaphorePermit> {
        if self.limits.max_concurrent_loads == usize::MAX {
            return None;
        }

        let waited = Instant::now();
        let permit = self
            .load_sem
            .clone()
            .acquire_owned()
            .await
            .expect("startup load semaphore is never closed");

        let queued = waited.elapsed();
        if queued > Duration::from_millis(500) {
            debug!(
                "startup: load slot for {who} waited {:.1}s",
                queued.as_secs_f64()
            );
        }
        Some(permit)
    }

    /// Block until the memory floor and the runnable-task ceiling are both
    /// satisfied, or `max_gate_wait` elapses.
    async fn wait_for_resources(&self, who: &str, since: Instant) {
        if self.limits.max_gate_wait.is_zero() {
            return;
        }
        let deadline = Instant::now() + self.limits.max_gate_wait;
        let mut logged = false;

        loop {
            let blocker = self.blocking_gate();
            let Some(reason) = blocker else {
                if logged {
                    info!(
                        "startup: {who} released after {:.1}s",
                        since.elapsed().as_secs_f64()
                    );
                }
                return;
            };

            if Instant::now() >= deadline {
                // Admitting anyway is deliberate: a launch that never starts
                // is a worse failure than one that starts under pressure, and
                // silently waiting forever would look like a hang.
                warn!(
                    "startup: admitting {who} despite {reason} — waited {:.0}s \
                     (raise the limit or lower the load; see startup config)",
                    self.limits.max_gate_wait.as_secs_f64()
                );
                return;
            }

            if !logged {
                debug!("startup: holding {who} — {reason}");
                logged = true;
            }
            tokio::time::sleep(Duration::from_millis(200)).await;
        }
    }

    /// Which gate, if any, currently forbids an admission.
    fn blocking_gate(&self) -> Option<String> {
        if self.limits.min_available_kb > 0
            && let Some(avail) = read_mem_available_kb()
            && avail < self.limits.min_available_kb
        {
            return Some(format!(
                "MemAvailable {} MiB below floor {} MiB",
                avail / 1024,
                self.limits.min_available_kb / 1024
            ));
        }

        if self.limits.max_runnable_factor > 0.0
            && let Some(running) = read_procs_running()
        {
            let ceiling = (self.limits.max_runnable_factor * self.ncpu as f64) as u64;
            if running > ceiling {
                return Some(format!("{running} runnable tasks above ceiling {ceiling}"));
            }
        }

        None
    }
}

/// A held admission slot.
pub struct StartupPermit {
    /// `None` for an inert permit (governor disabled) — dropping it does
    /// nothing and holding it costs nothing.
    inner: Option<OwnedSemaphorePermit>,
    limits: StartupLimits,
}

impl StartupPermit {
    fn inert() -> Self {
        Self {
            inner: None,
            limits: StartupLimits::unlimited(),
        }
    }

    /// Release this slot once `pid` has finished initialising.
    ///
    /// Spawns a detached task rather than blocking the caller: the actor must
    /// go on to supervise its child, and the permit's lifetime is about the
    /// *machine's* readiness for the next spawn, not this actor's control
    /// flow.
    pub fn hold_until_settled(mut self, pid: u32, name: String) {
        let Some(permit) = self.inner.take() else {
            return;
        };
        let limits = self.limits.clone();

        tokio::spawn(async move {
            let started = Instant::now();
            let mut prev = read_proc_cpu_ticks(pid);
            let mut quiet = 0u32;

            loop {
                if started.elapsed() >= limits.max_settle {
                    debug!(
                        "[{name}] startup slot released after {:.1}s cap",
                        started.elapsed().as_secs_f64()
                    );
                    break;
                }
                tokio::time::sleep(limits.settle_poll).await;

                let Some(now) = read_proc_cpu_ticks(pid) else {
                    // The process is gone. Whatever it was doing, it is not
                    // competing for CPU any more.
                    break;
                };
                let Some(before) = prev else {
                    prev = Some(now);
                    continue;
                };

                let ticks = now.saturating_sub(before);
                let pct =
                    ticks as f64 / clock_ticks_per_sec() as f64 / limits.settle_poll.as_secs_f64()
                        * 100.0;
                prev = Some(now);

                if pct < limits.settle_threshold_pct {
                    quiet += 1;
                    if quiet >= limits.settle_samples {
                        debug!(
                            "[{name}] settled at {pct:.0}% after {:.1}s",
                            started.elapsed().as_secs_f64()
                        );
                        break;
                    }
                } else {
                    quiet = 0;
                }
            }

            drop(permit);
        });
    }
}

/// `MemAvailable` from `/proc/meminfo`, in kB.
///
/// `MemAvailable` and not `MemFree`: free memory on a machine that has been up
/// for a while is mostly page cache, and gating on `MemFree` would refuse to
/// start anything on a perfectly healthy system.
fn read_mem_available_kb() -> Option<u64> {
    let text = std::fs::read_to_string("/proc/meminfo").ok()?;
    for line in text.lines() {
        if let Some(rest) = line.strip_prefix("MemAvailable:") {
            return rest.split_whitespace().next()?.parse().ok();
        }
    }
    None
}

/// Total RAM in kB, for deriving the memory floor.
pub fn read_mem_total_kb() -> Option<u64> {
    let text = std::fs::read_to_string("/proc/meminfo").ok()?;
    for line in text.lines() {
        if let Some(rest) = line.strip_prefix("MemTotal:") {
            return rest.split_whitespace().next()?.parse().ok();
        }
    }
    None
}

/// `procs_running` from `/proc/stat` — the instantaneous count of runnable
/// tasks, which is what oversubscription actually looks like.
fn read_procs_running() -> Option<u64> {
    let text = std::fs::read_to_string("/proc/stat").ok()?;
    for line in text.lines() {
        if let Some(rest) = line.strip_prefix("procs_running ") {
            return rest.trim().parse().ok();
        }
    }
    None
}

/// utime + stime for `pid`, in clock ticks. `None` once the process is gone.
///
/// `comm` (field 2) may contain both spaces and parentheses, so the fields are
/// located from the LAST `)` rather than by splitting the whole line.
fn read_proc_cpu_ticks(pid: u32) -> Option<u64> {
    let raw = std::fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
    let rp = raw.rfind(')')?;
    let fields: Vec<&str> = raw.get(rp + 2..)?.split_whitespace().collect();
    // fields[0] is field 3 (state), so field N is fields[N - 3].
    let utime: u64 = fields.get(11)?.parse().ok()?;
    let stime: u64 = fields.get(12)?.parse().ok()?;
    Some(utime + stime)
}

fn clock_ticks_per_sec() -> u64 {
    // 100 on every Linux configuration this runs on; read it anyway rather
    // than hard-coding a number that is only usually right.
    static CACHED: AtomicU64 = AtomicU64::new(0);
    let cached = CACHED.load(Ordering::Relaxed);
    if cached != 0 {
        return cached;
    }
    let hz = unsafe { libc::sysconf(libc::_SC_CLK_TCK) };
    let hz = if hz > 0 { hz as u64 } else { 100 };
    CACHED.store(hz, Ordering::Relaxed);
    hz
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unlimited_limits_are_a_no_op() {
        assert!(!StartupLimits::unlimited().is_enabled());
    }

    #[test]
    fn auto_leaves_concurrency_unlimited_but_keeps_the_memory_floor() {
        // The measured default: no throughput control, one safety net. If this
        // ever flips back to `ncpu`, the startup it paces doubles in length —
        // see the module docs.
        let small = StartupLimits::auto(4, 8 * 1024 * 1024);
        let big = StartupLimits::auto(64, 128 * 1024 * 1024);
        assert_eq!(small.max_concurrent, usize::MAX);
        assert_eq!(big.max_concurrent, usize::MAX);
        assert_eq!(small.max_runnable_factor, 0.0);
        assert!(small.min_available_kb > 0);
        assert!(
            small.is_enabled(),
            "the memory floor alone counts as enabled"
        );
    }

    #[tokio::test]
    async fn unlimited_concurrency_takes_no_permit() {
        // The default path must not allocate a permit or spawn a settle
        // watcher per node: with no limit there is no slot to hold.
        let mut limits = StartupLimits::auto(4, 8 * 1024 * 1024);
        limits.min_available_kb = 0;
        let gov = StartupGovernor::new(limits, 0);
        for i in 0..500 {
            let p = gov.admit(&format!("n{i}")).await;
            // An inert permit's settle hold is a no-op; calling it is the
            // production path and must not panic or leak a task.
            p.hold_until_settled(std::process::id(), format!("n{i}"));
        }
    }

    #[test]
    fn memory_floor_is_absolute_not_proportional() {
        const GIB: u64 = 1024 * 1024;

        // The whole point: the reserve a starting process needs does not
        // depend on how much memory the machine has, so the floor must be the
        // same on a small board and a large one.
        let small = StartupLimits::auto(4, 8 * GIB);
        let large = StartupLimits::auto(64, 128 * GIB);
        assert_eq!(small.min_available_kb, GIB);
        assert_eq!(large.min_available_kb, GIB);
        assert_eq!(
            small.min_available_kb, large.min_available_kb,
            "a 16x difference in RAM must not change the reserve"
        );
    }

    #[test]
    fn tiny_boards_cap_the_floor_rather_than_bouncing_off_it() {
        const GIB: u64 = 1024 * 1024;
        // 1 GiB of reserve on a 2 GiB board would block half the machine and
        // spend every startup hitting the bypass; the cap keeps it to a
        // quarter.
        let tiny = StartupLimits::auto(2, 2 * GIB);
        assert_eq!(tiny.min_available_kb, 2 * GIB / 4);
        assert!(tiny.min_available_kb < GIB);
    }

    #[test]
    fn unreadable_meminfo_still_yields_a_reserve() {
        // `read_mem_total_kb()` returning None resolves to 0; a floor of 0
        // would silently disable the one gate that ships enabled.
        let unknown = StartupLimits::auto(4, 0);
        assert_eq!(unknown.min_available_kb, 1024 * 1024);
    }

    #[test]
    fn self_cpu_ticks_are_readable_and_monotonic() {
        let pid = std::process::id();
        let a = read_proc_cpu_ticks(pid).expect("own /proc/self/stat is readable");
        // Burn a little CPU so the second read is not merely equal by luck of
        // the clock granularity.
        let mut x: u64 = 0;
        for i in 0..2_000_000u64 {
            x = x.wrapping_add(i);
        }
        std::hint::black_box(x);
        let b = read_proc_cpu_ticks(pid).unwrap();
        assert!(b >= a, "cpu ticks went backwards: {a} -> {b}");
    }

    #[test]
    fn dead_pid_reads_none() {
        // pid 0 is never a real process; /proc/0/stat does not exist.
        assert!(read_proc_cpu_ticks(0).is_none());
    }

    #[test]
    fn meminfo_and_stat_are_parseable_on_this_host() {
        assert!(read_mem_total_kb().is_some_and(|kb| kb > 0));
        assert!(read_mem_available_kb().is_some_and(|kb| kb > 0));
        assert!(read_procs_running().is_some());
    }

    #[tokio::test]
    async fn disabled_governor_admits_immediately() {
        let gov = StartupGovernor::disabled();
        // 200 admissions with no permits held would block forever if the
        // semaphore were real.
        for i in 0..200 {
            let permit = gov.admit(&format!("node{i}")).await;
            std::mem::forget(permit);
        }
    }

    #[tokio::test]
    async fn load_slots_are_separate_from_process_slots() {
        // The whole reason there are two semaphores: a container stuck in
        // LoadNode for minutes must not stop plain nodes from spawning.
        let mut limits = StartupLimits::auto(2, 8 * 1024 * 1024);
        limits.min_available_kb = 0;
        limits.max_runnable_factor = 0.0;
        limits.max_concurrent_loads = 1;
        let gov = Arc::new(StartupGovernor::new(limits, 0));

        let stuck_load = gov.admit_load("slow_composable").await;
        assert!(stuck_load.is_some());

        // Both process slots remain available while that load is held.
        let a = tokio::time::timeout(Duration::from_millis(500), gov.admit("node_a"))
            .await
            .expect("a held load slot must not block a process slot");
        let b = tokio::time::timeout(Duration::from_millis(500), gov.admit("node_b"))
            .await
            .expect("a held load slot must not block a process slot");
        drop((a, b, stuck_load));
    }

    #[tokio::test]
    async fn unlimited_loads_hand_back_no_permit() {
        // `max_concurrent_loads == usize::MAX` is the opt-out, and it must not
        // allocate a permit that a caller might then hold pointlessly.
        let mut limits = StartupLimits::auto(2, 8 * 1024 * 1024);
        limits.max_concurrent_loads = usize::MAX;
        let gov = StartupGovernor::new(limits, 0);
        assert!(gov.admit_load("anything").await.is_none());
    }

    #[tokio::test]
    async fn concurrency_limit_binds() {
        let mut limits = StartupLimits::auto(2, 8 * 1024 * 1024);
        // The limit is opt-in now, so state it rather than inheriting it.
        limits.max_concurrent = 2;
        // Isolate the concurrency gate from the resource gates, which depend
        // on whatever else is running on the test machine.
        limits.min_available_kb = 0;
        limits.max_runnable_factor = 0.0;
        let gov = Arc::new(StartupGovernor::new(limits, 0));

        let a = gov.admit("a").await;
        let b = gov.admit("b").await;

        let gov2 = gov.clone();
        let third = tokio::spawn(async move { gov2.admit("c").await });
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(!third.is_finished(), "third admission should be queued");

        drop(a);
        let c = tokio::time::timeout(Duration::from_secs(2), third)
            .await
            .expect("third admission should proceed once a slot frees")
            .unwrap();
        drop(b);
        drop(c);
    }
}
