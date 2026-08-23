use eyre::{Result, WrapErr};
use glob::Pattern;
use serde::Deserialize;
use std::path::Path;

/// Runtime configuration for play_launch
#[derive(Debug, Clone, Deserialize, Default)]
pub struct RuntimeConfig {
    /// Resource monitoring settings
    #[serde(default)]
    pub monitoring: MonitoringSettings,

    /// Composable node loading settings
    #[serde(default)]
    pub composable_node_loading: ComposableNodeLoadingSettings,

    /// Container readiness checking settings
    #[serde(default)]
    pub container_readiness: ContainerReadinessSettings,

    /// Diagnostics monitoring settings
    #[serde(default)]
    pub diagnostics: DiagnosticsSettings,

    /// RCL interception settings (LD_PRELOAD-based message introspection)
    #[serde(default)]
    pub interception: InterceptionSettings,

    /// Startup admission control (phase-61)
    #[serde(default)]
    pub startup: StartupSettings,

    /// Per-process configurations
    #[serde(default)]
    pub processes: Vec<ProcessConfig>,

    /// Per-container / per-node cgroup limits (phase-66 W2)
    #[serde(default)]
    pub cgroups: CgroupSettings,
}

/// Phase 66 W2 — limits applied to the per-node and per-container cgroups
/// created by [`crate::execution::cgroup`].
///
/// Everything is unset by default. W1 built the tree to *account* and *tear
/// down*; this makes limits expressible, not mandatory. A wrong `memory_max`
/// converts a slow launch into a killed one, and nobody has the per-system
/// knowledge to pick one for someone else's vehicle.
///
/// Inert when the cgroup tree is unavailable, which is the ordinary case —
/// play_launch started from a terminal cannot create cgroups at all.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct CgroupSettings {
    /// Rules in declaration order. The FIRST match wins, so a specific pattern
    /// belongs above a general one — the same precedence `startup.order` uses,
    /// and the one a reader assumes when scanning a list top to bottom.
    pub limits: Vec<CgroupLimitGroup>,
}

/// One `cgroups.limits` entry.
///
/// ```yaml
/// cgroups:
///   limits:
///     - match: ["/perception/**"]
///       memory_high_mb: 4096
///       oom_group: true
///     - match: ["**"]
///       pids_max: 2048
/// ```
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct CgroupLimitGroup {
    /// Glob patterns matched against the member's FQN (`/ns/node_name`), as in
    /// `startup.order`. `**` crosses namespace separators.
    #[serde(rename = "match")]
    pub match_: Vec<String>,

    /// `memory.high` — a THROTTLE, not a limit. Exceeding it puts the group
    /// under reclaim pressure and slows it; nothing is killed. This is the
    /// principled version of what `oom_score_adj` guesses at: instead of
    /// nominating a victim in advance, make the launch yield memory rather
    /// than take it.
    pub memory_high_mb: Option<u64>,

    /// `memory.max` — the hard ceiling. Allocation past it triggers OOM inside
    /// the group. Prefer `memory_high_mb` unless a hard bound is genuinely
    /// wanted: `max` kills, `high` slows.
    pub memory_max_mb: Option<u64>,

    /// `pids.max` — a ceiling on tasks in the group, threads included. A ROS
    /// node is 11-22 threads measured, so a container of ten composables is a
    /// couple of hundred; this turns runaway thread creation into a clean
    /// `EAGAIN` instead of a machine nobody can log into.
    pub pids_max: Option<u64>,

    /// `memory.oom.group` — whether the group is a FAULT UNIT.
    ///
    /// `true`: an OOM anywhere in the container kills every member together,
    /// which is what a real ROS container does (they share one process, so
    /// they die together whether or not anyone chose that). `false` (the
    /// default): only the offending process dies, which is the isolation
    /// `--container-mode isolated` forks to buy.
    ///
    /// Worth setting `true` where partial survival is the more dangerous
    /// outcome: a pipeline container that loses one stage keeps publishing
    /// stale or absent data, and a supervisor restarts a dead thing while
    /// never noticing a degraded one.
    ///
    /// Meaningless on a plain node's group — it holds one process.
    pub oom_group: Option<bool>,
}

/// Phase 61: how fast play_launch is allowed to start processes.
///
/// Every field defaults to "derive it from this machine", so the common case
/// needs no configuration and a workstation is effectively unthrottled — the
/// concurrency limit is the core count, which only binds when the launch is
/// wider than the machine.
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct StartupSettings {
    /// Master switch. `false` restores the pre-phase-61 behaviour of spawning
    /// every process the instant its actor starts.
    pub enabled: bool,

    /// Maximum processes starting at once. `None` (the default, written as an
    /// absent key or `~`) means UNLIMITED — the gate is off, because pacing to
    /// one-per-core was measured to double a 10.6 s startup for a ~10% cut in
    /// peak runnable tasks. See `StartupLimits::auto`, which records the
    /// numbers at the field.
    pub max_concurrent: Option<usize>,

    /// Refuse to start another process while `MemAvailable` is below this many
    /// MiB. `None` means an ABSOLUTE 1 GiB, capped at a quarter of RAM — not a
    /// percentage: 10% would give a 64 GiB box 4 GiB it does not need and a
    /// 4 GiB board 512 MiB, less than two of the processes it is protecting
    /// against. What the floor guards is one more process allocating before
    /// the next sample, and the largest launch-owned process measured 274 MiB
    /// regardless of machine size. See `StartupLimits::auto`.
    pub min_available_mb: Option<u64>,

    /// Refuse to start another process while runnable tasks exceed this
    /// multiple of the core count. `0` (the default) disables the gate —
    /// `StartupLimits::max_runnable_factor` records why it is off rather than
    /// set to something plausible.
    pub max_runnable_factor: f64,

    /// How long the memory/runnable gates may hold one admission before it is
    /// let through anyway with a warning. A launch that never starts is worse
    /// than one that starts under pressure.
    pub max_gate_wait_secs: u64,

    /// Minimum milliseconds between bypasses once `max_gate_wait_secs` has
    /// expired. `0` (the default) releases every held admission at once.
    ///
    /// Waiters all start at nearly the same instant, so they all reach the
    /// deadline at nearly the same instant: W3 measured 24 held and 24
    /// released together. Without a stagger the floor DELAYS the storm rather
    /// than spreading it, and delivers it at the moment memory was already
    /// declared short.
    pub bypass_stagger_ms: u64,

    /// Longest a single process may hold its startup slot, in seconds. Caps
    /// the CPU-decay wait for a node that legitimately never goes idle.
    pub max_settle_secs: u64,

    /// CPU percent (of one core) below which a starting process is considered
    /// to have finished initialising.
    pub settle_threshold_pct: f64,

    /// Phase 61 W2: groups of members that start after every earlier group has
    /// come up. Anything not matched by a group starts first.
    ///
    /// Empty by default. Reordering startup is a semantic change — a system
    /// where something waits on a driver being up early would break — so it is
    /// opt-in. The use it exists for is putting sensor drivers last, so nothing
    /// is published into subscribers that do not exist yet:
    ///
    /// ```yaml
    /// startup:
    ///   order:
    ///     - name: sensor-drivers
    ///       match: ["/sensing/**"]
    /// ```
    pub order: Vec<StartupOrderGroup>,

    /// Derive a final group from the model's topic graph: nodes that publish
    /// and never subscribe start last.
    ///
    /// Inert unless the model carries topics, which it does only when
    /// manifests have been authored — a plain launch file resolves to zero of
    /// them. Explicit `order` groups win over this.
    pub defer_sources: bool,

    /// How long one stage may hold up the next before it is released anyway,
    /// with the members still missing named in the warning. A driver whose
    /// hardware is absent never appears in the ROS graph, and must not stop the
    /// rest of the system from starting.
    pub stage_timeout_secs: u64,
}

/// One `startup.order` entry.
#[derive(Debug, Clone, Deserialize)]
pub struct StartupOrderGroup {
    /// Label for logs. Defaults to `stageN`.
    #[serde(default)]
    pub name: String,
    /// Glob patterns matched against node FQNs (`/ns/node_name`). `**` crosses
    /// namespace separators, so `/sensing/**` reaches a driver several levels
    /// down.
    #[serde(default, rename = "match")]
    pub match_: Vec<String>,
}

impl Default for StartupSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            max_concurrent: None,
            min_available_mb: None,
            max_runnable_factor: 0.0,
            max_gate_wait_secs: 30,
            bypass_stagger_ms: 0,
            max_settle_secs: 15,
            settle_threshold_pct: 20.0,
            order: Vec::new(),
            defer_sources: false,
            stage_timeout_secs: 30,
        }
    }
}

impl StartupSettings {
    /// Resolve to concrete limits for this machine.
    ///
    /// `max_concurrent_loads` comes from the composable-loading section rather
    /// than from here: it is the same quantity
    /// `composable_node_loading.max_concurrent_load_node_spawn` has always
    /// named, and giving it a second name in a second section would be a way
    /// for the two to disagree.
    pub fn resolve(
        &self,
        max_concurrent_loads: usize,
    ) -> crate::execution::startup_governor::StartupLimits {
        use crate::execution::startup_governor::{StartupLimits, read_mem_total_kb};

        if !self.enabled {
            return StartupLimits::unlimited();
        }

        let ncpu = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);
        let mem_total_kb = read_mem_total_kb().unwrap_or(0);

        let mut limits = StartupLimits::auto(ncpu, mem_total_kb);
        if let Some(n) = self.max_concurrent {
            // 0 would mean "start nothing"; read it as "no limit" instead,
            // which is the only interpretation that is not a deadlock.
            limits.max_concurrent = if n == 0 { usize::MAX } else { n };
        }
        if let Some(mb) = self.min_available_mb {
            limits.min_available_kb = mb.saturating_mul(1024);
        }
        limits.max_runnable_factor = self.max_runnable_factor;
        limits.max_gate_wait = std::time::Duration::from_secs(self.max_gate_wait_secs);
        limits.bypass_stagger = std::time::Duration::from_millis(self.bypass_stagger_ms);
        limits.max_settle = std::time::Duration::from_secs(self.max_settle_secs);
        limits.settle_threshold_pct = self.settle_threshold_pct;
        limits.max_concurrent_loads = if max_concurrent_loads == 0 {
            usize::MAX
        } else {
            max_concurrent_loads
        };
        limits
    }
}

/// Global monitoring settings
#[derive(Debug, Clone, Deserialize)]
pub struct MonitoringSettings {
    /// Enable monitoring (default: false, overridden by --enable-monitoring flag)
    #[serde(default)]
    pub enabled: bool,

    /// Sampling interval in milliseconds (default: 1000)
    #[serde(default = "default_sample_interval")]
    pub sample_interval_ms: u64,

    /// Monitor all nodes by default (default: true when monitoring is enabled)
    #[serde(default = "default_true")]
    pub monitor_all_nodes: bool,

    /// Only monitor nodes matching these patterns (empty = all nodes)
    #[serde(default)]
    pub monitor_patterns: Vec<String>,
}

impl Default for MonitoringSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            sample_interval_ms: default_sample_interval(),
            monitor_all_nodes: default_true(),
            monitor_patterns: Vec::new(),
        }
    }
}

fn default_sample_interval() -> u64 {
    2000 // Increased from 1000ms to reduce CPU overhead
}
fn default_true() -> bool {
    true
}

/// Diagnostics monitoring settings
#[derive(Debug, Clone, Deserialize)]
pub struct DiagnosticsSettings {
    /// Enable diagnostics monitoring (default: false, overridden by --enable-diagnostics flag)
    #[serde(default)]
    pub enabled: bool,

    /// Topics to subscribe to for diagnostics
    #[serde(default = "default_diagnostics_topics")]
    pub topics: Vec<String>,

    /// Filter diagnostics by hardware_id (empty = all)
    #[serde(default)]
    pub filter_hardware_ids: Vec<String>,

    /// Debounce period in milliseconds (minimum time between logging the same
    /// diagnostic at the same level). Level transitions ignore it.
    #[serde(default = "default_debounce_ms")]
    pub debounce_ms: u64,

    /// How long a diagnostic may go without an update before it is reported
    /// STALE. `0` disables ageing.
    ///
    /// Needed because a publisher that dies stops publishing rather than
    /// announcing level 3 — without a rule here, its last status reads OK for
    /// the rest of the run. The threshold must exceed the slowest publisher's
    /// period, and real systems mix 100 Hz sensors with 0.1 Hz housekeeping
    /// checks, so the default is deliberately loose. Tighten it per deployment.
    #[serde(default = "default_stale_after_ms")]
    pub stale_after_ms: u64,

    /// Store diagnostics per-node (future feature)
    #[serde(default = "default_true")]
    #[allow(dead_code)]
    pub store_per_node: bool,

    /// Store system-wide diagnostics
    #[serde(default = "default_true")]
    #[allow(dead_code)]
    pub store_system_wide: bool,
}

impl Default for DiagnosticsSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            topics: default_diagnostics_topics(),
            filter_hardware_ids: Vec::new(),
            debounce_ms: default_debounce_ms(),
            stale_after_ms: default_stale_after_ms(),
            store_per_node: default_true(),
            store_system_wide: default_true(),
        }
    }
}

/// `/diagnostics` only.
///
/// `/diagnostics_agg` used to be listed beside it and is a dead subscription on
/// most systems: it only exists if the classic `diagnostic_aggregator` node is
/// deliberately launched, and nothing launches it by default. Autoware, the
/// heaviest diagnostics user, does not ship that package at all — it aggregates
/// with `autoware_diagnostic_graph_aggregator`, which publishes its own message
/// types on `/diagnostics_graph/{struct,status}` rather than a `DiagnosticArray`.
/// `topics` stays configurable for anyone who does run an aggregator.
fn default_diagnostics_topics() -> Vec<String> {
    vec!["/diagnostics".to_string()]
}

fn default_debounce_ms() -> u64 {
    100
}

fn default_stale_after_ms() -> u64 {
    30_000
}

/// RCL interception settings — LD_PRELOAD-based message introspection.
///
/// When enabled, play_launch injects `libplay_launch_interception.so` via
/// LD_PRELOAD into all managed nodes, giving transparent data flow visibility
/// without modifying user code.
#[derive(Debug, Clone, Deserialize)]
pub struct InterceptionSettings {
    /// Enable interception (default: false)
    #[serde(default)]
    pub enabled: bool,

    /// Enable frontier tracking (default: true when interception is enabled)
    #[serde(default = "default_true")]
    pub frontier: bool,

    /// Enable message statistics (default: true when interception is enabled)
    #[serde(default = "default_true")]
    pub stats: bool,

    /// Write a Chrome Trace Event timeline (`interception/trace.json`).
    ///
    /// Off by default: it records every publish/take rather than a summary,
    /// so the file grows with traffic. Load it in `chrome://tracing`.
    #[serde(default)]
    pub trace: bool,

    /// Write the per-message record `play_launch measure` reads
    /// (`interception/events.jsonl`). Default: on.
    ///
    /// Like `trace`, this is one line per publish/take rather than a summary,
    /// so it grows with traffic (~110 bytes a message). Unlike `trace` it
    /// defaults ON, because its whole value is being there *after* a run you
    /// then decide to measure — a flag you had to set beforehand would be
    /// discovered exactly when it is too late. Turn it off for long runs on
    /// high-rate systems.
    #[serde(default = "default_true")]
    pub events: bool,

    /// SPSC ring buffer capacity per child process (default: 65536)
    #[serde(default = "default_ring_capacity")]
    pub ring_capacity: usize,
}

impl Default for InterceptionSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            frontier: default_true(),
            stats: default_true(),
            trace: false,
            events: default_true(),
            ring_capacity: default_ring_capacity(),
        }
    }
}

fn default_ring_capacity() -> usize {
    65536
}

/// Composable node loading settings (phase-52: wired into the container
/// LoadNode path via `container_actor::timing::LoadTimings`).
///
/// Autoware-scale guidance: composable ctors that block for minutes (e.g.
/// TensorRT engine builds on first run) hold a blocking container's
/// executor — raise `load_total_budget_secs` (and consider
/// `load_retry_timeout_millis`) rather than the base timeout; the loader
/// polls ListNodes during the budget instead of resending (LoadNode is
/// not idempotent).
#[derive(Debug, Clone, Deserialize)]
pub struct ComposableNodeLoadingSettings {
    /// Timeout for the FIRST LoadNode service call per composable (milliseconds)
    #[serde(default = "default_load_node_timeout_millis")]
    pub load_node_timeout_millis: u64,

    /// Maximum LoadNode attempts (first + retries) per composable
    #[serde(default = "default_load_node_attempts")]
    pub load_node_attempts: usize,

    /// Maximum concurrent composable node loading operations, counted across
    /// ALL containers.
    ///
    /// Phase 61 made this real. Between its introduction and then it was dead
    /// config: `dispatch_pending_loads` drained its queue and spawned a task
    /// per request with no gate, so a launch with 84 composables across 16
    /// containers put all 84 into LoadNode at once — and under
    /// `--container-mode isolated` each of those forks a process. `0` means
    /// unlimited, i.e. the old behaviour.
    #[serde(default = "default_max_concurrent_load_node_spawn")]
    pub max_concurrent_load_node_spawn: usize,

    /// Timeout for retry LoadNode calls (milliseconds) — longer than the
    /// first call because a busy single-threaded container answers late
    #[serde(default = "default_load_retry_timeout_millis")]
    pub load_retry_timeout_millis: u64,

    /// Total per-composable budget while the container is busy/unresponsive
    /// (seconds). The loader polls ListNodes within this budget instead of
    /// resending LoadNode. Raise for multi-minute ctors (TensorRT).
    #[serde(default = "default_load_total_budget_secs")]
    pub load_total_budget_secs: u64,

    /// Pause between ListNodes verification polls while busy (seconds)
    #[serde(default = "default_load_verify_poll_interval_secs")]
    pub load_verify_poll_interval_secs: u64,

    /// How long a composable may sit in Loading with a known unique_id
    /// before being promoted to Loaded (seconds) — covers DDS event loss
    #[serde(default = "default_loading_event_timeout_secs")]
    pub loading_event_timeout_secs: u64,

    /// Warmup delay after the LoadNode service appears before calling it
    /// (milliseconds) — the executor may register the service before it
    /// starts processing requests
    #[serde(default = "default_post_service_ready_warmup_ms")]
    pub post_service_ready_warmup_ms: u64,

    /// Phase 64: talk to `play_launch_container` over a private socketpair
    /// instead of the `LoadNode` service.
    ///
    /// Only ever applies to OUR container binary (`--container-mode
    /// observable`/`isolated`); a stock container has no such channel and is
    /// byte-for-byte unaffected. `false` restores the pre-phase-64 behaviour
    /// of loading every composable through the ROS service — useful for
    /// comparing the two paths, and the escape hatch if the socket ever
    /// misbehaves.
    #[serde(default = "default_control_socket")]
    pub control_socket: bool,

    /// Phase 64 W2: how long a `load` frame may go unacknowledged before the
    /// supervisor ASKS the container about it (milliseconds).
    ///
    /// Expiry never fails a load; it only causes a `query`. The answer decides
    /// — and `unknown` (nothing was accepted, so nothing was forked) is the
    /// one case where a resend cannot double-load anything.
    #[serde(default = "default_load_ack_timeout_ms")]
    pub ack_timeout_ms: u64,

    /// How long a load may go without any report from the container before the
    /// supervisor asks about it (seconds). Three times the container's 15 s
    /// liveness cadence, so a single dropped report is not an event.
    #[serde(default = "default_report_timeout_secs")]
    pub report_timeout_secs: u64,

    /// How often to re-ask when a `query` itself went unanswered (seconds).
    /// An unanswered query is its own state — neither loaded nor lost — and is
    /// reported rather than resolved by assumption.
    #[serde(default = "default_probe_interval_secs")]
    pub probe_interval_secs: u64,

    /// Total load attempts per composable, including the first.
    ///
    /// The default of 2 allows exactly one resend, and only ever after the
    /// container has CONFIRMED that nothing is running for that load — never
    /// on a timeout. Set to 1 to disable resending entirely.
    #[serde(default = "default_max_load_attempts")]
    pub max_load_attempts: usize,

    /// Declare a constructor stalled after this many seconds. `0` (the
    /// default) means NEVER.
    ///
    /// A fixed number here is a guess about hardware we do not have: a
    /// first-run TensorRT engine build takes ~33 s cold and ~45 s cached on one
    /// Orin, and when the guess is wrong it kills a node that was working. See
    /// `PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS`, which defaults to no deadline
    /// for the same reason.
    #[serde(default)]
    pub stall_after_secs: u64,

    /// CPU usage (percent of one core) below which a constructor counts as
    /// making no progress. Evidence for a stall, never the trigger on its own:
    /// a constructor blocked on a service that has not come up yet burns no
    /// CPU while behaving correctly.
    #[serde(default = "default_stall_cpu_threshold_pct")]
    pub stall_cpu_threshold_pct: f64,

    /// What to do about a stalled constructor. `report` (default) changes
    /// nothing and says so with the evidence; `fail` cancels it; `restart`
    /// cancels it and, once the container confirms nothing is running,
    /// resends.
    #[serde(default)]
    pub stall_action: StallAction,

    /// Whether a composable that CRASHED after loading is reloaded. Off by
    /// default: a component that dies in its constructor and is retried
    /// forever is a worse outcome than one that stays failed and says so.
    #[serde(default)]
    pub composable_respawn: ComposableRespawn,

    /// How long to wait for the container's control-channel `Hello` before
    /// falling back to the `LoadNode` service (milliseconds).
    ///
    /// The container sends it before `rclcpp::init`, so this is normally
    /// answered in milliseconds; the budget exists for the version-skew case
    /// where a container binary from an older wheel inherits the fd and never
    /// speaks. Loading waits on it, so it is deliberately short.
    #[serde(default = "default_control_hello_timeout_ms")]
    pub control_hello_timeout_ms: u64,
}

impl Default for ComposableNodeLoadingSettings {
    fn default() -> Self {
        Self {
            load_node_timeout_millis: default_load_node_timeout_millis(),
            load_node_attempts: default_load_node_attempts(),
            max_concurrent_load_node_spawn: default_max_concurrent_load_node_spawn(),
            load_retry_timeout_millis: default_load_retry_timeout_millis(),
            load_total_budget_secs: default_load_total_budget_secs(),
            load_verify_poll_interval_secs: default_load_verify_poll_interval_secs(),
            loading_event_timeout_secs: default_loading_event_timeout_secs(),
            post_service_ready_warmup_ms: default_post_service_ready_warmup_ms(),
            control_socket: default_control_socket(),
            control_hello_timeout_ms: default_control_hello_timeout_ms(),
            ack_timeout_ms: default_load_ack_timeout_ms(),
            report_timeout_secs: default_report_timeout_secs(),
            probe_interval_secs: default_probe_interval_secs(),
            max_load_attempts: default_max_load_attempts(),
            stall_after_secs: 0,
            stall_cpu_threshold_pct: default_stall_cpu_threshold_pct(),
            stall_action: StallAction::default(),
            composable_respawn: ComposableRespawn::default(),
        }
    }
}

fn default_load_node_timeout_millis() -> u64 {
    30000
}
fn default_load_node_attempts() -> usize {
    3
}
fn default_max_concurrent_load_node_spawn() -> usize {
    10
}
fn default_load_retry_timeout_millis() -> u64 {
    60000
}
fn default_load_total_budget_secs() -> u64 {
    600
}
fn default_load_verify_poll_interval_secs() -> u64 {
    5
}
fn default_loading_event_timeout_secs() -> u64 {
    10
}
fn default_post_service_ready_warmup_ms() -> u64 {
    200
}
/// What to do when a constructor is alive, past its budget, and burning no CPU.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum StallAction {
    /// Say so, with pid, elapsed and CPU evidence. Change nothing.
    #[default]
    Report,
    /// Cancel the load; the container kills the child and confirms.
    Fail,
    /// Cancel, wait for the confirmation, then resend — never the other order,
    /// because a resend before a confirmed teardown is a double load.
    Restart,
}

/// Whether a composable that crashed after loading is reloaded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "kebab-case")]
pub enum ComposableRespawn {
    /// Leave it failed and reported (the pre-phase-64 behaviour).
    #[default]
    Off,
    /// Reload it, bounded by the container's `max_respawn_attempts`.
    OnCrash,
}

fn default_control_socket() -> bool {
    true
}
fn default_load_ack_timeout_ms() -> u64 {
    5000
}
fn default_report_timeout_secs() -> u64 {
    45
}
fn default_probe_interval_secs() -> u64 {
    15
}
fn default_max_load_attempts() -> usize {
    2
}
fn default_stall_cpu_threshold_pct() -> f64 {
    1.0
}
fn default_control_hello_timeout_ms() -> u64 {
    10000
}

/// Container readiness checking settings
#[derive(Debug, Clone, Deserialize)]
pub struct ContainerReadinessSettings {
    /// Wait for container services to be available via ROS service discovery
    /// Default: true (changed from false - service readiness is now default behavior)
    #[serde(default = "default_wait_for_service_ready")]
    pub wait_for_service_ready: bool,

    /// Maximum time to wait for each container service (seconds)
    /// Set to 0 for unlimited wait time
    #[serde(default = "default_service_ready_timeout_secs")]
    pub service_ready_timeout_secs: u64,

    /// Interval for polling container service availability (milliseconds)
    #[serde(default = "default_service_poll_interval_ms")]
    #[allow(dead_code)]
    pub service_poll_interval_ms: u64,
}

impl Default for ContainerReadinessSettings {
    fn default() -> Self {
        Self {
            wait_for_service_ready: default_wait_for_service_ready(),
            service_ready_timeout_secs: default_service_ready_timeout_secs(),
            service_poll_interval_ms: default_service_poll_interval_ms(),
        }
    }
}

fn default_wait_for_service_ready() -> bool {
    true // Changed from false - service readiness is now the default
}
fn default_service_ready_timeout_secs() -> u64 {
    120
}
fn default_service_poll_interval_ms() -> u64 {
    500
}

/// Configuration for individual process control
#[derive(Debug, Clone, Deserialize)]
pub struct ProcessConfig {
    /// Node name or pattern (supports glob patterns)
    #[allow(dead_code)] // Used in spawn_nodes integration (not yet connected)
    pub node_pattern: String,

    /// Enable monitoring for this node (default: inherit from global)
    #[serde(default)]
    #[allow(dead_code)] // Used in spawn_nodes integration (not yet connected)
    pub monitor: Option<bool>,

    /// CPU cores to pin this process to (e.g., [0, 1, 2])
    #[serde(default)]
    pub cpu_affinity: Vec<usize>,

    /// Nice value (-20 to 19, lower = higher priority)
    #[serde(default)]
    pub nice: Option<i32>,
}

#[allow(dead_code)] // Methods used in spawn_nodes integration (not yet connected)
impl ProcessConfig {
    /// Check if this config matches a node name
    pub fn matches(&self, node_name: &str) -> bool {
        Pattern::new(&self.node_pattern)
            .map(|pattern| pattern.matches(node_name))
            .unwrap_or(false)
    }

    /// Check if monitoring is enabled for this node
    pub fn should_monitor(&self, global_enabled: bool) -> bool {
        self.monitor.unwrap_or(global_enabled)
    }
}

/// Resolved runtime configuration ready for use
#[derive(Debug, Clone)]
pub struct ResolvedRuntimeConfig {
    pub monitoring: ResolvedMonitoringConfig,
    pub composable_node_loading: ComposableNodeLoadingSettings,
    pub container_readiness: ContainerReadinessSettings,
    pub diagnostics: DiagnosticsSettings,
    pub interception: InterceptionSettings,
    /// Phase 61: startup admission control, carried through unchanged — its
    /// machine-dependent resolution happens at `StartupSettings::resolve`,
    /// where the core count and RAM are read, not here.
    pub startup: StartupSettings,
    /// Phase 66 W2 — per-container cgroup limits.
    pub cgroups: CgroupSettings,
}

/// Resolved monitoring configuration
#[derive(Debug, Clone)]
pub struct ResolvedMonitoringConfig {
    pub enabled: bool,
    pub sample_interval_ms: u64,
    #[allow(dead_code)] // Used in spawn_nodes integration (not yet connected)
    pub monitor_all_nodes: bool,
    #[allow(dead_code)] // Used in spawn_nodes integration (not yet connected)
    pub monitor_patterns: Vec<String>,
    #[allow(dead_code)] // Used in spawn_nodes integration (not yet connected)
    pub process_configs: Vec<ProcessConfig>,
}

#[allow(dead_code)] // Methods used in spawn_nodes integration (not yet connected)
impl ResolvedMonitoringConfig {
    /// Check if a node should be monitored
    pub fn should_monitor(&self, node_name: &str) -> bool {
        if !self.enabled {
            return false;
        }

        // Check process-specific config first
        for config in &self.process_configs {
            if config.matches(node_name)
                && let Some(monitor) = config.monitor
            {
                return monitor;
            }
        }

        // Check monitor_patterns if specified
        if !self.monitor_patterns.is_empty() {
            return self.monitor_patterns.iter().any(|pattern| {
                Pattern::new(pattern)
                    .map(|p| p.matches(node_name))
                    .unwrap_or(false)
            });
        }

        // Default: monitor all nodes if enabled
        self.monitor_all_nodes
    }

    /// Get process config for a node
    pub fn get_process_config(&self, node_name: &str) -> Option<&ProcessConfig> {
        self.process_configs
            .iter()
            .find(|config| config.matches(node_name))
    }
}

/// Load and resolve runtime configuration
pub fn load_runtime_config(
    config_path: Option<&Path>,
    monitoring_enabled: bool,
    monitor_interval_override: Option<u64>,
    diagnostics_enabled: bool,
) -> Result<ResolvedRuntimeConfig> {
    // Load config file or use defaults
    let mut config = if let Some(path) = config_path {
        let content = std::fs::read_to_string(path)
            .wrap_err_with(|| format!("Failed to read config file: {}", path.display()))?;
        serde_yaml_ng::from_str::<RuntimeConfig>(&content)
            .wrap_err_with(|| format!("Failed to parse config file: {}", path.display()))?
    } else {
        RuntimeConfig::default()
    };

    // Enable monitoring by default (enabled unless explicitly disabled via CLI)
    // If config file doesn't specify, use CLI decision
    if config_path.is_none() || !config.monitoring.enabled {
        config.monitoring.enabled = monitoring_enabled;
    }

    // Enable diagnostics by default (enabled unless explicitly disabled via CLI)
    // If config file doesn't specify, use CLI decision
    if config_path.is_none() || !config.diagnostics.enabled {
        config.diagnostics.enabled = diagnostics_enabled;
    }

    // CLI interval overrides config file
    if let Some(interval) = monitor_interval_override {
        config.monitoring.sample_interval_ms = interval;
    }

    // Validate process configs
    for process_config in &config.processes {
        // Validate CPU affinity
        if !process_config.cpu_affinity.is_empty() {
            let num_cpus = num_cpus::get();
            for &cpu in &process_config.cpu_affinity {
                if cpu >= num_cpus {
                    return Err(eyre::eyre!(
                        "Invalid CPU affinity: CPU {} does not exist (max: {})",
                        cpu,
                        num_cpus - 1
                    ));
                }
            }
        }

        // Validate nice value
        if let Some(nice) = process_config.nice
            && !(-20..=19).contains(&nice)
        {
            return Err(eyre::eyre!(
                "Invalid nice value: {} (must be between -20 and 19)",
                nice
            ));
        }
    }

    Ok(ResolvedRuntimeConfig {
        monitoring: ResolvedMonitoringConfig {
            enabled: config.monitoring.enabled,
            sample_interval_ms: config.monitoring.sample_interval_ms,
            monitor_all_nodes: config.monitoring.monitor_all_nodes,
            monitor_patterns: config.monitoring.monitor_patterns,
            process_configs: config.processes,
        },
        composable_node_loading: config.composable_node_loading,
        container_readiness: config.container_readiness,
        diagnostics: config.diagnostics,
        interception: config.interception,
        startup: config.startup,
        cgroups: config.cgroups,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = RuntimeConfig::default();
        assert!(!config.monitoring.enabled);
        assert_eq!(config.monitoring.sample_interval_ms, 2000);
        assert!(config.monitoring.monitor_all_nodes);
        assert!(config.processes.is_empty());

        // Test composable node loading defaults
        assert_eq!(
            config.composable_node_loading.load_node_timeout_millis,
            30000
        );
        assert_eq!(config.composable_node_loading.load_node_attempts, 3);
        assert_eq!(
            config
                .composable_node_loading
                .max_concurrent_load_node_spawn,
            10
        );

        // Test container readiness defaults
        assert!(config.container_readiness.wait_for_service_ready); // Now defaults to true
        assert_eq!(config.container_readiness.service_ready_timeout_secs, 120);
        assert_eq!(config.container_readiness.service_poll_interval_ms, 500);

        // Test interception defaults
        assert!(!config.interception.enabled);
        assert!(config.interception.frontier);
        assert!(config.interception.stats);
        assert_eq!(config.interception.ring_capacity, 65536);
    }

    #[test]
    fn test_pattern_matching() {
        let config = ProcessConfig {
            node_pattern: "/planning/*".to_string(),
            monitor: None,
            cpu_affinity: vec![],
            nice: None,
        };

        assert!(config.matches("/planning/behavior_planner"));
        assert!(config.matches("/planning/motion_planner"));
        assert!(!config.matches("/control/controller"));
    }

    #[test]
    fn test_should_monitor_logic() {
        let config = ResolvedMonitoringConfig {
            enabled: true,
            sample_interval_ms: 1000,
            monitor_all_nodes: true,
            monitor_patterns: vec![],
            process_configs: vec![],
        };

        assert!(config.should_monitor("/any/node"));

        // Disabled monitoring
        let config_disabled = ResolvedMonitoringConfig {
            enabled: false,
            ..config.clone()
        };
        assert!(!config_disabled.should_monitor("/any/node"));
    }

    #[test]
    fn test_should_monitor_with_patterns() {
        let config = ResolvedMonitoringConfig {
            enabled: true,
            sample_interval_ms: 1000,
            monitor_all_nodes: false,
            monitor_patterns: vec!["/planning/*".to_string(), "/control/*".to_string()],
            process_configs: vec![],
        };

        assert!(config.should_monitor("/planning/behavior"));
        assert!(config.should_monitor("/control/controller"));
        assert!(!config.should_monitor("/perception/detector"));
    }

    #[test]
    fn test_process_specific_override() {
        let config = ResolvedMonitoringConfig {
            enabled: true,
            sample_interval_ms: 1000,
            monitor_all_nodes: true,
            monitor_patterns: vec![],
            process_configs: vec![ProcessConfig {
                node_pattern: "/rviz*".to_string(),
                monitor: Some(false),
                cpu_affinity: vec![],
                nice: None,
            }],
        };

        assert!(config.should_monitor("/planning/behavior"));
        assert!(!config.should_monitor("/rviz2"));
    }

    #[test]
    fn test_nice_value_validation() {
        // Test that default config loads successfully
        let result = load_runtime_config(None, false, None, false);
        assert!(result.is_ok());

        // Test that config validates nice value ranges
        // (This would require a temp YAML file to fully test validation)
        let _config = ProcessConfig {
            node_pattern: "test".to_string(),
            monitor: None,
            cpu_affinity: vec![],
            nice: Some(10), // Valid nice value
        };
    }
}
