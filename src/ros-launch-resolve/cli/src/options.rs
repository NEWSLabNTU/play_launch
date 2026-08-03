use clap::{Args, Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

use ros_launch_resolve::config::SchedApplyMode;

/// Parser backend selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum ParserBackend {
    /// Use Rust parser (default, no fallback)
    Rust,
    /// Use Python parser
    Python,
}

/// The library's parser selector carries no clap derive on purpose — it must
/// not know a CLI exists. Each CLI keeps its own clap enum and maps into it.
impl From<ParserBackend> for ros_launch_resolve::verbs::ParserBackend {
    fn from(backend: ParserBackend) -> Self {
        match backend {
            ParserBackend::Rust => Self::Rust,
            ParserBackend::Python => Self::Python,
        }
    }
}

/// Container mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum ContainerMode {
    /// Use play_launch_container with ComponentEvent publishing
    Observable,
    /// Use play_launch_container with clone(CLONE_VM) per-node isolation (default)
    Isolated,
    /// Use the original container from the launch file (no override)
    Stock,
}

/// Features that can be selectively enabled
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum Feature {
    /// Resource monitoring (CPU, memory, I/O)
    Monitoring,
    /// Diagnostic monitoring (/diagnostics topic)
    Diagnostics,
    /// Web UI for node management
    WebUi,
}

// HISTORY (deliberately a `//` comment, not a doc comment: clap turns the
// struct's doc comment into `--help`'s `long_about`, and a code-review
// narrative is not what a user opens `--help` to read). The name, the about
// line and the examples below were inherited verbatim from `play_launch` when
// RFC-0060 W3 extracted this binary, so `--help` introduced itself as
// `play_launch`, described itself as a replay tool, and advertised `launch`,
// `run` and `replay` — three verbs this binary has never had and which issue
// 0013 later deleted the leftover argument structs for.
#[derive(Parser)]
#[command(name = "ros-launch-resolve")]
#[command(version)]
#[command(about = "Resolve ROS 2 launch trees into a checked SystemModel")]
#[command(
    long_about = "Resolve ROS 2 launch trees into a checked SystemModel.\n\n\
    Expands a launch file, applies manifest contracts and the scheduling \
    platform file, and emits `system_model.yaml` — the one artifact \
    `play_launch up` spawns from.\n\n\
    Needs NO ROS installation: the launch frontends and the checker are \
    pure-Rust plus embedded CPython, so this runs in a build container, in \
    CI, or on a developer machine that has never sourced a ROS setup file. \
    Running, replaying and supervising nodes stay in `play_launch`, which \
    does need ROS."
)]
#[command(after_help = "Examples:\n  \
    ros-launch-resolve resolve autoware_launch planning_simulator.launch.xml -o autoware.yaml\n  \
    ros-launch-resolve check my_bringup system.launch.xml\n  \
    ros-launch-resolve check my_bringup system.launch.xml --format json --explain\n  \
    ros-launch-resolve dump --output autoware.yaml launch autoware_launch planning_simulator.launch.xml\n  \
    ros-launch-resolve contract eject my_bringup system.launch.xml\n  \
    ros-launch-resolve plot play_log/latest")]
#[command(arg_required_else_help = true)]
pub struct Options {
    #[command(subcommand)]
    pub command: Command,
}

#[derive(Subcommand)]
pub enum Command {
    /// Resolve a launch tree into a checked SystemModel (RFC-0050).
    #[command(after_help = "Examples:\n  \
        ros-launch-resolve resolve my_bringup system.launch.xml -o system_model.yaml\n  \
        ros-launch-resolve resolve launch/system.launch.xml --system system.toml")]
    Resolve(ResolveArgs),

    /// Dump launch expansion without resolving contracts or scheduling.
    Dump(DumpArgs),

    /// Check manifest contracts against a launch file. No ROS install
    /// required — the checker is a layer-2 crate.
    #[command(after_help = "Examples:\n  \
        ros-launch-resolve check my_bringup system.launch.xml\n  \
        ros-launch-resolve check my_bringup system.launch.xml --format json\n  \
        ros-launch-resolve check --contracts ~/contracts launch/system.launch.xml mode:=lidar")]
    Check(CheckArgs),

    /// Eject a contract sidecar for a package.
    Contract(ContractArgs),

    /// Plot resource usage from execution logs.
    Plot(PlotArgs),
}

/// Arguments for `ros-launch-resolve contract`
#[derive(Args)]
pub struct ContractArgs {
    #[command(subcommand)]
    pub subcommand: ContractSubcommand,
}

#[derive(Subcommand)]
pub enum ContractSubcommand {
    /// Copy the resolved provider contract (and target's platform file, if
    /// any) into the overlay tree, ready to edit — editing never touches
    /// `/opt` (design §3.3).
    Eject(ContractEjectArgs),
}

/// Arguments for `ros-launch-resolve contract eject`
#[derive(Args)]
pub struct ContractEjectArgs {
    /// Package name or path to launch file
    pub package_or_path: String,

    /// Launch file name (if package_or_path is a package name)
    pub launch_file: Option<String>,

    /// Which scheduling target's platform file to eject (`<stem>.system.<target>.yaml`).
    #[arg(long, default_value = "posix")]
    pub target: String,

    /// Overlay root to eject into. Defaults to the discovered overlay root
    /// (same discovery as `check --contracts`: `$PLAY_LAUNCH_CONTRACTS`,
    /// then `$XDG_CONFIG_HOME/play_launch/contracts`, then
    /// `/etc/play_launch/contracts`) — errors if none of those exist yet
    /// and `--into` wasn't given.
    #[arg(long, value_name = "PATH")]
    pub into: Option<PathBuf>,

    /// Overwrite existing overlay files. Without this flag, `eject` refuses
    /// to touch a destination that already exists.
    #[arg(long)]
    pub force: bool,
}

/// Arguments for `ros-launch-resolve resolve`
#[derive(Args)]
pub struct ResolveArgs {
    /// Package name or path to launch file
    pub package_or_path: String,

    /// Launch file name (if package_or_path is a package name)
    pub launch_file: Option<String>,

    /// Launch arguments in KEY:=VALUE format. Flags may be placed before or
    /// after these (clap parses flags in any position); use `--` to force
    /// remaining tokens to be treated as positional launch arguments.
    pub launch_arguments: Vec<String>,

    /// nano-ros issue 0320 — the bringup package root that `meta.inputs[].path`
    /// are recorded relative to. When omitted, falls back to the launch file's
    /// grandparent (`<bringup>/launch/<f>.launch.xml`). Pass it to make model
    /// portability structural rather than inferred from the launch-path layout.
    #[arg(long, value_name = "PATH")]
    pub bringup_root: Option<PathBuf>,

    /// Overlay root for user-supplied contracts (see `check --contracts`).
    #[arg(long, value_name = "PATH")]
    pub contracts: Option<PathBuf>,

    /// Disable the provider-sidecar channel (see `check`).
    #[arg(long)]
    pub no_provider_contracts: bool,

    /// Path to a scheduling platform file (v2 `.yaml` or legacy `.toml`).
    #[arg(long)]
    pub sched: Option<std::path::PathBuf>,

    /// R1-P1 — the integrator `system.toml` (deploy placement, transports,
    /// bridges, capability features, domain/locator/rmw ladder). Fills the
    /// model's execution layer; consumers never parse system.toml
    /// themselves (canonical-path decision).
    #[arg(long, value_name = "system.toml")]
    pub system: Option<std::path::PathBuf>,

    /// Scheduling target the platform file must declare.
    #[arg(long, default_value = "posix")]
    pub target: String,

    /// Parser backend to use for launch file parsing (Phase 46.4).
    /// - rust: Use Rust parser (default, fast)
    /// - python: Use Python parser (maximum compatibility)
    ///
    /// Both produce the full model — the contract/sched layers apply on the
    /// shared scope table regardless of parser (Phase 40.1), so they're
    /// populated whenever a contract sidecar / platform file (or
    /// --contracts/--sched) resolves, and empty only when none does.
    #[arg(long, value_enum, default_value = "rust")]
    pub parser: ParserBackend,

    /// Output path for the SystemModel YAML. `-` writes to stdout.
    #[arg(long, short = 'o', default_value = "system_model.yaml")]
    pub out: String,

    /// Print the merged scheduling plan with provenance per node (Phase
    /// 45.6), rendered from the SystemModel this invocation just built —
    /// same table `check --sched --explain` shows for the same inputs, one
    /// renderer. Only meaningful together with a resolved scheduling
    /// platform file; a no-op note is printed otherwise (not an error).
    #[arg(long)]
    pub explain: bool,
}

/// Arguments for launching a launch file
#[derive(Args)]
pub struct LaunchArgs {
    /// Package name or path to launch file
    pub package_or_path: String,

    /// Launch file name (if package_or_path is a package name)
    pub launch_file: Option<String>,

    /// Launch arguments in KEY:=VALUE format. Flags may be placed before or
    /// after these (clap parses flags in any position); use `--` to force
    /// remaining tokens to be treated as positional launch arguments.
    pub launch_arguments: Vec<String>,

    /// Parser backend to use for launch file parsing.
    /// - rust: Use Rust parser (default, fast, no fallback on error)
    /// - python: Use Python parser (slower, maximum compatibility)
    #[arg(long, value_enum, default_value = "rust")]
    pub parser: ParserBackend,

    /// Block $(command ...) substitutions in launch files.
    /// When set, the parser rejects any $(command) with an error.
    #[arg(long)]
    pub block_commands: bool,

    #[command(flatten)]
    pub common: CommonOptions,
}

/// Arguments for dump command
#[derive(Args)]
pub struct DumpArgs {
    #[command(subcommand)]
    pub subcommand: DumpSubcommand,

    /// Output file for the dump. Defaults to `system_model.yaml`. May be
    /// given before or after the `launch` subcommand and its launch
    /// arguments.
    #[arg(long, short = 'o', global = true)]
    pub output: Option<PathBuf>,

    /// Enable debug output during dump. May be given before or after the
    /// subcommand.
    #[arg(long, global = true)]
    pub debug: bool,
}

#[derive(Subcommand)]
pub enum DumpSubcommand {
    /// Dump a launch file execution — emits the SystemModel
    /// (`system_model.yaml`), the one dump artifact (Phase 47.B2:
    /// `record.json`/`--format` retired; `dump` always emits the model).
    Launch(LaunchArgs),
}

/// Common options shared across all commands (phase-52.1: grouped into
/// `#[command(flatten)]` concern structs — flag names are unchanged).
#[derive(Args, Clone)]
pub struct CommonOptions {
    /// Log directory for execution outputs
    #[arg(long, default_value = "play_log")]
    pub log_dir: PathBuf,

    /// Runtime configuration file (YAML).
    /// Contains composable node loading, container readiness, monitoring settings, and process control.
    /// Service readiness checking is enabled by default - disable in config if needed.
    #[arg(long, short = 'c', value_name = "PATH")]
    pub config: Option<PathBuf>,

    /// Enable verbose output (INFO level logging).
    /// Without this flag, only warnings and errors are shown.
    /// Use RUST_LOG env var for debug-level logging.
    #[arg(long, short = 'v')]
    pub verbose: bool,

    #[command(flatten)]
    pub features: FeatureOptions,

    #[command(flatten)]
    pub containers: ContainerOptions,

    #[command(flatten)]
    pub web: WebOptions,

    #[command(flatten)]
    pub contract_opts: ContractOptions,

    #[command(flatten)]
    pub sched_opts: SchedOptions,
}

/// Feature toggles (monitoring / diagnostics / web UI)
#[derive(Args, Clone, Default)]
pub struct FeatureOptions {
    /// Enable only specific features. Can be specified multiple times.
    /// When used, only the specified features are enabled (others are disabled).
    /// Available features: monitoring, diagnostics, web-ui
    #[arg(long, value_enum, value_name = "FEATURE")]
    pub enable: Vec<Feature>,

    /// Disable resource monitoring (enabled by default).
    #[arg(long, conflicts_with = "enable")]
    pub disable_monitoring: bool,

    /// Disable diagnostic monitoring (enabled by default).
    #[arg(long, conflicts_with = "enable")]
    pub disable_diagnostics: bool,

    /// Disable web UI (enabled by default).
    #[arg(long, conflicts_with = "enable")]
    pub disable_web_ui: bool,

    /// Disable all features (monitoring, diagnostics, and web UI).
    #[arg(long, conflicts_with = "enable")]
    pub disable_all: bool,

    /// Resource sampling interval in milliseconds (overrides config file).
    #[arg(long, value_name = "MS")]
    pub monitor_interval_ms: Option<u64>,
}

/// Container / composable-node behavior
#[derive(Args, Clone)]
pub struct ContainerOptions {
    /// Run composable nodes in standalone mode instead of loading into containers
    #[arg(long)]
    pub standalone_composable_nodes: bool,

    /// Load composable nodes that have no matching container
    #[arg(long)]
    pub load_orphan_composable_nodes: bool,

    /// Disable automatic respawn even if configured in launch file
    #[arg(long)]
    pub disable_respawn: bool,

    /// Container mode: which container binary to use for composable nodes.
    /// - observable: use play_launch_container with ComponentEvent publishing
    /// - isolated: use play_launch_container with clone(CLONE_VM) per-node isolation (default)
    /// - stock: use the original container from the launch file (no override)
    #[arg(long, value_enum, default_value = "isolated")]
    pub container_mode: ContainerMode,

    /// Phase 52: per-composable total LoadNode budget in SECONDS while a
    /// container is busy (overrides config
    /// `composable_node_loading.load_total_budget_secs`, default 600).
    /// Raise for containers whose composable ctors block for minutes
    /// (e.g. TensorRT engine builds).
    #[arg(long, value_name = "SECS")]
    pub load_total_budget: Option<u64>,

    /// Phase 52: timeout in SECONDS for the first LoadNode call per
    /// composable (overrides config
    /// `composable_node_loading.load_node_timeout_millis`, default 30s).
    #[arg(long, value_name = "SECS")]
    pub load_node_timeout: Option<u64>,

    /// Phase 52: what to do when startup completes with failed members.
    /// `continue`: keep running (web UI shows the failures) — the
    /// pre-phase-52 behavior. `exit`: shut everything down and exit
    /// non-zero, naming the failed members (CI mode).
    #[arg(long, value_enum, default_value = "continue")]
    pub on_startup_failure: OnStartupFailure,
}

/// Web UI options
#[derive(Args, Clone)]
pub struct WebOptions {
    /// Web UI address in IP:PORT format (default: 127.0.0.1:8080).
    /// Use 0.0.0.0:8080 to expose to network (insecure, use with caution).
    #[arg(long, value_name = "IP:PORT", default_value = "127.0.0.1:8080")]
    pub web_addr: String,
}

/// Contract / runtime-enforcement options
#[derive(Args, Clone)]
pub struct ContractOptions {
    /// Overlay root for user-supplied contracts, mirroring the launch tree:
    /// <dir>/<pkg>/launch/<stem>.contract.yaml. Checked before provider
    /// sidecars.
    #[arg(long, value_name = "PATH")]
    pub contracts: Option<PathBuf>,

    /// Disable the provider-sidecar channel for BOTH contracts
    /// (<stem>.contract.yaml) and scheduling platform files
    /// (<stem>.system.<target>.yaml) shipped next to the launch file.
    /// On by default.
    #[arg(long)]
    pub no_provider_contracts: bool,

    /// Runtime enforcement mode for manifest contracts (Phase 36.3).
    /// Contracts come from any channel (overlay/provider); with no
    /// contracts resolved the engine has nothing to enforce. Off: no runtime
    /// checks. Warn: log violations. Strict: exit non-zero on first
    /// violation. RecordOnly: collect events without evaluating rules.
    #[arg(long, value_enum, default_value = "warn")]
    pub enforce_rules: EnforceMode,

    /// Phase 36.7: block unauthorized publisher/subscription creation
    /// at the rcl layer. The set of allowed topic FQNs is written from the
    /// merged ManifestIndex (any contract channel) and passed to every child
    /// via env var. Hooked rcl init calls for topics not in the set return
    /// `RCL_RET_TOPIC_INVALID` (1004) — the publisher/subscription is never
    /// created. If NO contract declares any topic, blocking is disabled with
    /// a warning (an empty allowlist would block every endpoint). Off by
    /// default because nodes that don't handle init failure may crash.
    #[arg(long, default_value_t = false)]
    pub block_unauthorized_endpoints: bool,
}

/// Scheduling (Phase 38) options
#[derive(Args, Clone)]
pub struct SchedOptions {
    /// Phase 38: path to a scheduling platform file — v2 `.yaml` schema
    /// (mapper + resources + overrides) or legacy `.toml` (dispatched by
    /// extension; Phase 41.2). When set, replay derives + validates a plan
    /// for `--target` and (per `--sched-apply`) applies SCHED_FIFO/RR +
    /// priority + CPU affinity to each spawned node/container process. Same
    /// file `ros-launch-resolve check --sched` validates.
    #[arg(long, value_name = "PATH")]
    pub sched: Option<PathBuf>,

    /// Which scheduling target the platform file must declare (`target:` in
    /// the v2 schema; legacy `.toml` always implies `posix`). Only `posix`
    /// (Linux RT) is ever applied by `play_launch` itself.
    #[arg(long, default_value = "posix")]
    pub target: String,

    /// Phase 38: how to apply the scheduling spec. `off` = resolve + report
    /// only (no syscalls). `warn` = apply, log a warning and continue on
    /// failure. `strict` = abort the run on any capability/apply failure.
    /// Only meaningful with `--sched`.
    #[arg(long, value_enum, default_value = "warn")]
    pub sched_apply: SchedApplyMode,
}

/// Policy for a startup that completes with failed members (phase-52.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum OnStartupFailure {
    /// Keep running; failures stay visible in the web UI and logs.
    Continue,
    /// Initiate shutdown and exit non-zero, naming the failed members.
    Exit,
}

/// Runtime enforcement mode for manifest contracts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum EnforceMode {
    /// Skip runtime checks entirely.
    Off,
    /// Log violations to `play_log/<ts>/runtime_violations.jsonl`. Never exit early.
    Warn,
    /// First violation triggers shutdown and non-zero exit (CI mode).
    Strict,
    /// Collect events without evaluating rules. For offline analysis.
    RecordOnly,
}

impl Default for CommonOptions {
    fn default() -> Self {
        Self {
            log_dir: PathBuf::from("play_log"),
            config: None,
            verbose: false,
            features: FeatureOptions::default(),
            containers: ContainerOptions::default(),
            web: WebOptions::default(),
            contract_opts: ContractOptions::default(),
            sched_opts: SchedOptions::default(),
        }
    }
}

impl Default for ContainerOptions {
    fn default() -> Self {
        Self {
            standalone_composable_nodes: false,
            load_orphan_composable_nodes: false,
            disable_respawn: false,
            container_mode: ContainerMode::Isolated,
            load_total_budget: None,
            load_node_timeout: None,
            on_startup_failure: OnStartupFailure::Continue,
        }
    }
}

impl Default for WebOptions {
    fn default() -> Self {
        Self {
            web_addr: "127.0.0.1:8080".to_string(),
        }
    }
}

impl Default for ContractOptions {
    fn default() -> Self {
        Self {
            contracts: None,
            no_provider_contracts: false,
            enforce_rules: EnforceMode::Warn,
            block_unauthorized_endpoints: false,
        }
    }
}

impl Default for SchedOptions {
    fn default() -> Self {
        Self {
            sched: None,
            target: "posix".to_string(),
            sched_apply: SchedApplyMode::Warn,
        }
    }
}

/// Arguments for plot command
#[derive(Args)]
pub struct PlotArgs {
    /// Specific log directory to plot (e.g., play_log/2025-10-28_16-17-56)
    #[arg(long, value_name = "PATH")]
    pub log_dir: Option<PathBuf>,

    /// Base log directory to search for latest execution
    #[arg(long, value_name = "PATH", default_value = "./play_log")]
    pub base_log_dir: PathBuf,

    /// Output directory for generated plots
    #[arg(long, short = 'o', value_name = "PATH")]
    pub output_dir: Option<PathBuf>,

    /// Metrics to plot (can be specified multiple times)
    #[arg(long, short = 'm', value_name = "METRIC")]
    pub metrics: Vec<String>,

    /// List available metrics and exit
    #[arg(long)]
    pub list_metrics: bool,
}

/// Arguments for `ros-launch-resolve check`.
///
/// Deleted as dead by issue 0013 when the verb still lived in play_launch;
/// restored verbatim from `a996e97^` now that the verb lives here. The 2026
/// CLI verb reshape moved the diagnostics to layer 2 because the checker
/// (`ros-launch-manifest-check`) is a layer-2 crate and needs no ROS install.
#[derive(Args)]
pub struct CheckArgs {
    /// Package name or path to launch file
    pub package_or_path: String,

    /// Launch file name (if package_or_path is a package name)
    pub launch_file: Option<String>,

    /// Launch arguments in KEY:=VALUE format. Flags may be placed before or
    /// after these (clap parses flags in any position); use `--` to force
    /// remaining tokens to be treated as positional launch arguments.
    pub launch_arguments: Vec<String>,

    /// Overlay root for user-supplied contracts, mirroring the launch tree:
    /// <dir>/<pkg>/launch/<stem>.contract.yaml. Checked before provider
    /// sidecars.
    #[arg(long, value_name = "PATH")]
    pub contracts: Option<PathBuf>,

    /// Disable the provider-sidecar channel for BOTH contracts
    /// (<stem>.contract.yaml) and scheduling platform files
    /// (<stem>.system.<target>.yaml) shipped next to the launch file.
    /// On by default.
    #[arg(long)]
    pub no_provider_contracts: bool,

    /// Path to a scheduling platform file — v2 `.yaml` schema (mapper +
    /// resources + overrides) or legacy `.toml` (dispatched by extension).
    /// When given, `check` also derives + validates a plan for `--target`.
    #[arg(long)]
    pub sched: Option<std::path::PathBuf>,

    /// Which scheduling target the platform file must declare (`target:` in
    /// the v2 schema; legacy `.toml` always implies `posix`). `posix` is
    /// Linux RT (the only target `play_launch` itself applies); RTOS targets
    /// (`zephyr`, `freertos`, ...) are for nano-ros's own consumption of the
    /// same file format.
    #[arg(long, default_value = "posix")]
    pub target: String,

    /// Output format: terminal (default, with source excerpts) or json
    #[arg(long, default_value = "terminal")]
    pub format: String,

    /// Show only diagnostics from these rules. Repeat to allow multiple.
    /// Example: --rule satisfiability --rule consistency
    #[arg(long, value_name = "RULE_ID")]
    pub rule: Vec<String>,

    /// Print the merged scheduling plan with provenance per node (design
    /// §7): which platform-file override, mapper-derived fact, or default
    /// placement produced each node's final class/priority/core, plus the
    /// platform-file and per-scope contract paths that fed the pipeline.
    /// Only meaningful together with a resolved scheduling platform file
    /// (`--sched`, or one resolved via the overlay/provider channels) — a
    /// no-op note is printed otherwise (not an error).
    #[arg(long)]
    pub explain: bool,

    /// Export the DECLARED causal graph (nodes, topics, pub/sub edges,
    /// node/scope paths, cycle catalogue) to `<path>` instead of — in
    /// addition to — the normal validation output. Format is picked by
    /// extension: `.json` (default, for tooling) or `.dot` (Graphviz, for
    /// human inspection via `dot -Tsvg`). Extension-less paths are written
    /// as JSON. This is an export, not a validation step — existing rules
    /// and exit codes are unaffected (Phase 42.1).
    #[arg(long, value_name = "PATH")]
    pub export_graph: Option<PathBuf>,
}

// `CheckArgs::contract_sources` moved to `verbs::check::CheckInputs` with the
// verb body: the overlay/provider two-step is checker policy, not CLI parsing,
// and both CLIs must resolve contract channels identically.

// NOTE: the argument-parsing tests for `launch`, `run` and `replay` moved to
// play_launch with those verbs — this crate no longer defines them. `check`
// moved back here (2026 CLI verb reshape, task 1) along with its test.
// Tests for the three remaining launch-tree verbs belong here and are
// phase-312 W1.6 follow-up.

#[cfg(test)]
mod tests {
    use super::*;
    use clap::{CommandFactory, Parser};

    fn parse(args: &[&str]) -> Result<Options, clap::Error> {
        let mut argv = vec!["ros-launch-resolve"];
        argv.extend_from_slice(args);
        Options::try_parse_from(argv)
    }

    // `--output` and `--debug` are `global = true` on `DumpArgs`, so clap
    // accepts them on either side of the `launch` subcommand and its
    // positional launch arguments. These two cases came over from
    // play_launch's `cli::options` when RFC-0060 W3 moved `dump` here but
    // left the tests behind; they kept referring to a `Command::Dump` variant
    // that no longer existed there, so play_launch failed to compile under
    // `cargo check --all-targets` from adc33a7 until 2026-08-02.

    #[test]
    fn dump_launch_flag_after_two_launch_arguments() {
        // The Autoware shape: multiple KEY:=VALUE args, then a flag, all
        // after `dump launch`.
        let opts = parse(&[
            "dump",
            "launch",
            "pkg",
            "file.launch.xml",
            "vehicle_model:=sample_vehicle",
            "sensor_model:=sample_sensor_kit",
            "--output",
            "/tmp/aw.yaml",
        ])
        .expect("--output after KEY:=VALUE launch args must parse (dump's global flags)");
        let Command::Dump(dump_args) = opts.command else {
            panic!("expected Dump");
        };
        assert_eq!(dump_args.output, Some(PathBuf::from("/tmp/aw.yaml")));
        // Phase 47.B2 retired `dump run`: `DumpSubcommand` has exactly one
        // variant, so this destructure is irrefutable — no `else` arm.
        let DumpSubcommand::Launch(args) = dump_args.subcommand;
        assert_eq!(
            args.launch_arguments,
            vec![
                "vehicle_model:=sample_vehicle".to_string(),
                "sensor_model:=sample_sensor_kit".to_string(),
            ]
        );
    }

    #[test]
    fn dump_launch_flag_before_subcommand_still_works() {
        let opts = parse(&[
            "dump",
            "--output",
            "/tmp/aw.yaml",
            "launch",
            "pkg",
            "file.launch.xml",
            "vehicle_model:=sample_vehicle",
        ])
        .expect("--output before `launch` must still parse");
        let Command::Dump(dump_args) = opts.command else {
            panic!("expected Dump");
        };
        assert_eq!(dump_args.output, Some(PathBuf::from("/tmp/aw.yaml")));
    }

    // The verbs this binary actually has. A `--help` that advertises verbs it
    // does not implement is what this test exists to stop recurring: the
    // extracted CLI shipped examples for `launch`, `run` and `replay`.
    #[test]
    fn help_examples_name_only_verbs_this_binary_has() {
        let help = Options::command().render_long_help().to_string();
        // Only the *invocation* forms are banned. Plain "play_launch" still
        // legitimately appears in flag help -- `play_launch_container`, the
        // `$XDG_CONFIG_HOME/play_launch/contracts` overlay path, and the note
        // that RT scheduling is applied by play_launch rather than here.
        for absent in [
            "play_launch launch ",
            "play_launch run ",
            "play_launch dump ",
            "play_launch replay ",
            "play_launch check ",
        ] {
            assert!(
                !help.contains(absent),
                "--help invokes `{absent}` -- wrong binary name, and a verb this one lacks:\n{help}"
            );
        }
        for present in ["resolve", "dump", "check", "contract", "plot"] {
            assert!(help.contains(present), "--help omits the `{present}` verb");
        }
    }

    #[test]
    fn check_accepts_the_full_diagnostic_surface() {
        let opts = parse(&[
            "check",
            "my_bringup",
            "system.launch.xml",
            "mode:=lidar",
            "--format",
            "json",
            "--rule",
            "satisfiability",
            "--rule",
            "consistency",
            "--explain",
        ])
        .expect("check must accept its diagnostic options after launch arguments");
        let Command::Check(args) = opts.command else {
            panic!("expected Check");
        };
        assert_eq!(args.package_or_path, "my_bringup");
        assert_eq!(args.launch_file.as_deref(), Some("system.launch.xml"));
        assert_eq!(args.launch_arguments, vec!["mode:=lidar".to_string()]);
        assert_eq!(args.format, "json");
        assert_eq!(
            args.rule,
            vec!["satisfiability".to_string(), "consistency".to_string()]
        );
        assert!(args.explain);
    }
}
