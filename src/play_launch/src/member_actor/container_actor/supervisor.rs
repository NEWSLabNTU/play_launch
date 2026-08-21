//! Composable-node supervision (phase-51.4): the composable map, the
//! load/unload queue, dispatch, and completion handling — split out of the
//! former ContainerActor god struct. The supervisor owns the composables;
//! the actor owns the container process; `ros_client` owns the ROS side.

use super::{
    control_channel::{ControlChannel, LoadFields},
    ros_client::{self, ContainerClients},
    timing::LoadTimings,
};
use crate::member_actor::{
    container_control::{ContainerControlEvent, CurrentUnload, LoadCompletion, LoadRequest},
    events::{StateEvent, emit},
    model::{BlockReason, ComposableState},
};
use std::{
    collections::{HashMap, VecDeque},
    path::PathBuf,
    time::Instant,
};
use tokio::sync::mpsc;
use tracing::{debug, warn};

/// Metadata for a composable node (Phase 12)
#[derive(Debug, Clone)]
pub struct ComposableNodeMetadata {
    /// Package containing the component
    pub package: String,
    /// Plugin name (fully qualified class name)
    pub plugin: String,
    /// ROS node name
    pub node_name: String,
    /// ROS namespace
    pub namespace: String,
    /// Remap rules
    pub remap_rules: Vec<String>,
    /// Parameters (key-value pairs)
    pub parameters: Vec<rcl_interfaces::msg::Parameter>,
    /// Extra arguments
    pub extra_args: Vec<rcl_interfaces::msg::Parameter>,
    /// Auto-load on container startup
    pub auto_load: bool,
    /// Output directory for per-node logging (isolated mode)
    pub output_dir: PathBuf,
    /// Phase 38.9: resolved posix scheduling for this composable (None = no tier).
    pub sched: Option<crate::execution::sched_apply::AppliedTier>,
}

/// Everything phase 64 W2 tracks about a load in flight over the control
/// socket, so that "slow" and "lost" stop being the same observation.
///
/// Present only for socket-dispatched loads; the LoadNode path keeps its own
/// (unchanged) machinery.
#[derive(Debug)]
pub(super) struct LoadTracking {
    /// Correlates the request until the container answers `Accepted`.
    pub(super) seq: u64,
    /// The container's last stated phase. `None` = not acknowledged yet.
    pub(super) phase: Option<crate::ipc::container_protocol::LoadPhase>,
    /// Mirrors the entry's id once the container has accepted the load, so the
    /// sweep can decide which key to ask by without reaching for the entry.
    pub(super) unique_id: Option<u64>,
    /// When the container last said anything about this load.
    pub(super) last_report: Instant,
    /// The child process, once one exists.
    pub(super) pid: i32,
    /// Last two CPU samples, for the stall check's delta.
    pub(super) cpu_ms: u64,
    pub(super) prev_cpu_ms: u64,
    pub(super) cpu_sampled_at: Option<Instant>,
    /// When a `Query` was sent and has not been answered.
    pub(super) probe_sent_at: Option<Instant>,
    /// Consecutive unanswered probes, for logarithmic warning backoff.
    pub(super) unanswered_probes: u32,
    /// Load attempts made so far, this one included.
    pub(super) attempts: usize,
    /// A cancel is out; what to do once the container confirms.
    pub(super) cancel_intent: Option<CancelIntent>,
    /// Whether the stall report has already been printed for this load.
    pub(super) stall_reported: bool,
}

impl LoadTracking {
    pub(super) fn new(seq: u64, attempts: usize) -> Self {
        Self {
            seq,
            phase: None,
            unique_id: None,
            last_report: Instant::now(),
            pid: 0,
            cpu_ms: 0,
            prev_cpu_ms: 0,
            cpu_sampled_at: None,
            probe_sent_at: None,
            unanswered_probes: 0,
            attempts,
            cancel_intent: None,
            stall_reported: false,
        }
    }
}

/// What a pending cancellation is for. The container's confirmation is what
/// makes the difference between a retry and a double load, so the intent is
/// recorded when the cancel is SENT and acted on only when it is answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CancelIntent {
    /// Give up on this composable.
    Fail,
    /// Load it again once the container confirms nothing is running.
    Restart,
}

/// Entry for a composable node managed by this container (Phase 12)
#[derive(Debug)]
pub(super) struct ComposableNodeEntry {
    /// Metadata for this composable node
    pub(super) metadata: ComposableNodeMetadata,
    /// Current state
    pub(super) state: ComposableState,
    /// Unique ID from LoadNode response (if loaded)
    pub(super) unique_id: Option<u64>,
    /// When the load started (for timeout detection)
    pub(super) load_started_at: Option<Instant>,
    /// Phase 64 W2 tracking for a socket-dispatched load (`None` on the
    /// LoadNode path).
    pub(super) tracking: Option<LoadTracking>,
    /// Set when a reload is due at a specific time (crash respawn delay).
    pub(super) retry_after: Option<Instant>,
    /// Crashes seen since this composable last loaded successfully.
    pub(super) crash_count: u32,
}

/// Supervises the composable nodes of one container: composable map,
/// pending-load queue, parallel dispatch, and completion handling.
pub(super) struct ComposableSupervisor {
    /// The owning container actor's name (canonical member id) — logs only
    container_name: String,
    /// Channel to send state events (shared with the actor)
    state_tx: mpsc::Sender<StateEvent>,
    /// Composable nodes managed by this container (member id -> entry)
    pub(super) composable_nodes: HashMap<String, ComposableNodeEntry>,
    /// Queue of pending load requests
    pending_loads: VecDeque<LoadRequest>,
    /// Channel for load completion notifications (parallel dispatch)
    load_completion_tx: mpsc::UnboundedSender<LoadCompletion>,
    pub(super) load_completion_rx: mpsc::UnboundedReceiver<LoadCompletion>,
    /// Currently processing unload
    pub(super) current_unload: Option<CurrentUnload>,
    /// Tunable LoadNode-path timings (phase-52.2)
    pub(super) timings: LoadTimings,
    /// Phase 61: shared startup admission control, consulted before each
    /// LoadNode call so composable loading is bounded across all containers.
    startup: std::sync::Arc<crate::execution::startup_governor::StartupGovernor>,
    /// When construction progress was last reported for this container, so a
    /// launch with dozens of composables does not print a wall of them.
    pub(super) last_progress_report: Option<Instant>,
}

impl ComposableSupervisor {
    pub(super) fn new(
        container_name: String,
        state_tx: mpsc::Sender<StateEvent>,
        timings: LoadTimings,
        startup: std::sync::Arc<crate::execution::startup_governor::StartupGovernor>,
    ) -> Self {
        let (load_completion_tx, load_completion_rx) = mpsc::unbounded_channel();
        Self {
            container_name,
            state_tx,
            composable_nodes: HashMap::new(),
            pending_loads: VecDeque::new(),
            load_completion_tx,
            load_completion_rx,
            current_unload: None,
            timings,
            startup,
            last_progress_report: None,
        }
    }

    pub(super) fn name(&self) -> &str {
        &self.container_name
    }

    pub(super) fn state_tx(&self) -> &mpsc::Sender<StateEvent> {
        &self.state_tx
    }

    /// Add a composable node to be managed by this container (Phase 12)
    pub(super) fn add_composable_node(&mut self, name: String, metadata: ComposableNodeMetadata) {
        let entry = ComposableNodeEntry {
            metadata,
            state: ComposableState::Blocked {
                reason: BlockReason::ContainerNotStarted,
            },
            unique_id: None,
            load_started_at: None,
            tracking: None,
            retry_after: None,
            crash_count: 0,
        };

        self.composable_nodes.insert(name, entry);
    }

    /// Convert ROS parameters to string tuples.
    pub(super) fn ros_params_to_strings(
        params: &[rcl_interfaces::msg::Parameter],
    ) -> Vec<(String, String)> {
        params
            .iter()
            .map(|param| {
                let value_str = Self::parameter_value_to_string(&param.value);
                (param.name.clone(), value_str)
            })
            .collect()
    }

    /// Convert a ParameterValue to string representation.
    pub(super) fn parameter_value_to_string(value: &rcl_interfaces::msg::ParameterValue) -> String {
        use rcl_interfaces::msg::ParameterType;

        match value.type_ {
            ParameterType::PARAMETER_BOOL => value.bool_value.to_string(),
            ParameterType::PARAMETER_INTEGER => value.integer_value.to_string(),
            ParameterType::PARAMETER_DOUBLE => {
                // Format double to ensure it always has a decimal point
                // This prevents "1.0" from becoming "1" which would be interpreted as integer
                let s = value.double_value.to_string();
                if s.contains('.') || s.contains('e') || s.contains('E') {
                    s
                } else {
                    format!("{}.0", s)
                }
            }
            ParameterType::PARAMETER_STRING => value.string_value.clone(),
            ParameterType::PARAMETER_BYTE_ARRAY => format!("{:?}", value.byte_array_value),
            ParameterType::PARAMETER_BOOL_ARRAY => format!("{:?}", value.bool_array_value),
            ParameterType::PARAMETER_INTEGER_ARRAY => format!("{:?}", value.integer_array_value),
            ParameterType::PARAMETER_DOUBLE_ARRAY => format!("{:?}", value.double_array_value),
            ParameterType::PARAMETER_STRING_ARRAY => format!("{:?}", value.string_array_value),
            _ => String::new(),
        }
    }

    /// Phase 64: ask our own container to load a composable over the private
    /// control socket.
    ///
    /// There is no queue, no dispatch task and no startup permit here. A permit
    /// meant "a LoadNode call is in flight"; on this path there is no call to
    /// be in flight — the frame is written and the container paces its own
    /// spawns against `MemAvailable`, which is the governor that was measured
    /// to matter (`docs/design/composable-load-admission.md`). The mode-aware
    /// cap was already disabled for `isolated` for the same reason.
    ///
    /// Parameters go over the wire in the types the model resolved them to,
    /// rather than through the `Parameter -> String -> Parameter` round trip
    /// the service path performs: re-inferring a type from a rendered string
    /// turns a `string` parameter whose value happens to be `"true"` into a
    /// `bool`. Same values, one fewer chance to change them.
    pub(super) async fn send_load_over_socket(&mut self, name: &str, channel: &mut ControlChannel) {
        use crate::ipc::container_protocol::ControlParam;

        let Some(entry) = self.composable_nodes.get(name) else {
            return;
        };
        let meta = &entry.metadata;

        // The isolated container redirects the child's stdout/stderr into this
        // directory, and cannot create it itself without knowing it is allowed
        // to: the service path creates it here too.
        if !meta.output_dir.as_os_str().is_empty()
            && let Err(e) = std::fs::create_dir_all(&meta.output_dir)
        {
            warn!(
                "{}: failed to create output dir for {}: {}",
                self.container_name, name, e
            );
        }

        let fields = LoadFields {
            package: meta.package.clone(),
            plugin: meta.plugin.clone(),
            node_name: meta.node_name.clone(),
            node_namespace: meta.namespace.clone(),
            remap_rules: meta.remap_rules.clone(),
            parameters: meta.parameters.iter().map(ControlParam::from).collect(),
            extra_arguments: meta.extra_args.iter().map(ControlParam::from).collect(),
            log_dir: meta.output_dir.to_string_lossy().into_owned(),
        };

        // Carry the attempt count across a resend: the budget is per
        // composable, not per dispatch, and the whole point of W2 is that a
        // second attempt is rare and accounted for.
        let attempts = self
            .composable_nodes
            .get(name)
            .and_then(|e| e.tracking.as_ref())
            .map(|t| t.attempts)
            .unwrap_or(0);

        match channel.send_load(name, fields) {
            Ok(seq) => {
                if let Some(entry) = self.composable_nodes.get_mut(name) {
                    entry.tracking = Some(LoadTracking::new(seq, attempts + 1));
                    entry.retry_after = None;
                    // Each attempt gets its own clock. Carrying the previous
                    // attempt's elapsed time forward would report a fresh
                    // constructor as already minutes old, and — where stall
                    // detection is enabled — declare it stalled before it had
                    // been given any time at all.
                    let started_at = Instant::now();
                    entry.state = ComposableState::Loading { started_at };
                    entry.load_started_at = Some(started_at);
                }
                debug!(
                    "{}: sent load for '{}' over the control channel (seq {}, attempt {})",
                    self.container_name,
                    name,
                    seq,
                    attempts + 1
                );
            }
            Err(e) => {
                let error = format!("control-channel: {e:#}");
                warn!(
                    "{}: could not send load for '{}': {}",
                    self.container_name, name, error
                );
                if let Some(entry) = self.composable_nodes.get_mut(name) {
                    entry.state = ComposableState::Failed {
                        error: error.clone(),
                    };
                    entry.load_started_at = None;
                }
                emit(
                    &self.state_tx,
                    StateEvent::LoadFailed {
                        name: name.to_string(),
                        error,
                    },
                )
                .await;
            }
        }
    }

    /// Dispatch ALL pending loads concurrently. Each load is spawned as an
    /// independent tokio task that sends its result back via the
    /// load_completion channel.
    ///
    /// Phase 61: the tasks are dispatched immediately but each one waits for a
    /// slot from the shared `StartupGovernor` before calling LoadNode, so the
    /// number of loads actually in flight is bounded across every container
    /// rather than being every composable in the launch at once.
    pub(super) fn dispatch_pending_loads(&mut self, clients: &ContainerClients) {
        while let Some(request) = self.pending_loads.pop_front() {
            let queue_wait_ms = request.request_time.elapsed().as_millis() as u64;

            debug!(
                "{}: Dispatching load for {} (queue wait: {}ms)",
                self.container_name, request.composable_name, queue_wait_ms
            );

            let tx = self.load_completion_tx.clone();
            let composable_name = request.composable_name.clone();
            let container_name = self.container_name.clone();
            let load_client = clients.load_client.clone();
            let list_client = clients.list_client.clone();
            let has_event_sub = clients.has_event_sub();
            let timings = self.timings;
            let params = crate::member_actor::container_control::LoadParams {
                composable_name: request.composable_name,
                package: request.package,
                plugin: request.plugin,
                node_name: request.node_name,
                node_namespace: request.node_namespace,
                remap_rules: request.remap_rules,
                parameters: request.parameters,
                extra_args: request.extra_args,
                output_dir: request.output_dir,
                request_time: request.request_time,
            };

            let startup = self.startup.clone();

            tokio::spawn(async move {
                // Phase 61: hold a load slot for the duration of the call.
                // Every container dispatches its whole queue here, so without
                // a shared gate a 16-container launch puts all 84 of its
                // composables into LoadNode simultaneously — and in
                // `isolated` mode each of those forks a process. The permit
                // is dropped when this task ends, which is exactly when the
                // load finished, failed, or gave up.
                let _slot = startup.admit_load(&composable_name).await;

                // The budget clock starts HERE, after the slot is held, not at
                // dispatch. `timings.total_budget` is measured from
                // `start_time` (`ros_client.rs`), so taking it before
                // `admit_load` charged time spent QUEUED against the time
                // allowed for the load itself — a node that waited out its
                // budget behind other loads would have been failed before its
                // LoadNode call was ever made. Harmless before phase 61, when
                // every load was dispatched immediately and there was no
                // queue; a real hazard now that there is one, and worst
                // exactly where loads are slowest.
                let start_time = std::time::Instant::now();

                let result = ros_client::call_load_node_service(
                    container_name,
                    load_client,
                    list_client,
                    has_event_sub,
                    timings,
                    params,
                    start_time,
                )
                .await;
                let _ = tx.send(LoadCompletion {
                    composable_name,
                    result,
                });
            });
        }
    }

    /// Handle a completed LoadNode service call.
    pub(super) async fn handle_load_completion(
        &mut self,
        completion: LoadCompletion,
        has_event_sub: bool,
    ) {
        let composable_name = &completion.composable_name;

        let Some(entry) = self.composable_nodes.get_mut(composable_name) else {
            warn!(
                "{}: Load completion for unknown composable '{}'",
                self.container_name, composable_name
            );
            return;
        };

        match completion.result {
            Ok(response) if response.success => {
                // unique_id 0 = synthesized "response lost, awaiting
                // ComponentEvent" — keep None so the event path's
                // name-fallback matching can claim this entry.
                if response.unique_id != 0 {
                    entry.unique_id = Some(response.unique_id);
                }

                if !has_event_sub {
                    // Stock container: service response is the only signal, treat as loaded.
                    if matches!(entry.state, ComposableState::Loading { .. }) {
                        entry.state = ComposableState::Loaded {
                            unique_id: response.unique_id,
                        };
                        entry.load_started_at = None;

                        debug!(
                            "{}: Successfully loaded composable node '{}' (unique_id: {})",
                            self.container_name, composable_name, response.unique_id
                        );

                        emit(
                            &self.state_tx,
                            StateEvent::LoadSucceeded {
                                name: composable_name.clone(),
                                full_node_name: response.full_node_name.clone(),
                                unique_id: response.unique_id,
                            },
                        )
                        .await;
                    }
                }
                // play_launch_container (with event sub): service responds immediately
                // before fork+exec completes. The real result arrives via ComponentEvent.
                // State stays Loading; ComponentEvent transitions to Loaded or Failed.
                //
                // NOTE: DDS may drop the ComponentEvent, leaving the node stuck in Loading.
                // The existing 10s timeout fallback (in component_events.rs) promotes
                // stuck nodes to Loaded. This is a known DDS limitation.
            }
            Ok(response) => {
                // LoadNode returned failure
                if matches!(entry.state, ComposableState::Loading { .. }) {
                    entry.state = ComposableState::Failed {
                        error: response.error_message.clone(),
                    };
                    entry.load_started_at = None;

                    warn!(
                        "{}: Failed to load composable node '{}': {}",
                        self.container_name, composable_name, response.error_message
                    );

                    emit(
                        &self.state_tx,
                        StateEvent::LoadFailed {
                            name: composable_name.clone(),
                            error: response.error_message,
                        },
                    )
                    .await;
                }
            }
            Err(e) => {
                // Typed load failure (phase-52.4) — Display carries the kind
                // prefix (timeout:/container-busy:/…), so the web UI's Failed
                // text distinguishes them too.
                if matches!(entry.state, ComposableState::Loading { .. }) {
                    let error_msg = e.to_string();
                    entry.state = ComposableState::Failed {
                        error: error_msg.clone(),
                    };
                    entry.load_started_at = None;

                    warn!(
                        "{}: LoadNode service call failed for '{}': {:#}",
                        self.container_name, composable_name, e
                    );

                    emit(
                        &self.state_tx,
                        StateEvent::LoadFailed {
                            name: composable_name.clone(),
                            error: error_msg,
                        },
                    )
                    .await;
                }
            }
        }
    }

    /// Drain the load queue.
    ///
    /// Pending loads are dropped. In-flight spawned tasks will send completions
    /// to the channel, but handle_load_completion will see the composable node
    /// is Blocked and ignore them.
    pub(super) fn drain_queue(&mut self, error: &str) {
        // Cancel current unload
        if let Some(unload) = self.current_unload.take() {
            debug!(
                "{}: Cancelling current unload for {}: {}",
                self.container_name, unload.composable_name, error
            );
        }

        // Drop all queued loads (spawned tasks will complete harmlessly)
        let queue_len = self.pending_loads.len();
        if queue_len > 0 {
            debug!(
                "{}: Draining {} queued load requests: {}",
                self.container_name, queue_len, error
            );
        }
        self.pending_loads.clear();
    }

    /// Handle LoadComposable control event.
    pub(super) async fn handle_load_composable(
        &mut self,
        name: &str,
        clients: &ContainerClients,
        control: Option<&mut ControlChannel>,
    ) {
        debug!(
            "{}: Handling LoadComposable for '{}'",
            self.container_name, name
        );

        // Check if composable node exists
        let entry = match self.composable_nodes.get_mut(name) {
            Some(e) => e,
            None => {
                warn!(
                    "{}: Composable node '{}' not found",
                    self.container_name, name
                );
                return;
            }
        };

        // Check current state
        match &entry.state {
            ComposableState::Unloaded | ComposableState::Failed { .. } => {
                // Transition to Loading and start the load operation
                let started_at = Instant::now();
                entry.state = ComposableState::Loading { started_at };
                entry.load_started_at = Some(started_at);

                debug!(
                    "{}: Transitioning '{}' to Loading state",
                    self.container_name, name
                );

                // Emit LoadStarted event
                emit(
                    &self.state_tx,
                    StateEvent::LoadStarted {
                        name: name.to_string(),
                    },
                )
                .await;

                // Phase 64: when the container is ours and the private
                // control channel negotiated, the load goes over the socket
                // and none of the LoadNode queue/dispatch/timeout machinery
                // below runs at all.
                if let Some(channel) = control.filter(|c| c.loads_over_socket()) {
                    self.send_load_over_socket(name, channel).await;
                    return;
                }

                // Phase 12: Queue LoadNode request and dispatch immediately
                if let Some(entry) = self.composable_nodes.get(name) {
                    // Convert ROS parameters to string format
                    let parameters = Self::ros_params_to_strings(&entry.metadata.parameters);
                    let extra_args = Self::ros_params_to_strings(&entry.metadata.extra_args);

                    let request = LoadRequest {
                        composable_name: name.to_string(),
                        package: entry.metadata.package.clone(),
                        plugin: entry.metadata.plugin.clone(),
                        node_name: entry.metadata.node_name.clone(),
                        node_namespace: entry.metadata.namespace.clone(),
                        remap_rules: entry.metadata.remap_rules.clone(),
                        parameters,
                        extra_args,
                        output_dir: entry.metadata.output_dir.clone(),
                        request_time: started_at,
                    };

                    self.pending_loads.push_back(request);
                    self.dispatch_pending_loads(clients);
                }
            }
            ComposableState::Loading { .. } => {
                debug!(
                    "{}: '{}' already loading, ignoring",
                    self.container_name, name
                );
            }
            ComposableState::Unloading { .. } => {
                debug!(
                    "{}: '{}' is unloading, ignoring load request",
                    self.container_name, name
                );
            }
            ComposableState::Loaded { .. } => {
                debug!(
                    "{}: '{}' already loaded, ignoring",
                    self.container_name, name
                );
            }
            ComposableState::Blocked { .. } => {
                warn!(
                    "{}: Cannot load '{}' - container blocked",
                    self.container_name, name
                );
            }
        }
    }

    /// Handle UnloadComposable control event.
    pub(super) async fn handle_unload_composable(
        &mut self,
        name: &str,
        clients: &ContainerClients,
    ) {
        debug!(
            "{}: Handling UnloadComposable for '{}'",
            self.container_name, name
        );

        // Check if composable node exists
        let entry = match self.composable_nodes.get_mut(name) {
            Some(e) => e,
            None => {
                warn!(
                    "{}: Composable node '{}' not found in composable_nodes HashMap",
                    self.container_name, name
                );
                warn!(
                    "{}: Available composable nodes: {:?}",
                    self.container_name,
                    self.composable_nodes.keys().collect::<Vec<_>>()
                );
                return;
            }
        };

        // Check current state - only unload if loaded
        match &entry.state {
            ComposableState::Loaded { unique_id } => {
                let unique_id = *unique_id;
                let started_at = Instant::now();

                debug!(
                    "{}: Unloading '{}' (unique_id: {})",
                    self.container_name, name, unique_id
                );

                // Transition to Unloading state
                entry.state = ComposableState::Unloading { started_at };
                entry.load_started_at = Some(started_at);

                // Spawn UnloadNode service call as a tracked task
                let container_name = self.container_name.clone();
                let unload_client = clients.unload_client.clone();

                let unload_timeout = self.timings.service_call_timeout;
                let task = tokio::spawn(async move {
                    ros_client::call_unload_node_service(
                        container_name,
                        unload_client,
                        unique_id,
                        unload_timeout,
                    )
                    .await
                });

                // Track the unload operation (will be polled in select! loop)
                self.current_unload = Some(CurrentUnload {
                    composable_name: name.to_string(),
                    task,
                });
            }
            _ => {
                warn!(
                    "{}: Cannot unload '{}' - not in Loaded state (current: {:?})",
                    self.container_name, name, entry.state
                );
            }
        }
    }

    /// Handle LoadAllComposables control event.
    pub(super) async fn handle_load_all_composables(
        &mut self,
        clients: &ContainerClients,
        mut control: Option<&mut ControlChannel>,
    ) {
        debug!("{}: Handling LoadAllComposables", self.container_name);

        // Log state of all composable nodes for debugging
        let total_nodes = self.composable_nodes.len();
        let state_summary: HashMap<String, usize> =
            self.composable_nodes
                .values()
                .fold(HashMap::new(), |mut acc, entry| {
                    let state_name = match &entry.state {
                        ComposableState::Blocked { .. } => "Blocked",
                        ComposableState::Unloaded => "Unloaded",
                        ComposableState::Loading { .. } => "Loading",
                        ComposableState::Unloading { .. } => "Unloading",
                        ComposableState::Loaded { .. } => "Loaded",
                        ComposableState::Failed { .. } => "Failed",
                    };
                    *acc.entry(state_name.to_string()).or_insert(0) += 1;
                    acc
                });

        debug!(
            "{}: Composable node states: {} total ({:?})",
            self.container_name, total_nodes, state_summary
        );

        // Collect names of nodes to load (to avoid borrowing issues)
        let nodes_to_load: Vec<String> = self
            .composable_nodes
            .iter()
            .filter(|(_, entry)| {
                entry.metadata.auto_load
                    && matches!(
                        entry.state,
                        ComposableState::Unloaded | ComposableState::Failed { .. }
                    )
            })
            .map(|(name, _)| name.clone())
            .collect();

        if nodes_to_load.is_empty() {
            debug!(
                "{}: No composable nodes to load (all {} nodes already loaded or not marked for auto_load)",
                self.container_name, total_nodes
            );
        } else {
            debug!(
                "{}: Queueing {} composable nodes for loading: {:?}",
                self.container_name,
                nodes_to_load.len(),
                nodes_to_load
            );
        }

        // Load each node
        for name in nodes_to_load {
            self.handle_load_composable(&name, clients, control.as_deref_mut())
                .await;
        }
    }

    /// Handle ToggleComposableAutoLoad control event.
    pub(super) async fn handle_toggle_composable_auto_load(&mut self, name: &str, enabled: bool) {
        debug!(
            "{}: Handling ToggleComposableAutoLoad for '{}' (enabled: {})",
            self.container_name, name, enabled
        );

        // Find and update the composable node's auto_load setting
        if let Some(entry) = self.composable_nodes.get_mut(name) {
            entry.metadata.auto_load = enabled;
            debug!(
                "{}: Updated auto_load for '{}' to {}",
                self.container_name, name, enabled
            );
        } else {
            warn!(
                "{}: Cannot toggle auto_load for '{}': composable node not found",
                self.container_name, name
            );
        }
    }

    /// Handle UnloadAllComposables control event.
    pub(super) async fn handle_unload_all_composables(&mut self, clients: &ContainerClients) {
        debug!("{}: Handling UnloadAllComposables", self.container_name);

        // Collect names of loaded nodes
        let nodes_to_unload: Vec<String> = self
            .composable_nodes
            .iter()
            .filter(|(_, entry)| matches!(entry.state, ComposableState::Loaded { .. }))
            .map(|(name, _)| name.clone())
            .collect();

        debug!(
            "{}: Unloading {} loaded composable nodes",
            self.container_name,
            nodes_to_unload.len()
        );

        // Unload each node
        for name in nodes_to_unload {
            self.handle_unload_composable(&name, clients).await;
        }
    }

    /// Handle a load control event (LoadNode / UnloadNode from the control channel).
    pub(super) async fn handle_load_control_event(
        &mut self,
        event: ContainerControlEvent,
        clients: &ContainerClients,
        control: Option<&mut ControlChannel>,
    ) {
        match event {
            ContainerControlEvent::LoadNode {
                composable_name,
                package,
                plugin,
                node_name,
                node_namespace,
                remap_rules,
                parameters,
                extra_args,
                output_dir,
            } => {
                debug!(
                    "{}: Received load request for {}",
                    self.container_name, composable_name
                );

                if let Some(channel) = control.filter(|c| c.loads_over_socket()) {
                    self.send_load_over_socket(&composable_name, channel).await;
                    return;
                }

                let request = LoadRequest {
                    composable_name,
                    package,
                    plugin,
                    node_name,
                    node_namespace,
                    remap_rules,
                    parameters,
                    extra_args,
                    output_dir,
                    request_time: std::time::Instant::now(),
                };

                self.pending_loads.push_back(request);
                self.dispatch_pending_loads(clients);
            }
            ContainerControlEvent::UnloadNode {
                composable_name,
                unique_id,
                response_tx,
            } => {
                debug!(
                    "{}: Received unload request for {} (unique_id: {})",
                    self.container_name, composable_name, unique_id
                );
                self.handle_unload_request(composable_name, unique_id, response_tx, clients)
                    .await;
            }
        }
    }

    /// Handle an UnloadNode request by spawning a service call task.
    pub(super) async fn handle_unload_request(
        &self,
        composable_name: String,
        unique_id: u64,
        response_tx: tokio::sync::oneshot::Sender<
            eyre::Result<crate::member_actor::container_control::UnloadNodeResponse>,
        >,
        clients: &ContainerClients,
    ) {
        let container_name = self.container_name.clone();
        let unload_client = clients.unload_client.clone();
        let unload_timeout = self.timings.service_call_timeout;

        // Spawn service call as a task (so it doesn't block the actor)
        tokio::spawn(async move {
            let result = ros_client::call_unload_node_service(
                container_name,
                unload_client,
                unique_id,
                unload_timeout,
            )
            .await;

            // Send response back
            if let Err(result) = response_tx.send(result) {
                warn!(
                    "Failed to send UnloadNode response for {}: {:?}",
                    composable_name, result
                );
            }
        });
    }

    /// Handle completion of an UnloadNode service call task.
    pub(super) async fn handle_unload_completion(
        &mut self,
        result: Result<
            eyre::Result<crate::member_actor::container_control::UnloadNodeResponse>,
            tokio::task::JoinError,
        >,
        composable_name: String,
    ) {
        match result {
            Ok(service_result) => {
                // Phase 12: Update composable node state directly
                if let Some(entry) = self.composable_nodes.get_mut(&composable_name) {
                    match &service_result {
                        Ok(response) if response.success => {
                            // Successfully unloaded
                            entry.state = ComposableState::Unloaded;
                            entry.unique_id = None;
                            entry.load_started_at = None;

                            debug!(
                                "{}: Successfully unloaded composable node '{}'",
                                self.container_name, composable_name
                            );

                            // Emit Unloaded event
                            emit(
                                &self.state_tx,
                                StateEvent::Unloaded {
                                    name: composable_name.clone(),
                                },
                            )
                            .await;
                        }
                        Ok(response) => {
                            // UnloadNode service returned failure - keep in current state
                            warn!(
                                "{}: Failed to unload composable node '{}': {}",
                                self.container_name, composable_name, response.error_message
                            );
                            // Transition back to Loaded state since unload failed
                            // Note: We don't have unique_id anymore, so this is a problem
                            // For now, keep in Unloading state (user can retry)
                        }
                        Err(e) => {
                            // Service call error - keep in current state
                            warn!(
                                "{}: UnloadNode service call failed for '{}': {:#}",
                                self.container_name, composable_name, e
                            );
                            // Keep in Unloading state for retry
                        }
                    }
                }
            }
            Err(e) => {
                // Task panicked
                warn!(
                    "{}: UnloadNode task panicked for {}: {:#}",
                    self.container_name, composable_name, e
                );
            }
        }
    }
}
