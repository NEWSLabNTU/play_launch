//! Staged startup: start consumers before producers.
//!
//! # The problem this addresses
//!
//! Phase 61 W1 measured a 144-process launch saturating a 12-core Orin for
//! ~50 s, and cut that by reducing the process count. What it did not address
//! is the *shape* of the remaining window: for however long the stack takes to
//! come up, sensor drivers are publishing into nodes that do not exist yet.
//! Their messages go somewhere — DDS history caches, and the callback queues of
//! subscribers that appear part-way through — and that backlog is the suspected
//! path from "the machine is busy" to "the OOM killer took the desktop".
//!
//! Starting the drivers LAST bounds it directly: by the time anything is
//! published, the things that consume it are already listening.
//!
//! # Why this is not automatic
//!
//! Deriving "who produces, who consumes" needs the topic graph, and
//! `structure.topics` is populated from authored manifests
//! (`model_builder.rs` walks `index.topics`), not from the launch file. A
//! launch with no manifests resolves to **zero topics** — which is the case for
//! the project this was written for. So there are two sources for a stage
//! assignment and the useful one today is the explicit one:
//!
//! 1. `startup.order` — glob patterns per group, in the order the groups
//!    should start. Works on any launch.
//! 2. `startup.defer_sources` — derived from `structure.topics`: a node that
//!    publishes and never subscribes is a source, and goes last. Inert unless
//!    the model carries a topic graph.
//!
//! Both are **off by default**. Reordering startup is a semantic change — a
//! system where something waits on a driver being up early would break — and
//! after W1's experience with a clever default that measured worse than
//! nothing, this one is opt-in and says so.
//!
//! # What a stage waits for
//!
//! Not "the process has spawned": `spawn()` returns long before a ROS node has
//! constructed its subscriptions, so a spawn-counting gate would order the
//! `fork()` calls and nothing that matters. A stage is complete when every
//! member of it is **accounted for** — present in the ROS graph (its
//! subscriptions exist), or loaded (for a composable), or dead (it is not
//! coming). Plus a timeout, because a stage that never completes must not
//! prevent the rest of the system from starting; when it fires, the members
//! still missing are named.

use std::{
    collections::{HashMap, HashSet},
    time::{Duration, Instant},
};

use glob::Pattern;
use ros_launch_manifest_model::SystemModel;
use tokio::sync::watch;
use tracing::{debug, info, warn};

/// Stage 0 starts immediately; higher stages wait for all lower ones.
pub type Stage = u32;

/// One configured group of members that start together, after every earlier
/// group has come up.
#[derive(Debug, Clone)]
pub struct OrderGroup {
    pub name: String,
    pub patterns: Vec<Pattern>,
}

/// Which stage each member FQN belongs to, plus a display name per stage.
#[derive(Debug, Clone, Default)]
pub struct StageAssignment {
    /// FQN → stage. An FQN absent from this map is stage 0.
    stages: HashMap<String, Stage>,
    /// Human-readable name per stage index; `names[0]` is always the implicit
    /// "everything else" group.
    names: Vec<String>,
    /// FQNs whose ROS graph name the model actually knows — i.e. whose launch
    /// entry carried an explicit `name=`.
    ///
    /// For the rest the model's FQN is built from the EXECUTABLE name, and the
    /// running node registers whatever name it was compiled with, so the two do
    /// not match and never will. Measured on the golf cart stack: 17 of 144
    /// nodes, e.g. the model's
    /// `/localization/pose_twist_fusion_filter/autoware_ekf_localizer_node`
    /// against the graph's `/localization/pose_twist_fusion_filter/ekf_localizer`.
    /// play_launch emits `__ns` but no `__node` remap, so it does not impose the
    /// model's name on the node.
    ///
    /// Requiring a graph match for those would make every stage burn its whole
    /// timeout — as it did on the first staged run, where stage 0 sat for the
    /// full 25 s reporting 16 healthy nodes as missing.
    graph_checkable: HashSet<String>,
}

impl StageAssignment {
    /// No staging — every member is stage 0, which is the same as no gate.
    pub fn flat() -> Self {
        Self {
            stages: HashMap::new(),
            names: vec!["default".to_string()],
            graph_checkable: HashSet::new(),
        }
    }

    pub fn stage_for(&self, fqn: &str) -> Stage {
        self.stages.get(fqn).copied().unwrap_or(0)
    }

    pub fn stage_count(&self) -> usize {
        self.names.len()
    }

    pub fn stage_name(&self, stage: Stage) -> &str {
        self.names
            .get(stage as usize)
            .map(String::as_str)
            .unwrap_or("?")
    }

    /// Whether the ROS graph can confirm this member is up, or whether the
    /// model only knows it by its executable name.
    pub fn is_graph_checkable(&self, fqn: &str) -> bool {
        self.graph_checkable.contains(fqn)
    }

    /// Whether anything is actually deferred. A single-stage assignment is a
    /// no-op and callers skip the gate entirely rather than paying for it.
    pub fn is_staged(&self) -> bool {
        self.names.len() > 1 && !self.stages.is_empty()
    }

    /// How many members are deferred past stage 0. Stage 0 is "everything not
    /// mentioned", so its size is the model's node count minus this.
    pub fn members_of_any_later_stage(&self) -> usize {
        self.stages.values().filter(|s| **s > 0).count()
    }

    /// Every FQN in a given stage.
    pub fn members_of(&self, stage: Stage) -> Vec<&str> {
        self.stages
            .iter()
            .filter(|(_, s)| **s == stage)
            .map(|(fqn, _)| fqn.as_str())
            .collect()
    }

    /// Build from explicit groups plus an optional derived "sources last"
    /// pass, against the FQNs the model actually contains.
    ///
    /// Explicit groups win: if a node matches a configured group AND is a
    /// derived source, the configured stage is the one that applies. An
    /// operator who has written the order down means it.
    pub fn resolve(model: &SystemModel, groups: &[OrderGroup], defer_sources: bool) -> Self {
        let mut stages = HashMap::new();
        let mut names = vec!["default".to_string()];

        for (i, group) in groups.iter().enumerate() {
            let stage = (i + 1) as Stage;
            names.push(group.name.clone());
            for fqn in model.structure.nodes.keys() {
                if stages.contains_key(fqn) {
                    // An earlier group already claimed it. First match wins, so
                    // overlapping patterns resolve to the EARLIER stage — a
                    // node listed twice starts as soon as either group allows,
                    // never later than the operator's first mention of it.
                    continue;
                }
                if group.patterns.iter().any(|p| p.matches(fqn)) {
                    stages.insert(fqn.clone(), stage);
                }
            }
        }

        if defer_sources {
            let sources = derive_sources(model);
            if !sources.is_empty() {
                let stage = names.len() as Stage;
                names.push("sources".to_string());
                for fqn in sources {
                    stages.entry(fqn).or_insert(stage);
                }
            }
        }

        // Drop trailing stages nobody landed in, so the gate does not wait on
        // an empty group and the log does not announce one.
        while names.len() > 1 {
            let last = (names.len() - 1) as Stage;
            if stages.values().any(|s| *s == last) {
                break;
            }
            names.pop();
        }

        // Issue #0017 — the rule for "is this key a ROS name?" lives in
        // `ros_launch_resolve::ros::graph_identity`, not here. This gate was
        // the first consumer to need it and had the test inline; every later
        // graph-joining consumer would otherwise have to rediscover it, which
        // is what the issue calls the workaround rather than the fix.
        let graph_checkable = model
            .structure
            .nodes
            .iter()
            .filter(|(_, n)| ros_launch_resolve::ros::graph_identity::is_ros_graph_name(n))
            .map(|(fqn, _)| fqn.clone())
            .collect();

        Self {
            stages,
            names,
            graph_checkable,
        }
    }
}

/// Nodes that publish and never subscribe, per the model's topic graph.
///
/// Returns empty when the model has no topics, which is the common case: a
/// launch with no authored manifests resolves to zero of them.
fn derive_sources(model: &SystemModel) -> Vec<String> {
    if model.structure.topics.is_empty() {
        return Vec::new();
    }

    let mut publishes: HashSet<&str> = HashSet::new();
    let mut subscribes: HashSet<&str> = HashSet::new();

    // Endpoint refs are `"<node FQN>/<endpoint>"`; the node part is everything
    // before the last `/`.
    for wiring in model.structure.topics.values() {
        for e in &wiring.publishers {
            if let Some(node) = e.rsplit_once('/').map(|(n, _)| n) {
                publishes.insert(node);
            }
        }
        for e in &wiring.subscribers {
            if let Some(node) = e.rsplit_once('/').map(|(n, _)| n) {
                subscribes.insert(node);
            }
        }
    }

    let mut out: Vec<String> = publishes
        .difference(&subscribes)
        // Only nodes the model actually knows about — an endpoint ref that did
        // not resolve to a node instance is not something we can schedule.
        .filter(|fqn| model.structure.nodes.contains_key(**fqn))
        .map(|fqn| fqn.to_string())
        .collect();
    out.sort();
    out
}

/// Compile `startup.order` config into groups.
pub fn compile_groups(
    config: &[crate::cli::config::StartupOrderGroup],
) -> eyre::Result<Vec<OrderGroup>> {
    let mut out = Vec::new();
    for (i, g) in config.iter().enumerate() {
        let mut patterns = Vec::new();
        for p in &g.match_ {
            patterns.push(Pattern::new(p).map_err(|e| {
                eyre::eyre!(
                    "startup.order[{i}] ({}): invalid glob {p:?}: {e}",
                    if g.name.is_empty() {
                        "unnamed"
                    } else {
                        &g.name
                    }
                )
            })?);
        }
        if patterns.is_empty() {
            eyre::bail!(
                "startup.order[{i}] ({}) has no `match` patterns — a group that \
                 matches nothing would silently never start anything",
                if g.name.is_empty() {
                    "unnamed"
                } else {
                    &g.name
                }
            );
        }
        out.push(OrderGroup {
            name: if g.name.is_empty() {
                format!("stage{}", i + 1)
            } else {
                g.name.clone()
            },
            patterns,
        });
    }
    Ok(out)
}

/// The gate itself: publishes the highest stage currently allowed to start.
#[derive(Debug)]
pub struct StageGate {
    open: watch::Sender<Stage>,
}

impl StageGate {
    pub fn new() -> Self {
        // Stage 0 is open from the outset: the first group must not wait for
        // anything, including for the advancer task to have been spawned.
        let (open, _) = watch::channel(0);
        Self { open }
    }

    /// Block until `stage` is allowed to start.
    pub async fn wait_for(&self, stage: Stage) {
        if stage == 0 {
            return;
        }
        let mut rx = self.open.subscribe();
        while *rx.borrow() < stage {
            if rx.changed().await.is_err() {
                // The advancer is gone. Opening rather than blocking forever is
                // the only safe reading: a launch that refuses to start is
                // worse than one that starts in the wrong order.
                return;
            }
        }
    }

    fn open_through(&self, stage: Stage) {
        let _ = self.open.send(stage);
    }
}

impl Default for StageGate {
    fn default() -> Self {
        Self::new()
    }
}

/// How a member of the current stage is counted as no longer pending.
enum Accounted {
    /// Present in the ROS graph, or loaded — it is listening.
    Ready,
    /// Dead. Not coming, so waiting for it would only burn the timeout.
    Gone,
    /// Still starting.
    No,
}

/// Advance the gate as each stage comes up. Runs for the life of the startup
/// and then exits; it does nothing once the last stage is open.
pub async fn run_stage_advancer(
    gate: std::sync::Arc<StageGate>,
    assignment: std::sync::Arc<StageAssignment>,
    member_handle: std::sync::Arc<crate::member_actor::MemberHandle>,
    ros_node: Option<std::sync::Arc<rclrs::Node>>,
    stage_timeout: Duration,
    mut shutdown_rx: watch::Receiver<bool>,
) {
    let last = assignment.stage_count().saturating_sub(1) as Stage;
    if last == 0 {
        return;
    }

    for stage in 0..last {
        let started = Instant::now();
        let mut logged_wait = false;

        loop {
            if *shutdown_rx.borrow() {
                debug!("startup order: shutting down while waiting for stage {stage}");
                return;
            }

            let missing =
                pending_members(&assignment, &member_handle, ros_node.as_deref(), stage).await;

            if missing.is_empty() {
                info!(
                    "startup order: stage {stage} ('{}') up after {:.1}s — releasing '{}'",
                    assignment.stage_name(stage),
                    started.elapsed().as_secs_f64(),
                    assignment.stage_name(stage + 1),
                );
                break;
            }

            if started.elapsed() >= stage_timeout {
                // Naming them matters: "timed out" alone would leave an
                // operator to guess which node never came up, and the answer is
                // usually a driver whose hardware is absent.
                let shown: Vec<&str> = missing.iter().take(8).map(String::as_str).collect();
                warn!(
                    "startup order: stage {stage} ('{}') still missing {} member(s) after {:.0}s \
                     — releasing '{}' anyway: {}{}",
                    assignment.stage_name(stage),
                    missing.len(),
                    stage_timeout.as_secs_f64(),
                    assignment.stage_name(stage + 1),
                    shown.join(", "),
                    if missing.len() > shown.len() {
                        format!(", … ({} more)", missing.len() - shown.len())
                    } else {
                        String::new()
                    },
                );
                break;
            }

            if !logged_wait {
                debug!(
                    "startup order: holding '{}' — {} member(s) of stage {stage} still starting",
                    assignment.stage_name(stage + 1),
                    missing.len()
                );
                logged_wait = true;
            }

            tokio::select! {
                _ = tokio::time::sleep(Duration::from_millis(250)) => {}
                _ = shutdown_rx.changed() => {}
            }
        }

        gate.open_through(stage + 1);
    }
}

/// FQNs in stages `<= stage` that are neither ready nor gone.
async fn pending_members(
    assignment: &StageAssignment,
    member_handle: &crate::member_actor::MemberHandle,
    ros_node: Option<&rclrs::Node>,
    stage: Stage,
) -> Vec<String> {
    use crate::member_actor::model::MemberState;

    // The ROS graph is the readiness signal for a plain node: a name appears
    // once the node's context is up, which is when its subscriptions exist.
    let graph: HashSet<String> = ros_node
        .and_then(|n| n.get_node_names().ok())
        .map(|names| {
            names
                .into_iter()
                .map(|info| {
                    let ns = info.namespace.trim_end_matches('/');
                    format!("{ns}/{}", info.name)
                })
                .collect()
        })
        .unwrap_or_default();

    let mut pending = Vec::new();
    for m in member_handle.list_members().await {
        let fqn = member_fqn(&m);
        if assignment.stage_for(&fqn) > stage {
            continue;
        }

        let accounted = match &m.state {
            // A composable that is loaded has been constructed inside its
            // container; that is a stronger and earlier signal than waiting for
            // the graph to propagate.
            MemberState::Loaded { .. } => Accounted::Ready,
            MemberState::Stopped | MemberState::Failed { .. } => Accounted::Gone,
            _ if graph.contains(&fqn) => Accounted::Ready,
            // The model does not know this node's graph name (no `name=` in
            // the launch file, so its FQN is the EXECUTABLE name while the node
            // registers whatever it was compiled with). Waiting for a name that
            // will never appear would burn the stage timeout on a healthy node,
            // so fall back to the process being up. Weaker — the process exists
            // before its subscriptions do — but it is the strongest signal
            // available for these, and it is the pre-staging behaviour rather
            // than a regression.
            MemberState::Running { .. } | MemberState::Respawning { .. }
                if !assignment.is_graph_checkable(&fqn) =>
            {
                Accounted::Ready
            }
            _ => Accounted::No,
        };

        if matches!(accounted, Accounted::No) {
            pending.push(fqn);
        }
    }
    pending.sort();
    pending
}

/// Reconstruct a member's FQN the way the model keys it.
fn member_fqn(m: &crate::member_actor::model::MemberSummary) -> String {
    let ns = m.namespace.as_deref().unwrap_or("/").trim_end_matches('/');
    let name = m.node_name.as_deref().unwrap_or(&m.name);
    format!("{ns}/{name}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_launch_manifest_model::{NodeInstance, SystemModel, TopicWiring};

    fn model_with(nodes: &[&str]) -> SystemModel {
        let mut m = SystemModel::default();
        for n in nodes {
            m.structure
                .nodes
                .insert((*n).to_string(), NodeInstance::default());
        }
        m
    }

    fn group(name: &str, pats: &[&str]) -> OrderGroup {
        OrderGroup {
            name: name.to_string(),
            patterns: pats.iter().map(|p| Pattern::new(p).unwrap()).collect(),
        }
    }

    #[test]
    fn unmatched_nodes_stay_in_stage_zero() {
        // The default must be "start now". A node nobody mentioned must never
        // be delayed by someone else's ordering.
        let m = model_with(&["/planning/x", "/sensing/lidar/driver"]);
        let a = StageAssignment::resolve(&m, &[group("drivers", &["/sensing/**"])], false);
        assert_eq!(a.stage_for("/planning/x"), 0);
        assert_eq!(a.stage_for("/sensing/lidar/driver"), 1);
        assert_eq!(a.stage_for("/not/in/the/model"), 0);
        assert!(a.is_staged());
    }

    #[test]
    fn double_star_crosses_namespace_separators() {
        // The whole feature rests on this: `/sensing/**` has to reach a driver
        // three levels down, or every real launch file needs a pattern per node.
        let m = model_with(&["/sensing/lidar/vlp32/velodyne_ros_wrapper_node"]);
        let a = StageAssignment::resolve(&m, &[group("drivers", &["/sensing/**"])], false);
        assert_eq!(
            a.stage_for("/sensing/lidar/vlp32/velodyne_ros_wrapper_node"),
            1
        );
    }

    #[test]
    fn first_matching_group_wins() {
        // Overlapping patterns resolve to the EARLIER stage, so a node
        // mentioned twice starts no later than its first mention.
        let m = model_with(&["/sensing/imu/corrector"]);
        let a = StageAssignment::resolve(
            &m,
            &[
                group("early", &["/sensing/imu/**"]),
                group("late", &["/sensing/**"]),
            ],
            false,
        );
        assert_eq!(a.stage_for("/sensing/imu/corrector"), 1);
    }

    #[test]
    fn empty_trailing_groups_are_dropped() {
        // A group that matched nothing must not become a stage the gate waits
        // on, or a typo in a pattern would cost a stage timeout.
        let m = model_with(&["/a"]);
        let a = StageAssignment::resolve(&m, &[group("nobody", &["/zzz/**"])], false);
        assert_eq!(a.stage_count(), 1);
        assert!(!a.is_staged());
    }

    #[test]
    fn no_groups_is_a_flat_assignment() {
        let m = model_with(&["/a", "/b"]);
        let a = StageAssignment::resolve(&m, &[], false);
        assert!(!a.is_staged());
        assert_eq!(a.stage_for("/a"), 0);
    }

    #[test]
    fn sources_are_nodes_that_publish_and_never_subscribe() {
        let mut m = model_with(&["/driver", "/filter", "/sink"]);
        m.structure.topics.insert(
            "/raw".to_string(),
            TopicWiring {
                msg_type: "sensor_msgs/msg/PointCloud2".to_string(),
                publishers: vec!["/driver/pub".to_string()],
                subscribers: vec!["/filter/sub".to_string()],
            },
        );
        m.structure.topics.insert(
            "/filtered".to_string(),
            TopicWiring {
                msg_type: "sensor_msgs/msg/PointCloud2".to_string(),
                publishers: vec!["/filter/pub".to_string()],
                subscribers: vec!["/sink/sub".to_string()],
            },
        );

        let sources = derive_sources(&m);
        assert_eq!(sources, vec!["/driver".to_string()]);

        let a = StageAssignment::resolve(&m, &[], true);
        assert_eq!(a.stage_for("/driver"), 1);
        assert_eq!(a.stage_for("/filter"), 0);
        assert_eq!(a.stage_for("/sink"), 0);
    }

    #[test]
    fn derivation_is_inert_without_a_topic_graph() {
        // The case that matters in practice: a launch with no manifests
        // resolves to zero topics, and `defer_sources` must then do nothing
        // rather than, say, treating every node as a source.
        let m = model_with(&["/a", "/b"]);
        assert!(derive_sources(&m).is_empty());
        let a = StageAssignment::resolve(&m, &[], true);
        assert!(!a.is_staged());
    }

    #[test]
    fn explicit_groups_beat_derivation() {
        let mut m = model_with(&["/driver"]);
        m.structure.topics.insert(
            "/raw".to_string(),
            TopicWiring {
                msg_type: "std_msgs/msg/Empty".to_string(),
                publishers: vec!["/driver/pub".to_string()],
                subscribers: vec![],
            },
        );
        // /driver is a derived source, but the operator put it in stage 1
        // explicitly; that is the stage that applies.
        let a = StageAssignment::resolve(&m, &[group("first", &["/driver"])], true);
        assert_eq!(a.stage_for("/driver"), 1);
        assert_eq!(a.stage_name(1), "first");
    }

    #[test]
    fn graph_checkable_only_where_the_launch_named_the_node() {
        // The distinction the first staged run turned up the hard way: a node
        // with no `name=` is keyed in the model by its EXECUTABLE, and the
        // running node registers its own compiled-in name instead, so no graph
        // match is ever possible for it.
        let mut m = SystemModel::default();
        m.structure.nodes.insert(
            "/ns/named_node".to_string(),
            NodeInstance {
                node_name: Some("named_node".to_string()),
                ..Default::default()
            },
        );
        m.structure.nodes.insert(
            "/ns/some_exec_node".to_string(),
            NodeInstance {
                node_name: None,
                exec: Some("some_exec_node".to_string()),
                ..Default::default()
            },
        );

        let a = StageAssignment::resolve(&m, &[], false);
        assert!(a.is_graph_checkable("/ns/named_node"));
        assert!(!a.is_graph_checkable("/ns/some_exec_node"));
    }

    #[tokio::test]
    async fn stage_zero_never_blocks() {
        let gate = StageGate::new();
        tokio::time::timeout(Duration::from_millis(100), gate.wait_for(0))
            .await
            .expect("stage 0 must be open from the outset");
    }

    #[tokio::test]
    async fn later_stages_wait_then_release() {
        let gate = std::sync::Arc::new(StageGate::new());
        let g = gate.clone();
        let waiter = tokio::spawn(async move { g.wait_for(1).await });

        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(!waiter.is_finished(), "stage 1 must not be open yet");

        gate.open_through(1);
        tokio::time::timeout(Duration::from_millis(500), waiter)
            .await
            .expect("opening the stage must release the waiter")
            .unwrap();
    }

    #[tokio::test]
    async fn a_dropped_advancer_opens_the_gate_rather_than_hanging() {
        // If the advancer dies, every remaining stage must start. A launch that
        // refuses to start is worse than one that starts in the wrong order.
        let gate = StageGate::new();
        let waiter = {
            let rx = gate.open.subscribe();
            drop(rx);
            gate.wait_for(3)
        };
        drop(gate.open.clone());
        // The sender still lives here, so exercise the explicit release path.
        tokio::time::timeout(Duration::from_millis(200), async {
            tokio::select! {
                _ = waiter => {}
                _ = tokio::time::sleep(Duration::from_millis(150)) => {}
            }
        })
        .await
        .expect("must not hang");
    }
}
