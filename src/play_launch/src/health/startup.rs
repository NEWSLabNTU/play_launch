//! The startup timeline (`docs/design/health-page.md` §C): when each member
//! was spawned, when its node first appeared, first published, loaded, and
//! when the run as a whole was up — so the member that set the startup time
//! can be named rather than guessed.
//!
//! Every time is milliseconds since the run started (the launch resolved,
//! nothing spawned yet), read from
//! `CLOCK_MONOTONIC`, the interceptor's clock, so its marks join the actors'
//! without conversion. Actor milestones are taken as the actor EMITS its
//! state event (`events::set_emit_hook`). Taken where the events are
//! received instead, a node's spawn read ~250 ms after its first endpoint
//! on Autoware: the receiver starts draining only after the spawn burst.
//! Even at emission, `Started` follows the actor's post-fork setup (pid
//! file, cgroup, scheduling), so a fast node's first endpoint can precede
//! its spawn by a few milliseconds (6–21 ms seen on a loaded host); the
//! numbers are reported as measured and the chart starts at the earlier.
//!
//! "Ready" is the member's own:
//! - a ROS node: its first endpoint (its node exists and is wiring up), when
//!   the run intercepts; without interception, its spawn, which is all the
//!   run can see;
//! - a plain `<executable>`, a container: its spawn;
//! - a composable: its load.

use std::collections::HashMap;

use serde::Serialize;

use super::{Condition, Link, Severity, Subject};
use crate::{
    interception::{LifecycleMark, StartupMark},
    member_actor::{
        events::StateEvent,
        model::member_view::{MemberState, MemberSummary, MemberType},
    },
};

/// A member not ready this long after its spawn is reported. A first-run
/// TensorRT engine build takes 30–45 s on an Orin, so this is a warning,
/// not a failure: the container still reports the constructor alive.
pub const NEVER_READY_AFTER_MS: u64 = 60_000;

fn now_ns() -> u64 {
    crate::runtime_enforcement::monotonic_now_ns()
}

#[derive(Debug, Clone, Default)]
struct Milestones {
    spawned_ns: Option<u64>,
    load_started_ns: Option<u64>,
    loaded_ns: Option<u64>,
    failed_ns: Option<u64>,
    error: Option<String>,
}

/// Actor milestones, first occurrence of each per member.
#[derive(Debug)]
pub struct StartupBook {
    t0_ns: u64,
    members: HashMap<String, Milestones>,
    /// Node FQN -> when it first reached inactive and active, as the
    /// autostart driver saw it.
    lifecycle: HashMap<String, (Option<u64>, Option<u64>)>,
    complete_ns: Option<u64>,
}

impl Default for StartupBook {
    fn default() -> Self {
        Self::new()
    }
}

impl StartupBook {
    /// Start the clock: the run's zero.
    pub fn new() -> Self {
        Self::starting_at(now_ns())
    }

    fn starting_at(t0_ns: u64) -> Self {
        Self {
            t0_ns,
            members: HashMap::new(),
            lifecycle: HashMap::new(),
            complete_ns: None,
        }
    }

    pub fn observe(&mut self, event: &StateEvent) {
        self.observe_at(event, now_ns());
    }

    fn observe_at(&mut self, event: &StateEvent, at: u64) {
        let (name, slot): (&str, fn(&mut Milestones) -> &mut Option<u64>) = match event {
            StateEvent::Started { name, .. } => (name, |m| &mut m.spawned_ns),
            StateEvent::LoadStarted { name } => (name, |m| &mut m.load_started_ns),
            StateEvent::LoadSucceeded { name, .. } => (name, |m| &mut m.loaded_ns),
            StateEvent::Failed { name, error } | StateEvent::LoadFailed { name, error } => {
                let m = self.members.entry(name.clone()).or_default();
                if m.failed_ns.is_none() {
                    m.failed_ns = Some(at);
                    m.error = Some(error.clone());
                }
                return;
            }
            _ => return,
        };
        let s = slot(self.members.entry(name.to_string()).or_default());
        if s.is_none() {
            *s = Some(at);
        }
    }

    /// A lifecycle node reached `state` (2 inactive, 3 active).
    pub fn observe_lifecycle(&mut self, fqn: &str, state: u8) {
        self.observe_lifecycle_at(fqn, state, now_ns());
    }

    fn observe_lifecycle_at(&mut self, fqn: &str, state: u8, at: u64) {
        let e = self.lifecycle.entry(fqn.to_string()).or_default();
        match state {
            2 => {
                e.0.get_or_insert(at);
            }
            3 => {
                e.1.get_or_insert(at);
            }
            _ => {}
        }
    }

    /// The run reached startup complete (first time only).
    pub fn mark_complete(&mut self) {
        self.complete_ns.get_or_insert_with(now_ns);
    }

    pub fn is_complete(&self) -> bool {
        self.complete_ns.is_some()
    }

    fn ms(&self, ns: Option<u64>) -> Option<u64> {
        ns.map(|n| n.saturating_sub(self.t0_ns) / 1_000_000)
    }

    /// The timeline as of `now_ns`. `fqns` maps member id to node FQN, for
    /// joining the interceptor's per-node marks; `intercepting` says whether
    /// those marks exist at all.
    #[allow(clippy::too_many_arguments)] // one caller, the Health task
    pub fn timeline(
        &self,
        members: &[MemberSummary],
        fqns: &HashMap<String, String>,
        marks: &[StartupMark],
        lifecycle_marks: &[LifecycleMark],
        autostart_active: &std::collections::HashSet<String>,
        intercepting: bool,
        now_ns: u64,
    ) -> StartupTimeline {
        // Lifecycle times by FQN: the earlier of the driver's and the
        // interceptor's (a node's own TransitionEvent, whoever asked).
        let mut lifecycle = self.lifecycle.clone();
        for m in lifecycle_marks {
            let e = lifecycle.entry(m.node.clone()).or_default();
            let min = |a: Option<u64>, b: Option<u64>| match (a, b) {
                (Some(x), Some(y)) => Some(x.min(y)),
                (x, y) => x.or(y),
            };
            e.0 = min(e.0, m.inactive_ns);
            e.1 = min(e.1, m.active_ns);
        }
        // A mark belongs to the member whose node it names, else to the
        // process that reported it (a member's own id when not a container).
        let by_fqn: HashMap<&str, &str> = fqns
            .iter()
            .map(|(id, f)| (f.as_str(), id.as_str()))
            .collect();
        let mut node_marks: HashMap<&str, (u64, Option<u64>)> = HashMap::new();
        for m in marks {
            let owner = m
                .node
                .as_deref()
                .and_then(|n| by_fqn.get(n).copied())
                .or_else(|| {
                    // The interceptor keys a process by its model key; a
                    // member id is that key behind a kind (`node:/a/b-1`).
                    members
                        .iter()
                        .find(|s| {
                            s.id == m.member
                                || s.id.split_once(':').is_some_and(|(_, k)| k == m.member)
                        })
                        .filter(|s| !s.is_container)
                        .map(|s| s.id.as_str())
                });
            let Some(owner) = owner else { continue };
            let e = node_marks
                .entry(owner)
                .or_insert((m.first_endpoint_ns, m.first_publish_ns));
            e.0 = e.0.min(m.first_endpoint_ns);
            e.1 = match (e.1, m.first_publish_ns) {
                (Some(a), Some(b)) => Some(a.min(b)),
                (a, b) => a.or(b),
            };
        }

        let mut rows: Vec<StartupRow> = members
            .iter()
            .map(|s| {
                let ms = self.members.get(&s.id).cloned().unwrap_or_default();
                let (endpoint, publish) = node_marks
                    .get(s.id.as_str())
                    .map_or((None, None), |(e, p)| (Some(*e), *p));
                let kind = match s.member_type {
                    MemberType::ComposableNode => "composable",
                    MemberType::Container => "container",
                    MemberType::Node if s.package.is_some() => "node",
                    MemberType::Node => "executable",
                };
                let fqn = fqns.get(&s.id);
                let (inactive_ns, active_ns) = fqn
                    .and_then(|f| lifecycle.get(f).copied())
                    .unwrap_or((None, None));
                // A lifecycle node its launch asks to activate is ready when
                // it is active, not when its process is up.
                let activates = fqn.is_some_and(|f| autostart_active.contains(f));
                let ready_ns = match kind {
                    _ if activates => active_ns,
                    "composable" => ms.loaded_ns,
                    "node" if intercepting => endpoint,
                    _ => ms.spawned_ns,
                };
                let row = StartupRow {
                    id: s.id.clone(),
                    name: s.name.clone(),
                    kind,
                    container: s.target_container.clone(),
                    spawned_ms: self.ms(ms.spawned_ns),
                    load_started_ms: self.ms(ms.load_started_ns),
                    loaded_ms: self.ms(ms.loaded_ns),
                    first_endpoint_ms: self.ms(endpoint),
                    first_publish_ms: self.ms(publish),
                    ready_ms: self.ms(ready_ns),
                    failed_ms: self.ms(ms.failed_ns),
                    error: ms.error.clone(),
                    inactive_ms: self.ms(inactive_ns),
                    active_ms: self.ms(active_ns),
                    activates,
                    waiting_ms: None,
                    state: state_word(&s.state),
                    recent_exits: s.recent_exits,
                };
                let start = match kind {
                    "composable" => ms.load_started_ns,
                    _ => ms.spawned_ns,
                };
                StartupRow {
                    waiting_ms: (row.ready_ms.is_none() && row.failed_ms.is_none())
                        .then(|| start.map(|t| now_ns.saturating_sub(t) / 1_000_000))
                        .flatten(),
                    ..row
                }
            })
            .collect();
        rows.sort_by(|a, b| {
            let key = |r: &StartupRow| r.spawned_ms.or(r.load_started_ms).unwrap_or(u64::MAX);
            key(a).cmp(&key(b)).then_with(|| a.name.cmp(&b.name))
        });
        let last = rows
            .iter()
            .filter_map(|r| r.ready_ms.map(|t| (t, r.id.clone())))
            .max();
        StartupTimeline {
            now_ms: now_ns.saturating_sub(self.t0_ns) / 1_000_000,
            complete_ms: self.ms(self.complete_ns),
            last_ready: last.map(|(_, id)| id),
            intercepting,
            rows,
        }
    }
}

fn state_word(s: &MemberState) -> &'static str {
    match s {
        MemberState::Pending => "pending",
        MemberState::Running { .. } => "running",
        MemberState::Respawning { .. } => "respawning",
        MemberState::Stopped => "stopped",
        MemberState::Failed { .. } => "failed",
        MemberState::Unloaded => "unloaded",
        MemberState::Loading => "loading",
        MemberState::Loaded { .. } => "loaded",
        MemberState::Unloading => "unloading",
        MemberState::Blocked { .. } => "blocked",
    }
}

/// One member's startup, in ms since play_launch started.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct StartupRow {
    pub id: String,
    pub name: String,
    /// `node`, `executable`, `container` or `composable`.
    pub kind: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub container: Option<String>,
    pub spawned_ms: Option<u64>,
    pub load_started_ms: Option<u64>,
    pub loaded_ms: Option<u64>,
    pub first_endpoint_ms: Option<u64>,
    pub first_publish_ms: Option<u64>,
    pub ready_ms: Option<u64>,
    pub failed_ms: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// A lifecycle node's first inactive and active.
    pub inactive_ms: Option<u64>,
    pub active_ms: Option<u64>,
    /// Its launch asks for it to be activated, so ready means active.
    pub activates: bool,
    /// How long it has been waiting to become ready, if it is not.
    pub waiting_ms: Option<u64>,
    pub state: &'static str,
    /// Exits in the last five minutes: a member that keeps dying is not
    /// slow to start, and the crash-loop rule reports it.
    pub recent_exits: u32,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct StartupTimeline {
    pub now_ms: u64,
    /// When the run reached startup complete.
    pub complete_ms: Option<u64>,
    /// The member that became ready last: the one that set the startup time.
    pub last_ready: Option<String>,
    /// Whether node readiness comes from the interceptor (first endpoint)
    /// or only from the spawn.
    pub intercepting: bool,
    pub rows: Vec<StartupRow>,
}

/// `never_ready`: a member waiting longer than [`NEVER_READY_AFTER_MS`]
/// that has not exited meanwhile.
pub fn startup_conditions(t: &StartupTimeline) -> Vec<Condition> {
    t.rows
        .iter()
        .filter(|r| r.recent_exits == 0 && r.waiting_ms.is_some_and(|w| w >= NEVER_READY_AFTER_MS))
        .map(|r| {
            let waited = r.waiting_ms.unwrap_or(0) / 1000;
            let (what, why) = match r.kind {
                _ if r.activates => (
                    if r.inactive_ms.is_some() {
                        "inactive"
                    } else {
                        "unconfigured"
                    },
                    "its launch asks for it to be activated and it has not been: its \
                     configure or activate was refused, or what drives it is waiting",
                ),
                "composable" => (
                    "loading",
                    "its constructor has not returned; the container reports it alive, so \
                     it may be working (a first TensorRT build takes 30–45 s on an Orin) or \
                     waiting on something that never comes",
                ),
                _ => (
                    "starting",
                    "the process runs but has created no publisher or subscription: \
                     blocked before its node, or waiting on something that never comes",
                ),
            };
            Condition {
                key: format!("never_ready:{}", r.id),
                kind: "never_ready",
                severity: Severity::Warning,
                subject: Subject {
                    kind: "member",
                    name: r.name.clone(),
                },
                title: format!("still {what} after {waited} s"),
                detail: why.to_string(),
                link: Some(Link {
                    view: "health",
                    target: Some("startup".to_string()),
                }),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    fn member(id: &str, ty: MemberType, package: bool, state: MemberState) -> MemberSummary {
        MemberSummary {
            id: id.to_string(),
            name: id.to_string(),
            member_type: ty,
            state,
            pid: None,
            package: package.then(|| "pkg".to_string()),
            executable: "x".to_string(),
            namespace: None,
            target_container: (ty == MemberType::ComposableNode).then(|| "c".to_string()),
            is_container: ty == MemberType::Container,
            exec_name: None,
            node_name: None,
            ros_name_known: true,
            stderr_last_modified: None,
            stderr_size: 0,
            stderr_preview: None,
            respawn_enabled: None,
            respawn_delay: None,
            auto_load: None,
            restart_count: None,
            output_dir: Default::default(),
            last_exit: None,
            recent_exits: 0,
        }
    }

    fn started(name: &str) -> StateEvent {
        StateEvent::Started {
            name: name.to_string(),
            pid: 1,
        }
    }

    #[test]
    fn the_last_member_ready_is_named_and_a_slow_one_is_reported() {
        let mut b = StartupBook::starting_at(1_000 * MS);
        b.observe_at(&started("n"), 1_100 * MS);
        b.observe_at(&started("c"), 1_200 * MS);
        b.observe_at(
            &StateEvent::LoadStarted {
                name: "fast".into(),
            },
            1_300 * MS,
        );
        b.observe_at(
            &StateEvent::LoadSucceeded {
                name: "fast".into(),
                full_node_name: "/fast".into(),
                unique_id: 1,
            },
            1_900 * MS,
        );
        b.observe_at(
            &StateEvent::LoadStarted {
                name: "slow".into(),
            },
            1_300 * MS,
        );
        // A respawn does not move the first spawn.
        b.observe_at(&started("n"), 5_000 * MS);
        let members = [
            member("n", MemberType::Node, true, MemberState::Running { pid: 1 }),
            member(
                "c",
                MemberType::Container,
                true,
                MemberState::Running { pid: 2 },
            ),
            member(
                "fast",
                MemberType::ComposableNode,
                true,
                MemberState::Loaded { unique_id: 1 },
            ),
            member(
                "slow",
                MemberType::ComposableNode,
                true,
                MemberState::Loading,
            ),
        ];
        let fqns: HashMap<String, String> = [("n".to_string(), "/n".to_string())].into();
        let marks = [StartupMark {
            member: "n".into(),
            node: Some("/n".into()),
            first_endpoint_ns: 1_150 * MS,
            first_publish_ns: Some(1_400 * MS),
        }];
        let t = b.timeline(
            &members,
            &fqns,
            &marks,
            &[],
            &Default::default(),
            true,
            62_000 * MS,
        );
        let row = |id: &str| t.rows.iter().find(|r| r.id == id).unwrap();
        assert_eq!(row("n").spawned_ms, Some(100));
        assert_eq!(row("n").ready_ms, Some(150), "ready at its first endpoint");
        assert_eq!(row("n").first_publish_ms, Some(400));
        assert_eq!(row("c").ready_ms, Some(200));
        assert_eq!(row("fast").ready_ms, Some(900));
        assert_eq!(row("slow").ready_ms, None);
        assert_eq!(row("slow").waiting_ms, Some(60_700));
        assert_eq!(t.last_ready.as_deref(), Some("fast"));
        assert_eq!(t.rows[0].id, "n", "sorted by spawn");

        let c = startup_conditions(&t);
        assert_eq!(c.len(), 1, "{c:?}");
        assert_eq!(c[0].key, "never_ready:slow");
    }

    #[test]
    fn a_lifecycle_node_its_launch_activates_is_ready_when_active() {
        let mut b = StartupBook::starting_at(0);
        b.observe_at(&started("node:/lc"), 100 * MS);
        b.observe_at(&started("node:/stuck"), 100 * MS);
        // The driver configured /lc; its own TransitionEvent says active.
        b.observe_lifecycle_at("/lc", 2, 400 * MS);
        b.observe_lifecycle_at("/stuck", 2, 500 * MS);
        let marks = [LifecycleMark {
            node: "/lc".into(),
            inactive_ns: Some(450 * MS),
            active_ns: Some(900 * MS),
        }];
        let members = [
            member(
                "node:/lc",
                MemberType::Node,
                true,
                MemberState::Running { pid: 1 },
            ),
            member(
                "node:/stuck",
                MemberType::Node,
                true,
                MemberState::Running { pid: 2 },
            ),
        ];
        let fqns: HashMap<String, String> = [
            ("node:/lc".to_string(), "/lc".to_string()),
            ("node:/stuck".to_string(), "/stuck".to_string()),
        ]
        .into();
        let activates: std::collections::HashSet<String> =
            ["/lc".to_string(), "/stuck".to_string()].into();
        let t = b.timeline(&members, &fqns, &[], &marks, &activates, true, 100_000 * MS);
        let row = |id: &str| t.rows.iter().find(|r| r.id == id).unwrap();
        assert_eq!(row("node:/lc").inactive_ms, Some(400), "the earlier source");
        assert_eq!(row("node:/lc").ready_ms, Some(900), "ready = active");
        assert_eq!(row("node:/stuck").ready_ms, None);
        let c = startup_conditions(&t);
        assert_eq!(c.len(), 1, "{c:?}");
        assert_eq!(c[0].key, "never_ready:node:/stuck");
        assert!(c[0].title.contains("inactive"), "{}", c[0].title);
    }

    #[test]
    fn without_interception_a_node_is_ready_at_its_spawn() {
        let mut b = StartupBook::starting_at(0);
        b.observe_at(&started("n"), 100 * MS);
        b.observe_at(&started("script"), 200 * MS);
        let members = [
            member("n", MemberType::Node, true, MemberState::Running { pid: 1 }),
            member(
                "script",
                MemberType::Node,
                false,
                MemberState::Running { pid: 2 },
            ),
        ];
        let t = b.timeline(
            &members,
            &HashMap::new(),
            &[],
            &[],
            &Default::default(),
            false,
            100_000 * MS,
        );
        assert_eq!(t.rows[0].ready_ms, Some(100));
        assert_eq!(t.rows[1].kind, "executable");
        assert!(startup_conditions(&t).is_empty());

        // With interception, a node without an endpoint after 60 s is late,
        // and a plain executable (no ROS node to wait for) is not.
        let t = b.timeline(
            &members,
            &HashMap::new(),
            &[],
            &[],
            &Default::default(),
            true,
            100_000 * MS,
        );
        let c = startup_conditions(&t);
        assert_eq!(c.len(), 1, "{c:?}");
        assert_eq!(c[0].key, "never_ready:n");
    }

    #[test]
    fn an_unnamed_node_is_joined_by_its_model_key_and_a_crashing_one_is_not_slow() {
        let mut b = StartupBook::starting_at(0);
        b.observe_at(&started("node:/s/agg-1"), 100 * MS);
        b.observe_at(&started("node:/viz"), 100 * MS);
        let mut viz = member(
            "node:/viz",
            MemberType::Node,
            true,
            MemberState::Running { pid: 3 },
        );
        viz.recent_exits = 4;
        let members = [
            member(
                "node:/s/agg-1",
                MemberType::Node,
                true,
                MemberState::Running { pid: 1 },
            ),
            viz,
        ];
        // The interceptor names the real node, which no FQN map knows, and
        // the process by its model key.
        let marks = [StartupMark {
            member: "/s/agg-1".into(),
            node: Some("/s/aggregator".into()),
            first_endpoint_ns: 300 * MS,
            first_publish_ns: None,
        }];
        let t = b.timeline(
            &members,
            &HashMap::new(),
            &marks,
            &[],
            &Default::default(),
            true,
            100_000 * MS,
        );
        assert_eq!(
            t.rows
                .iter()
                .find(|r| r.id == "node:/s/agg-1")
                .unwrap()
                .ready_ms,
            Some(300)
        );
        assert!(
            startup_conditions(&t).is_empty(),
            "the crash loop is not never-ready"
        );
    }
}
