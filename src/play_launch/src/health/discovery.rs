//! Discovery load (big-system-observability §N): how many DDS participants
//! the run puts on its domain, against what the middleware allows, and
//! whether the domain's sockets drop datagrams.
//!
//! A participant is a process holding a ROS context, so the count is
//! processes, not nodes: one per node process, one per container, and under
//! `isolated` one more per composable (the monitor counts a member's forked
//! descendants). A plain `<executable>` may or may not be a ROS process;
//! those are reported as a range rather than guessed.
//!
//! Limits:
//! - Fast DDS maps participant `p` of domain `d` to unicast ports
//!   `7410 + 250d + 2p`; past 120 participants the ports run into domain
//!   `d + 1`'s range.
//! - Cyclone DDS with `ParticipantIndex` `auto` refuses a participant past
//!   `MaxAutoParticipantIndex` (default 9): "Failed to find a free participant
//!   index", and the process fails to start.

use serde::Serialize;

use super::{Condition, Severity, Subject};
use crate::member_actor::model::member_view::{MemberState, MemberSummary, MemberType};

/// Fast DDS participants per domain before ports overlap the next domain.
const FASTDDS_PER_DOMAIN: u32 = 120;
/// Cyclone's default `MaxAutoParticipantIndex`.
const CYCLONE_DEFAULT_MAX_AUTO: u32 = 9;
/// Drops on the domain's sockets per minute worth reporting.
const DOMAIN_DROPS_PER_MIN: f64 = 10.0;
/// How often the domain's sockets are read while starting up.
pub const STARTUP_DROP_SAMPLE: std::time::Duration = std::time::Duration::from_secs(5);
/// Startup, for drops, lasts this long past startup complete: the last
/// participants are still being discovered by everyone else.
const STARTUP_TAIL: std::time::Duration = std::time::Duration::from_secs(30);
/// Datagrams dropped during startup worth one `discovery_pressure` issue.
const STARTUP_DROPS: u64 = 1000;
/// Issue kinds a discovery storm shows up as, listed as its evidence.
const STORM_SYMPTOMS: [&str; 6] = [
    "dds_udp_drops",
    "participants_over_limit",
    "take_deficit",
    "delivery_latency",
    "never_ready",
    "missing_node",
];

#[derive(Debug, Clone, Default, Serialize, PartialEq)]
pub struct DiscoveryView {
    pub rmw: String,
    pub domain_id: u32,
    /// Participants this run's ROS members hold (nodes, containers and
    /// their forked composables).
    pub participants: u32,
    /// Running `<executable>`s, which may also hold one each.
    pub executables: u32,
    /// What `--container-mode observable` would hold: one per container.
    pub participants_observable: u32,
    /// The middleware's limit for this setup, when it has one.
    pub limit: Option<u32>,
    pub limit_reason: Option<String>,
    /// Nodes on the domain at the last graph read, this run's and others'.
    pub graph_nodes: Option<usize>,
    pub domain_udp_sockets: usize,
    pub domain_udp_drops_per_min: Option<f64>,
    pub udp_rcvbuf_errors_per_min: Option<f64>,
}

/// Count participants from the members and, when the monitor runs, each
/// member's process count.
pub fn view(
    members: &[MemberSummary],
    processes: &std::collections::HashMap<String, u32>,
    host: Option<&super::host::HostFacts>,
    graph_nodes: Option<usize>,
) -> DiscoveryView {
    let up = |m: &MemberSummary| matches!(m.state, MemberState::Running { .. });
    let dir = |m: &MemberSummary| {
        m.output_dir
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default()
    };
    let (mut participants, mut observable, mut executables) = (0u32, 0u32, 0u32);
    for m in members.iter().filter(|m| up(m)) {
        match m.member_type {
            MemberType::ComposableNode => {}
            MemberType::Container => {
                participants += processes.get(&dir(m)).copied().unwrap_or(1).max(1);
                observable += 1;
            }
            MemberType::Node if m.package.is_some() => {
                participants += 1;
                observable += 1;
            }
            MemberType::Node => executables += 1,
        }
    }
    let rmw = host.map_or_else(|| "rmw_fastrtps_cpp".to_string(), |h| h.rmw.clone());
    let (limit, limit_reason) = if rmw.contains("cyclonedds") {
        match host.and_then(|h| h.cyclone_participant_index.as_deref()) {
            Some(idx) if idx.eq_ignore_ascii_case("auto") => {
                let max = host
                    .and_then(|h| h.cyclone_max_auto_index)
                    .unwrap_or(CYCLONE_DEFAULT_MAX_AUTO);
                (
                    Some(max + 1),
                    Some(format!(
                        "Cyclone ParticipantIndex auto, MaxAutoParticipantIndex {max}"
                    )),
                )
            }
            _ => (None, None),
        }
    } else if rmw.contains("fastrtps") || rmw.contains("fastdds") {
        (
            Some(FASTDDS_PER_DOMAIN),
            Some("Fast DDS default port mapping, 120 participants a domain".to_string()),
        )
    } else {
        (None, None)
    };
    DiscoveryView {
        rmw,
        domain_id: host.map_or(0, |h| h.domain_id),
        participants,
        executables,
        participants_observable: observable,
        limit,
        limit_reason,
        graph_nodes,
        domain_udp_sockets: host.map_or(0, |h| h.domain_udp_sockets),
        domain_udp_drops_per_min: host.and_then(|h| h.domain_udp_drops_per_min),
        udp_rcvbuf_errors_per_min: host.and_then(|h| h.udp_rcvbuf_errors_per_min),
    }
}

/// Datagrams the domain's sockets dropped while the run started (phase 89
/// D3), read every [`STARTUP_DROP_SAMPLE`] until [`STARTUP_TAIL`] after
/// startup completes, then never again.
#[derive(Debug, Default)]
pub struct StartupDrops {
    last: Option<(std::time::Instant, u64)>,
    done_at: Option<std::time::Instant>,
    closed: bool,
    pub total: u64,
    /// The largest per-minute rate between two samples.
    pub peak_per_min: f64,
}

impl StartupDrops {
    /// Read the domain's sockets if a sample is due. Blocking, a few ms.
    pub fn sample(&mut self, now: std::time::Instant, domain: u32, startup_complete: bool) {
        if self.closed
            || self
                .last
                .is_some_and(|(t, _)| now.duration_since(t) < STARTUP_DROP_SAMPLE)
        {
            return;
        }
        let (_, drops) = super::host::udp_drops(&super::host::domain_ports(domain));
        self.add(now, drops);
        if startup_complete && self.done_at.is_none() {
            self.done_at = Some(now);
        }
        if self
            .done_at
            .is_some_and(|d| now.duration_since(d) >= STARTUP_TAIL)
        {
            self.closed = true;
        }
    }

    fn add(&mut self, now: std::time::Instant, drops: u64) {
        if let Some((t, prev)) = self.last {
            let d = drops.saturating_sub(prev);
            self.total += d;
            let mins = now.duration_since(t).as_secs_f64() / 60.0;
            if mins > 0.0 {
                self.peak_per_min = self.peak_per_min.max(d as f64 / mins);
            }
        }
        self.last = Some((now, drops));
    }
}

/// `discovery_pressure`: one issue for the startup discovery storm, naming
/// its likely cause (the participant count) and the lever (container mode),
/// with the symptoms it shows up as listed as evidence. The plain
/// `dds_udp_drops` it subsumes is removed from `conditions`. Open for the
/// rest of the run once found: it is a fact about how this launch starts.
pub fn discovery_pressure(
    v: &DiscoveryView,
    drops: &StartupDrops,
    conditions: &mut Vec<Condition>,
) {
    if drops.total < STARTUP_DROPS {
        return;
    }
    let mut evidence: std::collections::BTreeMap<&str, usize> = Default::default();
    for c in conditions.iter() {
        if let Some(k) = STORM_SYMPTOMS.iter().find(|k| **k == c.kind) {
            *evidence.entry(k).or_default() += 1;
        }
    }
    conditions.retain(|c| c.kind != "dds_udp_drops");
    let lever = if v.participants_observable < v.participants {
        format!(
            "; under --container-mode observable it would be {} (one per container \
             instead of one per composable)",
            v.participants_observable
        )
    } else {
        String::new()
    };
    let limit = v
        .limit
        .map(|l| format!(" against a limit of {l}"))
        .unwrap_or_default();
    let seen = if evidence.is_empty() {
        String::new()
    } else {
        format!(
            ". Open now and likely the same cause: {}",
            evidence
                .iter()
                .map(|(k, n)| format!("{n} {k}"))
                .collect::<Vec<_>>()
                .join(", ")
        )
    };
    conditions.push(Condition {
        key: "discovery_pressure".to_string(),
        kind: "discovery_pressure",
        severity: Severity::Warning,
        subject: Subject {
            kind: "run",
            name: format!("ROS_DOMAIN_ID {}", v.domain_id),
        },
        title: format!(
            "discovery storm at startup: {} datagrams dropped, {} participants",
            drops.total, v.participants
        ),
        detail: format!(
            "the domain's DDS sockets dropped {} datagrams while the run started (peak \
             {:.0} a minute): every participant announces itself to every other, so \
             the traffic grows with the square of their number. {} participants{limit}\
             {lever}. A larger net.core.rmem_max and middleware socket buffer absorb \
             more of it{seen}",
            drops.total, drops.peak_per_min, v.participants
        ),
        link: None,
    });
}

pub fn discovery_conditions(v: &DiscoveryView) -> Vec<Condition> {
    let mut out = Vec::new();
    let subject = Subject {
        kind: "run",
        name: format!("ROS_DOMAIN_ID {}", v.domain_id),
    };
    let observable_note = if v.participants_observable < v.participants {
        format!(
            "; under --container-mode observable it would be {}",
            v.participants_observable
        )
    } else {
        String::new()
    };
    if let Some(limit) = v.limit {
        let cyclone = v.rmw.contains("cyclonedds");
        if v.participants >= limit || (cyclone && v.participants + v.executables >= limit) {
            out.push(Condition {
                key: "participants_over_limit".to_string(),
                kind: "participants_over_limit",
                severity: if cyclone {
                    Severity::Error
                } else {
                    Severity::Warning
                },
                subject: subject.clone(),
                title: format!(
                    "{} DDS participants against a limit of {limit}",
                    v.participants
                ),
                detail: format!(
                    "{}. {}{observable_note}",
                    v.limit_reason.as_deref().unwrap_or(""),
                    if cyclone {
                        "Past it a process cannot create its participant and fails to start"
                    } else {
                        "Past it, ports run into the next domain's: a run on that domain \
                         would see discovery traffic that is not its own"
                    }
                ),
                link: None,
            });
        }
    }
    if let Some(rate) = v.domain_udp_drops_per_min
        && rate >= DOMAIN_DROPS_PER_MIN
    {
        out.push(Condition {
            key: "dds_udp_drops".to_string(),
            kind: "dds_udp_drops",
            severity: Severity::Warning,
            subject,
            title: format!("the domain's sockets dropped {rate:.0} datagrams a minute"),
            detail: format!(
                "{} sockets on its DDS ports; the kernel dropped datagrams for a full \
                 receive buffer. Discovery bursts ({} participants{observable_note}) or \
                 large messages: raise net.core.rmem_max and the middleware's socket \
                 buffer",
                v.domain_udp_sockets, v.participants
            ),
            link: None,
        });
    }
    out
}

/// Nodes the launch puts on another DDS network or config from the rest
/// (phase 89 D1), from the model's per-node `<env>`, against play_launch's
/// own environment, which every other node inherits.
pub fn dds_env_splits(
    model: &ros_launch_manifest_model::SystemModel,
) -> Vec<ros_launch_resolve::ros::dds_env::DdsEnvSplit> {
    let envs: Vec<(String, Vec<(String, String)>)> = model
        .structure
        .nodes
        .iter()
        .filter(|(_, n)| !n.env.is_empty())
        .map(|(fqn, n)| {
            (
                fqn.clone(),
                n.env
                    .iter()
                    .map(|e| (e.name.clone(), e.value.clone()))
                    .collect(),
            )
        })
        .collect();
    ros_launch_resolve::ros::dds_env::dds_env_splits(
        envs.iter().map(|(n, e)| (n.clone(), e.as_slice())),
        |k| std::env::var(k).ok(),
    )
}

/// `dds_env_split`: a warning when the nodes cannot discover the rest, info
/// for another config file.
pub fn dds_env_conditions(
    splits: &[ros_launch_resolve::ros::dds_env::DdsEnvSplit],
) -> Vec<Condition> {
    splits
        .iter()
        .map(|s| Condition {
            key: format!("dds_env_split:{}={}", s.var, s.value),
            kind: "dds_env_split",
            severity: if s.splits_network {
                Severity::Warning
            } else {
                Severity::Info
            },
            subject: Subject {
                kind: "run",
                name: format!("{}={}", s.var, s.value),
            },
            title: if s.splits_network {
                format!(
                    "{} node{} on another DDS network ({}={})",
                    s.nodes.len(),
                    if s.nodes.len() == 1 { "" } else { "s" },
                    s.var,
                    s.value
                )
            } else {
                format!(
                    "{} node{} with another DDS config ({})",
                    s.nodes.len(),
                    if s.nodes.len() == 1 { "" } else { "s" },
                    s.var
                )
            },
            detail: format!(
                "{}. Set in the launch file's <env>; if it is meant, nothing to do",
                s.message()
            ),
            link: None,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn member(id: &str, ty: MemberType, package: bool) -> MemberSummary {
        MemberSummary {
            id: id.to_string(),
            name: id.to_string(),
            member_type: ty,
            state: MemberState::Running { pid: 1 },
            pid: Some(1),
            package: package.then(|| "pkg".to_string()),
            executable: "x".to_string(),
            namespace: None,
            target_container: None,
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
            output_dir: std::path::PathBuf::from(format!("/log/node/{id}")),
            last_exit: None,
            recent_exits: 0,
        }
    }

    #[test]
    fn isolated_composables_are_participants_and_observable_would_not_be() {
        let members = vec![
            member("n1", MemberType::Node, true),
            member("c1", MemberType::Container, true),
            member("comp", MemberType::ComposableNode, true),
            member("script", MemberType::Node, false),
        ];
        let processes = [("c1".to_string(), 7u32)].into();
        let v = view(&members, &processes, None, None);
        assert_eq!(
            (v.participants, v.participants_observable, v.executables),
            (8, 2, 1)
        );
        assert_eq!(v.limit, Some(120));
    }

    #[test]
    fn cyclone_auto_index_is_an_error_past_its_maximum() {
        let host = super::super::host::HostFacts {
            rmw: "rmw_cyclonedds_cpp".into(),
            cyclone_participant_index: Some("auto".into()),
            ..Default::default()
        };
        let members: Vec<_> = (0..12)
            .map(|i| member(&format!("n{i}"), MemberType::Node, true))
            .collect();
        let v = view(&members, &Default::default(), Some(&host), None);
        assert_eq!(v.limit, Some(10));
        let c = discovery_conditions(&v);
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].severity, Severity::Error);
    }

    #[test]
    fn a_startup_storm_is_one_issue_naming_the_participants_and_the_lever() {
        let v = DiscoveryView {
            participants: 119,
            participants_observable: 49,
            limit: Some(120),
            domain_udp_drops_per_min: Some(5000.0),
            ..Default::default()
        };
        let mut drops = StartupDrops::default();
        let t = std::time::Instant::now();
        drops.add(t, 0);
        drops.add(t + std::time::Duration::from_secs(5), 400_000);
        let mut conds = discovery_conditions(&v);
        conds.push(Condition {
            key: "x".into(),
            kind: "take_deficit",
            severity: Severity::Info,
            subject: Subject {
                kind: "topic",
                name: "/t".into(),
            },
            title: String::new(),
            detail: String::new(),
            link: None,
        });
        discovery_pressure(&v, &drops, &mut conds);
        let kinds: Vec<&str> = conds.iter().map(|c| c.kind).collect();
        assert!(!kinds.contains(&"dds_udp_drops"), "{kinds:?}");
        let p = conds
            .iter()
            .find(|c| c.kind == "discovery_pressure")
            .unwrap();
        assert!(p.title.contains("119 participants"), "{}", p.title);
        assert!(
            p.detail.contains("observable it would be 49"),
            "{}",
            p.detail
        );
        assert!(p.detail.contains("1 take_deficit"), "{}", p.detail);
        assert!(p.detail.contains("peak 4800000 a minute"), "{}", p.detail);
    }

    #[test]
    fn a_few_drops_are_no_storm() {
        let v = DiscoveryView::default();
        let mut drops = StartupDrops::default();
        let t = std::time::Instant::now();
        drops.add(t, 0);
        drops.add(t + std::time::Duration::from_secs(5), 10);
        let mut conds = Vec::new();
        discovery_pressure(&v, &drops, &mut conds);
        assert!(conds.is_empty());
    }

    #[test]
    fn drops_on_the_domains_ports_are_reported() {
        let v = DiscoveryView {
            domain_udp_drops_per_min: Some(50.0),
            domain_udp_sockets: 40,
            participants: 30,
            participants_observable: 10,
            ..Default::default()
        };
        let c = discovery_conditions(&v);
        assert_eq!(c[0].kind, "dds_udp_drops");
        assert!(
            c[0].detail.contains("observable it would be 10"),
            "{}",
            c[0].detail
        );
    }
}
