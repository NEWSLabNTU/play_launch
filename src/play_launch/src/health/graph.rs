//! The ROS graph against what this run started (`docs/design/health-page.md`
//! §F): nodes the run expects and the graph does not have, nodes on the
//! domain the run did not start, names two nodes share, and subscriptions to
//! a topic nobody publishes when a near name has a publisher — the remap
//! typo.
//!
//! One graph read every [`GRAPH_PERIOD`]: node names, and per topic the nodes
//! on each side. It is synchronous rcl work and runs on `spawn_blocking`.
//!
//! What "started by this run" means: every member the launch named (its FQN
//! is in the coordinator's FQN map, composables once loaded), every node the
//! interceptor saw a process of ours create, and play_launch's own nodes. A
//! node the launch did not name, and an `<executable>`'s nodes, have no name
//! the run can know without interception, so they are accounted for by
//! NAMESPACE: a namespace holding k such members excuses up to k unclaimed
//! graph nodes. Beyond that the excess is reported, as the namespace's, since
//! which of them is the stranger cannot be told. rmw reports no host or pid,
//! so neither can a foreign node's origin.

use std::{
    collections::{HashMap, HashSet},
    time::Duration,
};

use super::{Condition, Link, Severity, Subject};

/// How often the graph is read.
pub const GRAPH_PERIOD: Duration = Duration::from_secs(10);
/// A member up this long whose node the graph does not list is missing.
pub const MISSING_AFTER: Duration = Duration::from_secs(30);
/// Candidates reported per near-miss.
const NEAR_MISS_CANDIDATES: usize = 3;
/// Topics that are someone's by convention, never a typo of another.
const IGNORED_TOPICS: [&str; 2] = ["/rosout", "/parameter_events"];

/// One graph read.
#[derive(Debug, Clone, Default)]
pub struct GraphFacts {
    /// Every node FQN, once per node: a name listed twice is two nodes.
    pub nodes: Vec<String>,
    /// Topic name → node FQNs publishing and subscribing.
    pub topics: Vec<TopicFacts>,
}

#[derive(Debug, Clone, Default)]
pub struct TopicFacts {
    pub name: String,
    pub pubs: Vec<String>,
    pub subs: Vec<String>,
}

fn fqn(ns: &str, name: &str) -> String {
    if ns.ends_with('/') {
        format!("{ns}{name}")
    } else {
        format!("{ns}/{name}")
    }
}

/// Read the graph through play_launch's own node. Blocking.
pub fn read_graph(node: &rclrs::Node) -> eyre::Result<GraphFacts> {
    let _cost = crate::util::self_cost::section(crate::util::self_cost::Subsystem::Graph);
    let nodes = node
        .get_node_names()?
        .into_iter()
        .map(|n| fqn(&n.namespace, &n.name))
        .collect();
    let mut topics = Vec::new();
    for name in node.get_topic_names_and_types()?.into_keys() {
        let names = |infos: Vec<rclrs::TopicEndpointInfo>| -> Vec<String> {
            infos
                .iter()
                .map(|i| fqn(&i.node_namespace, &i.node_name))
                .collect()
        };
        let pubs = names(node.get_publishers_info_by_topic(&name).unwrap_or_default());
        let subs = names(
            node.get_subscriptions_info_by_topic(&name)
                .unwrap_or_default(),
        );
        topics.push(TopicFacts { name, pubs, subs });
    }
    Ok(GraphFacts { nodes, topics })
}

/// A member's part in the comparison.
#[derive(Debug, Clone)]
pub struct Expected {
    pub member_id: String,
    pub member_name: String,
    /// The node's FQN when the run knows it: named nodes, containers, and
    /// loaded composables.
    pub fqn: Option<String>,
    /// For a member whose node name the run cannot know, its namespace.
    pub unnamed_ns: Option<String>,
    /// How long it has been up (running, or loaded), if it is.
    pub up_for: Option<Duration>,
}

/// What this run accounts for, besides its members.
#[derive(Debug, Clone, Default)]
pub struct Ours {
    /// Node FQNs the interceptor saw created in our processes.
    pub announced: HashSet<String>,
    /// play_launch's own node names start with this (`/play_launch_<pid>`).
    pub own_prefix: String,
}

fn ns_of(fqn: &str) -> &str {
    match fqn.rfind('/') {
        Some(0) | None => "/",
        Some(i) => &fqn[..i],
    }
}

fn last_segment(name: &str) -> &str {
    name.rsplit('/').next().unwrap_or(name)
}

/// Hidden by ROS convention (a segment starting with `_`), or an internal
/// node rclcpp's tf2 listener creates in every process that listens to tf.
fn is_infrastructure(fqn: &str) -> bool {
    fqn.split('/').any(|s| s.starts_with('_'))
        || last_segment(fqn).starts_with("transform_listener_impl_")
}

/// The conditions one graph read supports.
pub fn graph_conditions(
    facts: &GraphFacts,
    members: &[Expected],
    ours: &Ours,
    previously_dangling: &HashSet<String>,
) -> Vec<Condition> {
    let mut out = Vec::new();
    let mut in_graph: HashMap<&str, usize> = HashMap::new();
    for n in &facts.nodes {
        *in_graph.entry(n.as_str()).or_default() += 1;
    }

    // Missing: a named member up for a while whose node is not listed.
    for m in members {
        let (Some(fqn), Some(up)) = (&m.fqn, m.up_for) else {
            continue;
        };
        if up >= MISSING_AFTER && !in_graph.contains_key(fqn.as_str()) {
            out.push(Condition {
                key: format!("missing_node:{}", m.member_id),
                kind: "missing_node",
                severity: Severity::Error,
                subject: Subject {
                    kind: "node",
                    name: fqn.clone(),
                },
                title: format!("up for {} s but not in the ROS graph", up.as_secs()),
                detail: format!(
                    "{} is running, yet no node named {fqn} is discoverable: it may have \
                     created its node under another name, failed before creating it, or be \
                     on another ROS_DOMAIN_ID",
                    m.member_name
                ),
                link: Some(Link {
                    view: "nodes",
                    target: Some(m.member_id.clone()),
                }),
            });
        }
    }

    // Duplicates: one name, several nodes. ROS 2 allows it and every
    // service, parameter and graph tool then talks to one of them at random.
    for (name, &n) in &in_graph {
        if n > 1 && !is_infrastructure(name) {
            out.push(Condition {
                key: format!("duplicate_node:{name}"),
                kind: "duplicate_node",
                severity: Severity::Warning,
                subject: Subject {
                    kind: "node",
                    name: name.to_string(),
                },
                title: format!("{n} nodes share this name"),
                detail: "services, parameters and introspection reach only one of them; \
                         two launches on one ROS_DOMAIN_ID, or a launch file that names two \
                         nodes alike"
                    .to_string(),
                link: Some(Link {
                    view: "graph",
                    target: Some(name.to_string()),
                }),
            });
        }
    }

    // Foreign: graph nodes nothing of ours accounts for.
    let claimed: HashSet<&str> = members
        .iter()
        .filter_map(|m| m.fqn.as_deref())
        .chain(ours.announced.iter().map(String::as_str))
        .collect();
    let mut slack: HashMap<&str, usize> = HashMap::new();
    for m in members {
        if let Some(ns) = &m.unnamed_ns {
            *slack.entry(ns.as_str()).or_default() += 1;
        }
    }
    let mut unclaimed: HashMap<&str, Vec<&str>> = HashMap::new();
    for name in in_graph.keys() {
        if claimed.contains(name)
            || is_infrastructure(name)
            || (!ours.own_prefix.is_empty() && name.starts_with(ours.own_prefix.as_str()))
        {
            continue;
        }
        unclaimed.entry(ns_of(name)).or_default().push(name);
    }
    // Announced nodes of an unnamed member use up its namespace's slack.
    for name in &ours.announced {
        if !members
            .iter()
            .any(|m| m.fqn.as_deref() == Some(name.as_str()))
            && let Some(s) = slack.get_mut(ns_of(name))
        {
            *s = s.saturating_sub(1);
        }
    }
    for (ns, mut names) in unclaimed {
        let excused = slack.get(ns).copied().unwrap_or(0);
        if names.len() <= excused {
            continue;
        }
        names.sort();
        let shown = names
            .iter()
            .take(10)
            .copied()
            .collect::<Vec<_>>()
            .join(", ");
        let detail = if excused == 0 {
            format!(
                "not started by this run: {shown}. Another launch or a `ros2 run` on the same \
                 ROS_DOMAIN_ID; rmw does not say which host it runs on"
            )
        } else {
            format!(
                "{} nodes in {ns} where this run accounts for {excused} it cannot name (nodes \
                 the launch did not name, or an executable's): {shown}. Some are not this \
                 run's; run with --interception on to name every node of this run",
                names.len()
            )
        };
        out.push(Condition {
            key: format!("foreign_node:{ns}"),
            kind: "foreign_node",
            severity: Severity::Warning,
            subject: Subject {
                kind: "node",
                name: if names.len() == 1 && excused == 0 {
                    names[0].to_string()
                } else {
                    format!("{ns} ({} unaccounted)", names.len() - excused)
                },
            },
            title: "a node this run did not start is on the domain".to_string(),
            detail,
            link: Some(Link {
                view: "graph",
                target: Some(names[0].to_string()),
            }),
        });
    }

    // A subscription is judged when its node is ours, or may be: an
    // unclaimed node in a namespace where the run has nodes it cannot name.
    let maybe_ours = |n: &str| claimed.contains(n) || slack.get(ns_of(n)).is_some_and(|&k| k > 0);
    out.extend(near_misses(facts, &maybe_ours, previously_dangling));
    out
}

/// Topics of ours that have subscribers and no publisher.
pub fn dangling_subscribed(facts: &GraphFacts) -> HashSet<String> {
    facts
        .topics
        .iter()
        .filter(|t| t.pubs.is_empty() && !t.subs.is_empty())
        .map(|t| t.name.clone())
        .collect()
}

/// A subscription of ours whose topic has no publisher, dangling in two reads
/// in a row (discovery settles), with a published topic a typo away.
///
/// Measured on Autoware's planning simulator, which subscribes ~50 topics
/// nothing publishes in that configuration (optional inputs, per-module
/// `processing_time_ms`): any looser rule — one segment differing entirely,
/// or a leaf shared with any published topic — named a "candidate" for most
/// of them, since `pointcloud`, `status` and `processing_time_ms` are leaves
/// of dozens of topics. So a candidate is either one segment off by a typo,
/// or the ONLY published topic with that leaf and one nobody subscribes to
/// (the other half of a broken pair).
fn near_misses(
    facts: &GraphFacts,
    ours: &dyn Fn(&str) -> bool,
    previously_dangling: &HashSet<String>,
) -> Vec<Condition> {
    let published: Vec<&TopicFacts> = facts
        .topics
        .iter()
        .filter(|t| !t.pubs.is_empty() && !skipped_topic(&t.name))
        .collect();
    let mut leaves: HashMap<&str, usize> = HashMap::new();
    for t in &published {
        *leaves.entry(last_segment(&t.name)).or_default() += 1;
    }
    let mut out = Vec::new();
    for t in &facts.topics {
        if !t.pubs.is_empty() || skipped_topic(&t.name) || !previously_dangling.contains(&t.name) {
            continue;
        }
        let our_subs: Vec<&str> = t
            .subs
            .iter()
            .map(String::as_str)
            .filter(|s| ours(s))
            .collect();
        if our_subs.is_empty() {
            continue;
        }
        let mut candidates: Vec<&str> = published
            .iter()
            .filter(|c| match near_miss(&t.name, &c.name) {
                Some(Near::Typo) => true,
                Some(Near::SameLeaf) => {
                    leaves.get(last_segment(&c.name)) == Some(&1) && c.subs.is_empty()
                }
                None => false,
            })
            .map(|c| c.name.as_str())
            .collect();
        if candidates.is_empty() {
            continue;
        }
        candidates.sort();
        let list = candidates
            .iter()
            .take(NEAR_MISS_CANDIDATES)
            .map(|n| format!("`{n}`"))
            .collect::<Vec<_>>()
            .join(", ");
        out.push(Condition {
            key: format!("near_miss_topic:{}", t.name),
            kind: "near_miss_topic",
            severity: Severity::Warning,
            subject: Subject {
                kind: "topic",
                name: t.name.clone(),
            },
            title: "subscribed, never published; a similar name has a publisher".to_string(),
            detail: format!(
                "{} subscribe{} `{}`, which nothing publishes; published: {list}. A remap \
                 or namespace typo?",
                our_subs.join(", "),
                if our_subs.len() == 1 { "s" } else { "" },
                t.name
            ),
            link: Some(Link {
                view: "graph",
                target: Some(t.name.clone()),
            }),
        });
    }
    out
}

fn skipped_topic(name: &str) -> bool {
    IGNORED_TOPICS.contains(&name) || name.split('/').any(|s| s.starts_with('_'))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Near {
    /// Same depth, one segment differs by a typo ([`is_typo`]).
    Typo,
    /// Same leaf, another namespace.
    SameLeaf,
}

fn near_miss(a: &str, b: &str) -> Option<Near> {
    if a == b {
        return None;
    }
    let sa: Vec<&str> = a.split('/').filter(|s| !s.is_empty()).collect();
    let sb: Vec<&str> = b.split('/').filter(|s| !s.is_empty()).collect();
    if sa.len() == sb.len() {
        let diff: Vec<usize> = (0..sa.len()).filter(|&i| sa[i] != sb[i]).collect();
        if diff.len() == 1 && is_typo(sa[diff[0]], sb[diff[0]]) {
            return Some(Near::Typo);
        }
    }
    (sa.last() == sb.last() && sa.last().is_some_and(|l| l.len() >= 4)).then_some(Near::SameLeaf)
}

/// Edit distance at most 2 and at most a third of the shorter word: `points`
/// and `pointz` are a typo, `a` and `c` are two names. One extending the
/// other (`initialpose`, `initialpose3d`) is a sibling, not a typo.
fn is_typo(a: &str, b: &str) -> bool {
    if a.starts_with(b) || b.starts_with(a) {
        return false;
    }
    let d = edit_distance(a, b);
    d <= 2 && d * 3 <= a.chars().count().min(b.chars().count())
}

fn edit_distance(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    for (i, ca) in a.iter().enumerate() {
        let mut cur = vec![i + 1; b.len() + 1];
        for (j, cb) in b.iter().enumerate() {
            cur[j + 1] = (prev[j] + usize::from(ca != cb))
                .min(prev[j + 1] + 1)
                .min(cur[j] + 1);
        }
        prev = cur;
    }
    prev[b.len()]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn topic(name: &str, pubs: &[&str], subs: &[&str]) -> TopicFacts {
        TopicFacts {
            name: name.to_string(),
            pubs: pubs.iter().map(|s| s.to_string()).collect(),
            subs: subs.iter().map(|s| s.to_string()).collect(),
        }
    }

    fn named(id: &str, fqn: &str, up_s: Option<u64>) -> Expected {
        Expected {
            member_id: id.to_string(),
            member_name: id.to_string(),
            fqn: Some(fqn.to_string()),
            unnamed_ns: None,
            up_for: up_s.map(Duration::from_secs),
        }
    }

    fn unnamed(id: &str, ns: &str) -> Expected {
        Expected {
            member_id: id.to_string(),
            member_name: id.to_string(),
            fqn: None,
            unnamed_ns: Some(ns.to_string()),
            up_for: Some(Duration::from_secs(60)),
        }
    }

    fn kinds(c: &[Condition], kind: &str) -> Vec<String> {
        c.iter()
            .filter(|c| c.kind == kind)
            .map(|c| c.subject.name.clone())
            .collect()
    }

    fn graph(nodes: &[&str]) -> GraphFacts {
        GraphFacts {
            nodes: nodes.iter().map(|s| s.to_string()).collect(),
            topics: Vec::new(),
        }
    }

    #[test]
    fn a_named_member_up_without_its_node_is_missing_after_a_grace() {
        let g = graph(&["/a"]);
        let members = [
            named("a", "/a", Some(100)),
            named("b", "/b", Some(100)),
            named("young", "/young", Some(5)),
            named("down", "/down", None),
        ];
        let c = graph_conditions(&g, &members, &Ours::default(), &HashSet::new());
        assert_eq!(kinds(&c, "missing_node"), ["/b"]);
    }

    #[test]
    fn a_node_nothing_accounts_for_is_foreign_and_ours_are_not() {
        let g = graph(&[
            "/a",
            "/stranger",
            "/play_launch_42",
            "/play_launch_42_diagnostics",
            "/_ros2cli_daemon_0_abc",
            "/x/transform_listener_impl_5f3a",
            "/sensing/seen_by_interception",
        ]);
        let ours = Ours {
            announced: ["/sensing/seen_by_interception".to_string()].into(),
            own_prefix: "/play_launch_42".to_string(),
        };
        let c = graph_conditions(&g, &[named("a", "/a", Some(100))], &ours, &HashSet::new());
        assert_eq!(kinds(&c, "foreign_node"), ["/stranger"]);
    }

    #[test]
    fn unnamed_members_excuse_as_many_nodes_in_their_namespace() {
        // Two unnamed members in /s; three unclaimed nodes there.
        let g = graph(&["/s/n1", "/s/n2", "/s/n3", "/t/n4"]);
        let members = [
            unnamed("u1", "/s"),
            unnamed("u2", "/s"),
            unnamed("u3", "/t"),
        ];
        let c = graph_conditions(&g, &members, &Ours::default(), &HashSet::new());
        let foreign: Vec<&Condition> = c.iter().filter(|c| c.kind == "foreign_node").collect();
        assert_eq!(foreign.len(), 1, "{foreign:?}");
        assert_eq!(foreign[0].subject.name, "/s (1 unaccounted)");
        assert!(foreign[0].detail.contains("--interception on"));
    }

    #[test]
    fn two_nodes_with_one_name_are_reported() {
        let g = graph(&["/a", "/a", "/b"]);
        let c = graph_conditions(
            &g,
            &[named("a", "/a", Some(1)), named("b", "/b", Some(1))],
            &Ours::default(),
            &HashSet::new(),
        );
        assert_eq!(kinds(&c, "duplicate_node"), ["/a"]);
    }

    #[test]
    fn a_remap_typo_is_a_near_miss_once_it_has_dangled_twice() {
        let g = GraphFacts {
            nodes: vec!["/lidar".into(), "/filter".into()],
            topics: vec![
                topic("/sensing/lidar/points", &["/lidar"], &[]),
                topic("/sensing/lidar/pointz", &[], &["/filter"]),
                topic("/unrelated/thing", &["/lidar"], &[]),
                topic("/optional/input", &[], &["/filter"]),
            ],
        };
        let members = [
            named("lidar", "/lidar", Some(1)),
            named("filter", "/filter", Some(1)),
        ];
        let first = graph_conditions(&g, &members, &Ours::default(), &HashSet::new());
        assert!(kinds(&first, "near_miss_topic").is_empty(), "first read");
        let dangling = dangling_subscribed(&g);
        let c = graph_conditions(&g, &members, &Ours::default(), &dangling);
        assert_eq!(kinds(&c, "near_miss_topic"), ["/sensing/lidar/pointz"]);
        let d = &c
            .iter()
            .find(|c| c.kind == "near_miss_topic")
            .unwrap()
            .detail;
        assert!(d.contains("`/sensing/lidar/points`"), "{d}");
        assert!(!d.contains("/unrelated"), "{d}");
    }

    #[test]
    fn a_near_miss_is_a_typo_or_a_moved_namespace() {
        use Near::*;
        assert_eq!(near_miss("/a/b/points", "/a/b/pointz"), Some(Typo));
        assert_eq!(near_miss("/a/lidar/points", "/a/lidr/points"), Some(Typo));
        assert_eq!(near_miss("/a/b/points", "/c/points"), Some(SameLeaf));
        assert_eq!(
            near_miss("/a/b/points", "/a/x/points"),
            Some(SameLeaf),
            "b, x: no typo"
        );
        assert_eq!(near_miss("/a/b/points", "/a/b/odometry"), None);
        assert_eq!(
            near_miss("/initialpose", "/initialpose3d"),
            None,
            "a sibling"
        );
        assert_eq!(near_miss("/a/bb", "/c/d/bb"), None, "short leaf");
        assert_eq!(edit_distance("kitten", "sitting"), 3);
    }

    #[test]
    fn a_shared_leaf_names_a_candidate_only_when_it_is_unique_and_unconsumed() {
        let members = [named("me", "/me", Some(1)), named("p", "/p", Some(1))];
        let run = |topics: Vec<TopicFacts>| {
            let g = GraphFacts {
                nodes: vec!["/me".into(), "/p".into()],
                topics,
            };
            let d = dangling_subscribed(&g);
            graph_conditions(&g, &members, &Ours::default(), &d)
        };
        // The leaf of dozens of topics names nothing.
        let c = run(vec![
            topic("/x/status", &[], &["/me"]),
            topic("/a/status", &["/p"], &[]),
            topic("/b/status", &["/p"], &[]),
        ]);
        assert!(kinds(&c, "near_miss_topic").is_empty(), "{c:?}");
        // One published topic with that leaf, consumed elsewhere: not a pair.
        let c = run(vec![
            topic("/x/status", &[], &["/me"]),
            topic("/a/status", &["/p"], &["/p"]),
        ]);
        assert!(kinds(&c, "near_miss_topic").is_empty(), "{c:?}");
        // The only one, and nobody takes it: the other half of a broken pair.
        let c = run(vec![
            topic("/x/status", &[], &["/me"]),
            topic("/a/status", &["/p"], &[]),
        ]);
        assert_eq!(kinds(&c, "near_miss_topic"), ["/x/status"]);
    }

    #[test]
    fn a_subscription_that_is_not_ours_is_not_judged() {
        let g = GraphFacts {
            nodes: vec!["/lidar".into(), "/someone".into()],
            topics: vec![
                topic("/points", &["/lidar"], &[]),
                topic("/pointz", &[], &["/someone"]),
            ],
        };
        let dangling = dangling_subscribed(&g);
        let c = graph_conditions(
            &g,
            &[named("lidar", "/lidar", Some(1))],
            &Ours::default(),
            &dangling,
        );
        assert!(kinds(&c, "near_miss_topic").is_empty());
    }
}
