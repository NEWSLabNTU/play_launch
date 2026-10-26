//! `contract-vs-code` (phase 85 D6): the endpoints a contract declares,
//! compared with the endpoints the code created in a recorded run.
//!
//! The contract is the only source for everything sized or checked from it,
//! and nothing else compared it with the code. Three edits measured on the
//! safety island all checked clean (nano-ros phase 463): a deleted `sub:` row
//! (found at boot as `ExecutorFull`), a phantom one (never found; every pool
//! one longer) and one wired to no topic (dropped silently; I8 now warns).
//! nano-ros closes this for an image with its own entity census. This is
//! the host half: a run made with `--interception on` records every
//! publisher and subscription created, with its node and its type, in
//! `interception/endpoints.tsv`, the file `contract capture` reads.
//!
//! What is compared, and what is not:
//!
//! - Only nodes a contract names. A node no contract mentions has nothing to
//!   disagree with, and reporting its endpoints would turn every partly
//!   contracted system into noise.
//! - Only topics. `endpoints.tsv` records the two topic init hooks, so a
//!   declared service or action has nothing to be compared with, and saying
//!   it is absent would be a claim the run cannot make.
//! - Not QoS, which the file does not carry.
//! - Not the infrastructure topics every node touches (`/rosout`,
//!   `/parameter_events`, ...), the set `contract capture` leaves out.
//!
//! An endpoint only one side has is a warning, not an error: a run sees the
//! endpoints its arguments selected, so a declared endpoint that is
//! conditional in the code, or created late, is absent from a run that did
//! not reach it. A type the two sides disagree on is an error: both
//! endpoints exist and the run proved the declaration wrong.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
};

use eyre::{Result, bail};
use ros_launch_manifest_check::{Diagnostic, Severity};

use super::manifest_loader::{ManifestIndex, resolve_node_fqn};
use crate::verbs::capture::{Direction, EndpointRow, is_infra, load_endpoints, run_paths};

const RULE: &str = "contract-vs-code";

/// One topic endpoint a contract declares, by the node's launch FQN.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct Declared {
    pub(crate) node: String,
    pub(crate) direction: Direction,
    pub(crate) topic: String,
    pub(crate) msg_type: String,
}

/// What the run directory held, for the summary line.
pub struct Against {
    pub endpoints_file: std::path::PathBuf,
    pub rows: usize,
    pub malformed: usize,
    pub diagnostics: Vec<Diagnostic>,
}

/// Read `run`'s `endpoints.tsv` and compare it with the index's contracts.
pub fn check_against(index: &ManifestIndex, run: &Path) -> Result<Against> {
    let (_, idir) = run_paths(run);
    let path = idir.join("endpoints.tsv");
    if !path.is_file() {
        bail!(
            "--against: no endpoints.tsv under {}. Only the interceptor records the \
             endpoints a run created; record the run with `--interception on` (or \
             `interception: {{enabled: true}}` in --config)",
            idir.display()
        );
    }
    let body = std::fs::read_to_string(&path)?;
    let (rows, malformed) = load_endpoints(&body);
    let (declared, contracted) = declared_endpoints(index);
    Ok(Against {
        endpoints_file: path,
        rows: rows.len(),
        malformed,
        diagnostics: compare(&declared, &contracted, &rows),
    })
}

/// Every topic endpoint the contracts declare, and every node a contract
/// names (whether or not it declares an endpoint).
///
/// Endpoints come from `index.topics`, where each ref is already resolved to
/// the node's launch FQN; a topic inferred from launch remaps (phase 76) is
/// no declaration and is skipped. An endpoint under `pub:`/`sub:` that no
/// `topics:` entry wires has no topic to compare; I8 reports it.
pub(crate) fn declared_endpoints(index: &ManifestIndex) -> (Vec<Declared>, BTreeSet<String>) {
    let mut contracted = BTreeSet::new();
    for m in index.manifests.values() {
        for node in m.manifest.nodes.keys() {
            contracted.insert(resolve_node_fqn(index, m.scope_id, &m.ns, node));
        }
    }
    let mut declared = Vec::new();
    for (topic, t) in &index.topics {
        if t.derived_from_remaps {
            continue;
        }
        let sides = [
            (Direction::Pub, &t.publishers),
            (Direction::Sub, &t.subscribers),
        ];
        for (direction, refs) in sides {
            for ep in refs {
                let Some((node, _)) = ep.rsplit_once('/') else {
                    continue;
                };
                contracted.insert(node.to_string());
                declared.push(Declared {
                    node: node.to_string(),
                    direction,
                    topic: topic.clone(),
                    msg_type: t.msg_type.clone(),
                });
            }
        }
    }
    declared.sort();
    declared.dedup();
    (declared, contracted)
}

/// The comparison itself, on plain values so it is testable without a
/// launch tree.
///
/// A row is attributed to a contracted node by its registered FQN first,
/// then by the member play_launch spawned it under: the real name is what a
/// composable in a shared container is told apart by, and the member is the
/// only join for a node whose real name the launch file did not set.
pub(crate) fn compare(
    declared: &[Declared],
    contracted: &BTreeSet<String>,
    rows: &BTreeSet<EndpointRow>,
) -> Vec<Diagnostic> {
    let mut ran: BTreeSet<&str> = BTreeSet::new();
    // (node, direction, topic) -> observed type, empty when unknown.
    let mut observed: BTreeMap<(&str, Direction, &str), &str> = BTreeMap::new();
    for row in rows {
        let node = if contracted.contains(&row.fqn) {
            row.fqn.as_str()
        } else if contracted.contains(&row.member) {
            row.member.as_str()
        } else {
            continue;
        };
        ran.insert(node);
        if is_infra(&row.topic) {
            continue;
        }
        let ty = observed
            .entry((node, row.direction, row.topic.as_str()))
            .or_insert("");
        if ty.is_empty() {
            *ty = row.msg_type.as_str();
        }
    }

    let mut out = Vec::new();
    let declared_nodes: BTreeSet<&str> = declared.iter().map(|d| d.node.as_str()).collect();
    for node in &declared_nodes {
        if !ran.contains(node) {
            out.push(Diagnostic {
                rule_id: RULE.to_string(),
                severity: Severity::Warning,
                message: format!(
                    "{node} created no endpoint in this run, so none of its declared endpoints \
                     could be compared: it did not start, or its process was not intercepted"
                ),
                path: format!("nodes.{node}"),
                span: None,
            });
        }
    }

    let mut seen: BTreeSet<(&str, Direction, &str)> = BTreeSet::new();
    for d in declared {
        if !ran.contains(d.node.as_str()) {
            continue;
        }
        let key = (d.node.as_str(), d.direction, d.topic.as_str());
        seen.insert(key);
        match observed.get(&key) {
            None => out.push(Diagnostic {
                rule_id: RULE.to_string(),
                severity: Severity::Warning,
                message: format!(
                    "the contract declares a {} on '{}' for {}, and the code created none in \
                     this run: a phantom endpoint over-provisions everything sized from the \
                     contract, unless the code creates it only under arguments this run did \
                     not select",
                    d.direction.as_str(),
                    d.topic,
                    d.node
                ),
                path: format!("topics.{}", d.topic),
                span: None,
            }),
            Some(ty) if !ty.is_empty() && !d.msg_type.is_empty() && *ty != d.msg_type => {
                out.push(Diagnostic {
                    rule_id: RULE.to_string(),
                    severity: Severity::Error,
                    message: format!(
                        "the contract types '{}' as {}, and {}'s {} in this run is {}",
                        d.topic,
                        d.msg_type,
                        d.node,
                        d.direction.as_str(),
                        ty
                    ),
                    path: format!("topics.{}", d.topic),
                    span: None,
                });
            }
            Some(_) => {}
        }
    }

    for (key @ (node, direction, topic), ty) in &observed {
        if seen.contains(key) {
            continue;
        }
        let ty = if ty.is_empty() {
            String::new()
        } else {
            format!(" ({ty})")
        };
        out.push(Diagnostic {
            rule_id: RULE.to_string(),
            severity: Severity::Warning,
            message: format!(
                "{node} created a {} on '{topic}'{ty} that no contract declares: an \
                 undeclared endpoint is missing from everything sized or checked from the \
                 contract",
                direction.as_str()
            ),
            path: format!("nodes.{node}"),
            span: None,
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(member: &str, fqn: &str, direction: Direction, topic: &str, ty: &str) -> EndpointRow {
        EndpointRow {
            member: member.into(),
            fqn: fqn.into(),
            direction,
            topic: topic.into(),
            msg_type: ty.into(),
        }
    }

    fn decl(node: &str, direction: Direction, topic: &str, ty: &str) -> Declared {
        Declared {
            node: node.into(),
            direction,
            topic: topic.into(),
            msg_type: ty.into(),
        }
    }

    fn set<const N: usize>(names: [&str; N]) -> BTreeSet<String> {
        names.iter().map(|s| s.to_string()).collect()
    }

    const STR: &str = "std_msgs/msg/String";

    #[test]
    fn a_contract_that_matches_the_run_is_clean() {
        let declared = [
            decl("/talker", Direction::Pub, "/chatter", STR),
            decl("/listener", Direction::Sub, "/chatter", STR),
        ];
        let rows = BTreeSet::from([
            row("/talker", "/talker", Direction::Pub, "/chatter", STR),
            row("/talker", "/talker", Direction::Pub, "/rosout", ""),
            row("/listener", "/listener", Direction::Sub, "/chatter", STR),
        ]);
        let out = compare(&declared, &set(["/talker", "/listener"]), &rows);
        assert!(out.is_empty(), "{out:?}");
    }

    #[test]
    fn each_side_reports_what_only_it_has() {
        // The island's E3a and E3b: a sub the code creates and the contract
        // left out, and a sub the contract declares and the code never makes.
        let declared = [
            decl("/gate", Direction::Sub, "/cmd", STR),
            decl("/gate", Direction::Sub, "/phantom", STR),
        ];
        let rows = BTreeSet::from([
            row("/gate", "/gate", Direction::Sub, "/cmd", STR),
            row("/gate", "/gate", Direction::Sub, "/heartbeat", STR),
        ]);
        let out = compare(&declared, &set(["/gate"]), &rows);
        assert_eq!(out.len(), 2, "{out:?}");
        assert!(out.iter().all(|d| d.severity == Severity::Warning));
        assert!(
            out.iter()
                .any(|d| d.message.contains("'/phantom'") && d.message.contains("created none")),
            "{out:?}"
        );
        assert!(
            out.iter()
                .any(|d| d.message.contains("'/heartbeat'") && d.message.contains("no contract")),
            "{out:?}"
        );
    }

    #[test]
    fn a_type_the_run_contradicts_is_an_error() {
        let declared = [decl("/talker", Direction::Pub, "/chatter", STR)];
        let rows = BTreeSet::from([row(
            "/talker",
            "/talker",
            Direction::Pub,
            "/chatter",
            "std_msgs/msg/Int32",
        )]);
        let out = compare(&declared, &set(["/talker"]), &rows);
        assert_eq!(out.len(), 1, "{out:?}");
        assert_eq!(out[0].severity, Severity::Error);
        assert!(out[0].message.contains("std_msgs/msg/Int32"), "{out:?}");
    }

    #[test]
    fn an_unknown_type_is_not_a_disagreement() {
        let declared = [decl("/talker", Direction::Pub, "/chatter", STR)];
        let rows = BTreeSet::from([row("/talker", "/talker", Direction::Pub, "/chatter", "")]);
        assert!(compare(&declared, &set(["/talker"]), &rows).is_empty());
    }

    #[test]
    fn a_node_no_contract_names_is_not_compared() {
        let declared = [decl("/talker", Direction::Pub, "/chatter", STR)];
        let rows = BTreeSet::from([
            row("/talker", "/talker", Direction::Pub, "/chatter", STR),
            row("/other", "/other", Direction::Pub, "/elsewhere", STR),
        ]);
        assert!(compare(&declared, &set(["/talker"]), &rows).is_empty());
    }

    #[test]
    fn a_node_absent_from_the_run_is_one_finding_not_one_per_endpoint() {
        let declared = [
            decl("/talker", Direction::Pub, "/a", STR),
            decl("/talker", Direction::Pub, "/b", STR),
        ];
        let out = compare(&declared, &set(["/talker"]), &BTreeSet::new());
        assert_eq!(out.len(), 1, "{out:?}");
        assert!(out[0].message.contains("created no endpoint"), "{out:?}");
    }

    #[test]
    fn a_composable_is_matched_by_its_own_name_not_its_containers() {
        // Under one container every composable's rows carry the container's
        // member; the registered name tells them apart.
        let declared = [decl("/a", Direction::Pub, "/x", STR)];
        let rows = BTreeSet::from([
            row("/container", "/a", Direction::Pub, "/x", STR),
            row("/container", "/b", Direction::Pub, "/y", STR),
        ]);
        assert!(compare(&declared, &set(["/a"]), &rows).is_empty());
    }

    #[test]
    fn a_node_the_launch_did_not_name_is_matched_by_its_member() {
        let declared = [decl("/ns/talker-1", Direction::Pub, "/chatter", STR)];
        let rows = BTreeSet::from([row(
            "/ns/talker-1",
            "/ns/talker",
            Direction::Pub,
            "/chatter",
            STR,
        )]);
        assert!(compare(&declared, &set(["/ns/talker-1"]), &rows).is_empty());
    }
}
