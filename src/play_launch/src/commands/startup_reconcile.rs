//! Issue #0023 — declared against loaded, per container, at startup-complete.
//!
//! The golf cart bundle carried `composable_node_count: 16` in one container's
//! `metadata.json` and ten `Accepted load request` lines in its `err`, and
//! nothing compared the two. `HealthSummary` sums composables across the whole
//! launch, so `78/84 loaded` was visible only as a total that nobody read
//! against the declaration, and never as *which* container lost *which*
//! nodes. This module makes the comparison the summary does not: for every
//! container, the composables the launch declared for it, the ones that
//! reached `Loaded`, and the FQN and state of each one that did not.
//!
//! Pure over the member list, so the arithmetic is testable without an actor
//! system and cannot disagree with the web UI's view of the same members.

use crate::member_actor::model::{MemberState, MemberSummary, MemberType};
use std::collections::BTreeMap;

/// A composable that was declared for a container and is not `Loaded`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MissingComposable {
    /// The composable's FQN (`/ns/name`, with the canonical id's `#N`
    /// ordinal kept if it has one).
    pub fqn: String,
    /// Where it ended up instead, in the words the log should use.
    pub state: String,
}

/// One container's reconciliation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ContainerShortfall {
    /// The container as the composables name it (`target_container`).
    pub container: String,
    pub declared: usize,
    pub loaded: usize,
    /// Sorted by FQN.
    pub missing: Vec<MissingComposable>,
}

impl ContainerShortfall {
    /// One line naming the container, the count, and every missing FQN.
    pub fn describe(&self) -> String {
        let names: Vec<String> = self
            .missing
            .iter()
            .map(|m| format!("{} ({})", m.fqn, m.state))
            .collect();
        format!(
            "{}: {}/{} composables loaded — {} MISSING: {}",
            self.container,
            self.loaded,
            self.declared,
            self.missing.len(),
            names.join(", ")
        )
    }
}

/// The FQN half of a canonical member id (`composable:/ns/name#2` →
/// `/ns/name#2`). An id with no kind prefix is returned whole.
fn fqn_of(id: &str) -> &str {
    id.split_once(':').map(|(_, fqn)| fqn).unwrap_or(id)
}

fn describe_state(state: &MemberState) -> String {
    match state {
        MemberState::Loaded { .. } => "loaded".to_string(),
        MemberState::Failed { error } => format!("failed: {error}"),
        MemberState::Loading => "still loading".to_string(),
        MemberState::Unloading => "unloading".to_string(),
        MemberState::Unloaded => "never loaded".to_string(),
        MemberState::Blocked { reason } => format!("blocked: {reason:?}"),
        MemberState::Pending => "pending".to_string(),
        // Process states never describe a composable; spelled out rather
        // than wildcarded so a new variant is a compile error here.
        MemberState::Running { .. } | MemberState::Respawning { .. } | MemberState::Stopped => {
            format!("{state:?}")
        }
    }
}

/// Every container with at least one declared composable that is not
/// `Loaded`, sorted by container name. Empty means every declared composable
/// loaded — the only case in which the launch may say so.
pub fn composable_shortfall(members: &[MemberSummary]) -> Vec<ContainerShortfall> {
    let mut by_container: BTreeMap<String, ContainerShortfall> = BTreeMap::new();
    for m in members {
        if m.member_type != MemberType::ComposableNode {
            continue;
        }
        // A composable whose container was never matched (phase-50 registers
        // it visibly as Blocked{ContainerNotFound}) has no container to be
        // grouped under; it is reported under that name so it is not lost
        // from the count a second time.
        let container = m
            .target_container
            .clone()
            .unwrap_or_else(|| "<no container>".to_string());
        let entry = by_container
            .entry(container.clone())
            .or_insert_with(|| ContainerShortfall {
                container,
                declared: 0,
                loaded: 0,
                missing: Vec::new(),
            });
        entry.declared += 1;
        if matches!(m.state, MemberState::Loaded { .. }) {
            entry.loaded += 1;
        } else {
            entry.missing.push(MissingComposable {
                fqn: fqn_of(&m.id).to_string(),
                state: describe_state(&m.state),
            });
        }
    }
    by_container
        .into_values()
        .filter(|c| !c.missing.is_empty())
        .map(|mut c| {
            c.missing.sort_by(|a, b| a.fqn.cmp(&b.fqn));
            c
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::member_actor::model::BlockReason;

    fn composable(id: &str, container: &str, state: MemberState) -> MemberSummary {
        MemberSummary {
            id: id.to_string(),
            name: fqn_of(id).rsplit('/').next().unwrap().to_string(),
            member_type: MemberType::ComposableNode,
            state,
            pid: None,
            package: Some("pkg".into()),
            executable: "plugin".into(),
            namespace: None,
            target_container: Some(container.to_string()),
            is_container: false,
            exec_name: None,
            node_name: None,
            stderr_last_modified: None,
            stderr_size: 0,
            stderr_preview: None,
            respawn_enabled: None,
            respawn_delay: None,
            auto_load: Some(true),
            output_dir: std::path::PathBuf::from("/nonexistent"),
        }
    }

    fn container(id: &str) -> MemberSummary {
        let mut m = composable(id, "", MemberState::Running { pid: 1 });
        m.member_type = MemberType::Container;
        m.is_container = true;
        m.target_container = None;
        m
    }

    /// The golf cart shape in miniature: one container short, one whole.
    #[test]
    fn names_the_container_and_every_missing_fqn() {
        let members = vec![
            container("container:/system/component_state_monitor/container"),
            container("container:/pointcloud_container"),
            composable(
                "composable:/system/component_state_monitor/component",
                "/system/component_state_monitor/container",
                MemberState::Failed {
                    error: "lost".into(),
                },
            ),
            composable(
                "composable:/system/topic_state_monitor_vector_map",
                "/system/component_state_monitor/container",
                MemberState::Failed {
                    error: "lost".into(),
                },
            ),
            composable(
                "composable:/system/topic_state_monitor_initialpose3d",
                "/system/component_state_monitor/container",
                MemberState::Loaded { unique_id: 3 },
            ),
            composable(
                "composable:/sensing/crop_box",
                "/pointcloud_container",
                MemberState::Loaded { unique_id: 1 },
            ),
        ];

        let shortfall = composable_shortfall(&members);
        assert_eq!(shortfall.len(), 1, "{shortfall:?}");
        let c = &shortfall[0];
        assert_eq!(c.container, "/system/component_state_monitor/container");
        assert_eq!((c.declared, c.loaded), (3, 1));
        assert_eq!(
            c.missing.iter().map(|m| m.fqn.as_str()).collect::<Vec<_>>(),
            vec![
                "/system/component_state_monitor/component",
                "/system/topic_state_monitor_vector_map"
            ]
        );
        let line = c.describe();
        assert!(
            line.starts_with(
                "/system/component_state_monitor/container: 1/3 composables loaded — 2 MISSING: "
            ),
            "{line}"
        );
        assert!(
            line.contains("/system/topic_state_monitor_vector_map (failed: lost)"),
            "{line}"
        );
    }

    /// Everything loaded: nothing to report, so the "all nodes ready" line
    /// stays exactly what it was.
    #[test]
    fn no_shortfall_when_every_declared_composable_is_loaded() {
        let members = vec![
            container("container:/c"),
            composable("composable:/a", "/c", MemberState::Loaded { unique_id: 1 }),
            composable("composable:/b", "/c", MemberState::Loaded { unique_id: 2 }),
        ];
        assert!(composable_shortfall(&members).is_empty());
    }

    /// The comparison is against the DECLARATION, not against the failure
    /// count: a composable that is neither loaded nor failed — blocked because
    /// its container never came up, or still loading — is missing too.
    /// `HealthSummary` counts those as nothing at all.
    #[test]
    fn a_composable_that_is_not_loaded_is_missing_whatever_its_state() {
        let members = vec![
            composable(
                "composable:/blocked",
                "/c",
                MemberState::Blocked {
                    reason: BlockReason::ContainerFailed,
                },
            ),
            composable("composable:/loading", "/c", MemberState::Loading),
            composable("composable:/unloaded", "/c", MemberState::Unloaded),
        ];
        let shortfall = composable_shortfall(&members);
        assert_eq!(shortfall.len(), 1);
        let states: Vec<&str> = shortfall[0]
            .missing
            .iter()
            .map(|m| m.state.as_str())
            .collect();
        assert_eq!(
            states,
            vec!["blocked: ContainerFailed", "still loading", "never loaded"]
        );
    }

    /// Regular nodes never enter the arithmetic, and an ordinal-suffixed id
    /// keeps its suffix so two same-named composables stay distinguishable.
    #[test]
    fn ignores_non_composables_and_keeps_the_ordinal() {
        let mut node = composable(
            "node:/talker",
            "/c",
            MemberState::Failed { error: "x".into() },
        );
        node.member_type = MemberType::Node;
        node.target_container = None;
        let members = vec![
            node,
            composable(
                "composable:/dup#2",
                "/c",
                MemberState::Failed { error: "x".into() },
            ),
        ];
        let shortfall = composable_shortfall(&members);
        assert_eq!(shortfall.len(), 1);
        assert_eq!(shortfall[0].declared, 1);
        assert_eq!(shortfall[0].missing[0].fqn, "/dup#2");
    }
}
