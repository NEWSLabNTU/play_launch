//! Member identity — collision-proof keys for every member registry.
//!
//! Design: docs/design/member-identity.md (phase-50). Every registry that
//! used to key on a bare display name (silently overwriting collisions —
//! issue 0001) now keys on the canonical id produced here:
//!
//! ```text
//! node:/ns/node_name          regular node
//! container:/ns/name          container
//! composable:/ns/node_name    composable node
//! …#2                         ordinal suffix for TRUE duplicates
//! ```
//!
//! `fqn_join` is the ONE namespace-join rule — the same rule
//! `ros/model_builder.rs` uses for the emitted SystemModel, so the model
//! and the runtime can never disagree.

use std::collections::HashMap;

/// Kind of member — part of the canonical id so a regular node, a
/// container, and a composable with the same FQN never collide.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MemberKind {
    Node,
    Container,
    Composable,
}

impl MemberKind {
    pub fn prefix(self) -> &'static str {
        match self {
            MemberKind::Node => "node",
            MemberKind::Container => "container",
            MemberKind::Composable => "composable",
        }
    }
}

/// Join a namespace and a name into an FQN (`/ns/name`), root-safe.
///
/// The single owner of the join rules: trailing-slash trim, empty/root
/// namespace → `/name`, missing leading slash prefixed.
pub fn fqn_join(ns: &str, name: &str) -> String {
    let ns = ns.trim_end_matches('/');
    if ns.is_empty() {
        format!("/{name}")
    } else if ns.starts_with('/') {
        format!("{ns}/{name}")
    } else {
        format!("/{ns}/{name}")
    }
}

/// Allocates canonical member ids, deduplicating TRUE duplicates (same
/// kind + same FQN) with an ordinal suffix. Used by the coordinator
/// builder while registering members; the member set is fixed afterwards.
#[derive(Default)]
pub struct IdAllocator {
    seen: HashMap<String, u16>,
}

impl IdAllocator {
    pub fn new() -> Self {
        Self::default()
    }

    /// Canonical id for a member. First occurrence: `kind:/ns/name`;
    /// duplicates: `kind:/ns/name#2`, `#3`, …
    pub fn allocate(&mut self, kind: MemberKind, namespace: Option<&str>, name: &str) -> String {
        let fqn = fqn_join(namespace.unwrap_or("/"), name);
        let base = format!("{}:{}", kind.prefix(), fqn);
        let n = self.seen.entry(base.clone()).or_insert(0);
        *n += 1;
        if *n == 1 { base } else { format!("{base}#{n}") }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fqn_join_rules() {
        assert_eq!(fqn_join("/", "talker"), "/talker");
        assert_eq!(fqn_join("", "talker"), "/talker");
        assert_eq!(fqn_join("/robot1", "camera"), "/robot1/camera");
        assert_eq!(fqn_join("/robot1/", "camera"), "/robot1/camera");
        assert_eq!(fqn_join("robot1", "camera"), "/robot1/camera");
    }

    #[test]
    fn cross_namespace_no_collision() {
        let mut a = IdAllocator::new();
        let x = a.allocate(MemberKind::Node, Some("/robot1"), "camera");
        let y = a.allocate(MemberKind::Node, Some("/robot2"), "camera");
        assert_ne!(x, y);
        assert_eq!(x, "node:/robot1/camera");
        assert_eq!(y, "node:/robot2/camera");
    }

    #[test]
    fn true_duplicates_get_ordinals() {
        // name=null nodes falling back to a shared exec_name (issue 0001
        // cause 2), and the literal "unknown" bucket.
        let mut a = IdAllocator::new();
        assert_eq!(
            a.allocate(MemberKind::Node, None, "lidar_driver"),
            "node:/lidar_driver"
        );
        assert_eq!(
            a.allocate(MemberKind::Node, None, "lidar_driver"),
            "node:/lidar_driver#2"
        );
        assert_eq!(
            a.allocate(MemberKind::Node, None, "unknown"),
            "node:/unknown"
        );
        assert_eq!(
            a.allocate(MemberKind::Node, None, "unknown"),
            "node:/unknown#2"
        );
    }

    #[test]
    fn cross_kind_no_collision() {
        let mut a = IdAllocator::new();
        let n = a.allocate(MemberKind::Node, Some("/x"), "pipeline");
        let c = a.allocate(MemberKind::Container, Some("/x"), "pipeline");
        let l = a.allocate(MemberKind::Composable, Some("/x"), "pipeline");
        assert!(n != c && c != l && n != l);
    }

    #[test]
    fn composable_same_name_two_containers() {
        let mut a = IdAllocator::new();
        let one = a.allocate(MemberKind::Composable, Some("/left"), "image_proc");
        let two = a.allocate(MemberKind::Composable, Some("/right"), "image_proc");
        assert_ne!(one, two);
    }
}
