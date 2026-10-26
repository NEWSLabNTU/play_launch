//! `acknowledged:` (phase 85 D8): a finding the author has read and keeps.
//!
//! A contract's root `acknowledged:` list names findings by rule id and the
//! dotted path the diagnostic reports (`{ rule, at, reason }`, rlm v0.2.3).
//! [`apply`] matches the list against the diagnostics `check` is about to
//! print:
//!
//! - a WARNING or INFO that matches an entry leaves the diagnostic lists and
//!   moves to [`ManifestIndex::acknowledged`], carrying the reason. It is
//!   printed as `ack[<rule>]` and counted apart, so it no longer counts as a
//!   warning;
//! - an ERROR that matches stays where it is, and the entry is reported as
//!   `error[ack-error]`: an error cannot be acknowledged;
//! - an entry that matches nothing is `warning[ack-stale]`, so an
//!   acknowledgement cannot outlive its finding.
//!
//! **Which contract's entries a diagnostic is matched against.** A per-scope
//! diagnostic (`ResolvedManifest::diagnostics`, from rlm's own rules) belongs
//! to one contract file, and only that file's entries match it. A cross-scope
//! diagnostic (`ManifestIndex::merge_diagnostics`) names no scope -- `Diagnostic`
//! has no field for one, and the loader's rules compute over the merged
//! graph -- so it is matched against every contract's entries, the first
//! (lowest scope id) that matches winning. The two `reaction-unguarded`
//! warnings that motivated this are cross-scope; their paths are the
//! contract's own (`hazards.<h>.reaction`), so two contracts that both name a
//! hazard `h` would match each other's entry. That is the price of a
//! diagnostic with no provenance, and it errs toward acknowledging, never
//! toward hiding an error.
//!
//! Matching is exact on both the rule id and the path: a path is the key the
//! rule names, never a prefix of it.
//!
//! Called by `check`, AFTER every step that adds or re-grades diagnostics
//! (`--strict`, `--against`), so an entry is judged against the verdict
//! that is printed; and by `model::build_checked_model` right after it
//! loads its own index, so `resolve` and `launch` embed an acknowledged
//! finding as acknowledged (`model_builder::acknowledged_line`). It is not
//! idempotent: a second call would find the acknowledged diagnostics gone
//! and report every entry stale, so each index is matched exactly once.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};

use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::AckDecl;

use super::manifest_loader::ManifestIndex;

/// A diagnostic an `acknowledged:` entry matched.
#[derive(Debug, Clone)]
pub struct Acknowledged {
    /// The diagnostic as the rule emitted it (severity Warning or Info).
    pub diagnostic: Diagnostic,
    /// The entry's `reason`.
    pub reason: String,
    /// The contract file whose entry matched.
    pub contract_path: PathBuf,
    /// The entry's index in that file's `acknowledged:` list.
    pub entry: usize,
    /// The scope whose own diagnostics it was; `None` for a cross-scope
    /// diagnostic.
    pub scope_id: Option<usize>,
}

/// One entry, located: the first scope bound to its contract, the file, its
/// index.
struct Entry<'a> {
    scope_id: usize,
    contract_path: PathBuf,
    index: usize,
    decl: &'a AckDecl,
}

impl Entry<'_> {
    fn key(&self) -> (PathBuf, usize) {
        (self.contract_path.clone(), self.index)
    }
    fn matches(&self, d: &Diagnostic) -> bool {
        d.rule_id == self.decl.rule && d.path == self.decl.at
    }
}

/// Match every contract's `acknowledged:` entries against the index's
/// diagnostics. See the module documentation.
pub fn apply(index: &mut ManifestIndex) {
    let mut scope_ids: Vec<usize> = index.manifests.keys().copied().collect();
    scope_ids.sort_unstable();

    // Owned copies of every entry, so the diagnostic lists can be edited
    // while the entries are read.
    let decls: BTreeMap<usize, Vec<AckDecl>> = scope_ids
        .iter()
        .map(|id| (*id, index.manifests[id].manifest.acknowledged.clone()))
        .filter(|(_, v)| !v.is_empty())
        .collect();
    if decls.is_empty() {
        return;
    }
    let mut seen: BTreeSet<PathBuf> = BTreeSet::new();
    let mut all: Vec<Entry> = Vec::new();
    let mut per_scope: BTreeMap<usize, Vec<Entry>> = BTreeMap::new();
    for (id, list) in &decls {
        let contract_path = index.manifests[id].contract_path.clone();
        for (i, decl) in list.iter().enumerate() {
            per_scope.entry(*id).or_default().push(Entry {
                scope_id: *id,
                contract_path: contract_path.clone(),
                index: i,
                decl,
            });
        }
        // A contract bound to several scopes (one launch file included
        // twice) is one list: matched per scope, reported once.
        if seen.insert(contract_path.clone()) {
            for (i, decl) in list.iter().enumerate() {
                all.push(Entry {
                    scope_id: *id,
                    contract_path: contract_path.clone(),
                    index: i,
                    decl,
                });
            }
        }
    }

    let mut used: BTreeSet<(PathBuf, usize)> = BTreeSet::new();
    let mut on_error: BTreeSet<(PathBuf, usize)> = BTreeSet::new();
    let mut acked: Vec<Acknowledged> = Vec::new();

    let mut sort = |diags: Vec<Diagnostic>,
                    entries: &[Entry],
                    scope_id: Option<usize>,
                    acked: &mut Vec<Acknowledged>|
     -> Vec<Diagnostic> {
        let mut kept = Vec::with_capacity(diags.len());
        for d in diags {
            let Some(e) = entries.iter().find(|e| e.matches(&d)) else {
                kept.push(d);
                continue;
            };
            used.insert(e.key());
            if d.severity == Severity::Error {
                on_error.insert(e.key());
                kept.push(d);
                continue;
            }
            acked.push(Acknowledged {
                reason: e.decl.reason.clone(),
                contract_path: e.contract_path.clone(),
                entry: e.index,
                scope_id,
                diagnostic: d,
            });
        }
        kept
    };

    for (id, entries) in &per_scope {
        let m = index.manifests.get_mut(id).expect("scope listed above");
        let diags = std::mem::take(&mut m.diagnostics);
        m.diagnostics = sort(diags, entries, Some(*id), &mut acked);
    }
    let diags = std::mem::take(&mut index.merge_diagnostics);
    index.merge_diagnostics = sort(diags, &all, None, &mut acked);

    let mut out: Vec<Diagnostic> = Vec::new();
    for e in &all {
        let key = e.key();
        let site = super::value_rules::site(index, e.scope_id, "acknowledged");
        let (rule, at) = (&e.decl.rule, &e.decl.at);
        if on_error.contains(&key) {
            out.push(Diagnostic {
                rule_id: "ack-error".to_string(),
                severity: Severity::Error,
                message: format!(
                    "{site}: acknowledged[{}] names `{rule}` at `{at}`, which is an ERROR -- \
                     an error cannot be acknowledged, only fixed. The error is reported as it \
                     stands; delete the entry",
                    e.index
                ),
                path: format!("acknowledged[{}]", e.index),
                span: None,
            });
        } else if !used.contains(&key) {
            out.push(Diagnostic {
                rule_id: "ack-stale".to_string(),
                severity: Severity::Warning,
                message: format!(
                    "{site}: acknowledged[{}] (`{rule}` at `{at}`) matches no diagnostic -- the \
                     finding it acknowledged is gone, or the rule id or path is misspelled \
                     (`check --format json` prints each diagnostic's `rule` and `path`). Delete \
                     or correct the entry",
                    e.index
                ),
                path: format!("acknowledged[{}]", e.index),
                span: None,
            });
        }
    }

    let acked_warnings = acked
        .iter()
        .filter(|a| a.diagnostic.severity == Severity::Warning)
        .count();
    index.total_warnings = index.total_warnings.saturating_sub(acked_warnings);
    for d in &out {
        match d.severity {
            Severity::Error => index.total_errors += 1,
            Severity::Warning => index.total_warnings += 1,
            Severity::Info => {}
        }
    }
    index.merge_diagnostics.extend(out);
    index.acknowledged.extend(acked);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::manifest_loader::{ContractChannel, ResolvedManifest};
    use ros_launch_manifest_types::Manifest;

    fn diag(rule: &str, severity: Severity, path: &str) -> Diagnostic {
        Diagnostic {
            rule_id: rule.to_string(),
            severity,
            message: format!("{rule} finding"),
            path: path.to_string(),
            span: None,
        }
    }

    fn ack(rule: &str, at: &str) -> AckDecl {
        AckDecl {
            rule: rule.into(),
            at: at.into(),
            reason: format!("keep {rule}"),
        }
    }

    fn scope(
        id: usize,
        file: &str,
        acks: Vec<AckDecl>,
        diags: Vec<Diagnostic>,
    ) -> ResolvedManifest {
        ResolvedManifest {
            scope_id: id,
            pkg: None,
            file: format!("{file}.launch.xml"),
            ns: "/".into(),
            channel: ContractChannel::Provider,
            contract_path: PathBuf::from(format!("/c/{file}.contract.yaml")),
            manifest: Manifest {
                version: 1,
                acknowledged: acks,
                ..Default::default()
            },
            source: String::new(),
            diagnostics: diags,
        }
    }

    fn rules(diags: &[Diagnostic]) -> Vec<(&str, &str)> {
        diags
            .iter()
            .map(|d| (d.rule_id.as_str(), d.path.as_str()))
            .collect()
    }

    #[test]
    fn a_matching_warning_is_moved_and_counted_apart() {
        let mut index = ManifestIndex::default();
        index.manifests.insert(
            0,
            scope(
                0,
                "a",
                vec![ack("wiring", "nodes.n.paths.p")],
                vec![
                    diag("wiring", Severity::Warning, "nodes.n.paths.p"),
                    diag("wiring", Severity::Warning, "nodes.n.paths.q"),
                ],
            ),
        );
        index.total_warnings = 2;
        apply(&mut index);
        assert_eq!(
            rules(&index.manifests[&0].diagnostics),
            vec![("wiring", "nodes.n.paths.q")]
        );
        assert_eq!(index.acknowledged.len(), 1);
        let a = &index.acknowledged[0];
        assert_eq!(a.diagnostic.path, "nodes.n.paths.p");
        assert_eq!(a.reason, "keep wiring");
        assert_eq!(a.scope_id, Some(0));
        assert_eq!(a.entry, 0);
        assert!(index.merge_diagnostics.is_empty(), "no stale, no error");
        assert_eq!(index.total_warnings, 1);
    }

    #[test]
    fn an_info_can_be_acknowledged_too() {
        let mut index = ManifestIndex::default();
        index.manifests.insert(
            0,
            scope(
                0,
                "a",
                vec![ack("derivable-rate", "topics./t.rate_hz")],
                vec![],
            ),
        );
        index.merge_diagnostics = vec![diag("derivable-rate", Severity::Info, "topics./t.rate_hz")];
        apply(&mut index);
        assert!(index.merge_diagnostics.is_empty());
        assert_eq!(index.acknowledged.len(), 1);
        assert_eq!(index.acknowledged[0].scope_id, None);
    }

    #[test]
    fn an_entry_matching_an_error_is_itself_an_error_and_the_error_stays() {
        let mut index = ManifestIndex::default();
        index.manifests.insert(
            0,
            scope(
                0,
                "a",
                vec![ack("rate-hierarchy", "nodes.n.sub.s")],
                vec![diag("rate-hierarchy", Severity::Error, "nodes.n.sub.s")],
            ),
        );
        index.total_errors = 1;
        apply(&mut index);
        assert_eq!(
            rules(&index.manifests[&0].diagnostics),
            vec![("rate-hierarchy", "nodes.n.sub.s")]
        );
        assert!(index.acknowledged.is_empty());
        assert_eq!(
            rules(&index.merge_diagnostics),
            vec![("ack-error", "acknowledged[0]")]
        );
        assert_eq!(index.merge_diagnostics[0].severity, Severity::Error);
        assert!(
            index.merge_diagnostics[0]
                .message
                .contains("an error cannot be acknowledged"),
            "{}",
            index.merge_diagnostics[0].message
        );
        assert_eq!(index.total_errors, 2);
    }

    #[test]
    fn an_entry_matching_nothing_is_stale() {
        let mut index = ManifestIndex::default();
        index.manifests.insert(
            0,
            scope(
                0,
                "a",
                vec![
                    ack("wiring", "nodes.n.paths.p"),
                    // Right rule, wrong path: matching is exact, not a prefix.
                    ack("wiring", "nodes.n"),
                ],
                vec![diag("wiring", Severity::Warning, "nodes.n.paths.p")],
            ),
        );
        apply(&mut index);
        assert_eq!(
            rules(&index.merge_diagnostics),
            vec![("ack-stale", "acknowledged[1]")]
        );
        assert_eq!(index.merge_diagnostics[0].severity, Severity::Warning);
        assert!(
            index.merge_diagnostics[0]
                .message
                .contains("acknowledged[1] (`wiring` at `nodes.n`) matches no diagnostic"),
            "{}",
            index.merge_diagnostics[0].message
        );
    }

    #[test]
    fn a_per_scope_diagnostic_matches_only_its_own_contract() {
        let mut index = ManifestIndex::default();
        index.manifests.insert(
            0,
            scope(0, "a", vec![ack("wiring", "nodes.n.paths.p")], vec![]),
        );
        index.manifests.insert(
            1,
            scope(
                1,
                "b",
                vec![],
                vec![diag("wiring", Severity::Warning, "nodes.n.paths.p")],
            ),
        );
        apply(&mut index);
        assert_eq!(
            index.manifests[&1].diagnostics.len(),
            1,
            "b's own finding stands"
        );
        assert!(index.acknowledged.is_empty());
        assert_eq!(
            rules(&index.merge_diagnostics),
            vec![("ack-stale", "acknowledged[0]")]
        );
    }

    #[test]
    fn a_cross_scope_diagnostic_matches_any_contract() {
        let mut index = ManifestIndex::default();
        index.manifests.insert(0, scope(0, "a", vec![], vec![]));
        index.manifests.insert(
            1,
            scope(
                1,
                "b",
                vec![ack("reaction-unguarded", "hazards.h.reaction")],
                vec![],
            ),
        );
        index.merge_diagnostics = vec![diag(
            "reaction-unguarded",
            Severity::Warning,
            "hazards.h.reaction",
        )];
        apply(&mut index);
        assert!(index.merge_diagnostics.is_empty());
        assert_eq!(index.acknowledged.len(), 1);
        assert_eq!(
            index.acknowledged[0].contract_path,
            PathBuf::from("/c/b.contract.yaml")
        );
    }

    #[test]
    fn one_contract_in_two_scopes_reports_a_stale_entry_once() {
        let mut index = ManifestIndex::default();
        for id in [0, 1] {
            let mut m = scope(id, "a", vec![ack("wiring", "nodes.gone")], vec![]);
            m.contract_path = PathBuf::from("/c/a.contract.yaml");
            index.manifests.insert(id, m);
        }
        apply(&mut index);
        assert_eq!(
            rules(&index.merge_diagnostics),
            vec![("ack-stale", "acknowledged[0]")]
        );
    }

    #[test]
    fn without_entries_nothing_changes() {
        let mut index = ManifestIndex::default();
        index.manifests.insert(
            0,
            scope(0, "a", vec![], vec![diag("wiring", Severity::Warning, "x")]),
        );
        index.merge_diagnostics = vec![diag("endpoint-unwired", Severity::Warning, "y")];
        apply(&mut index);
        assert_eq!(index.manifests[&0].diagnostics.len(), 1);
        assert_eq!(index.merge_diagnostics.len(), 1);
        assert!(index.acknowledged.is_empty());
    }
}
