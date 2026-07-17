//! Cross-scope chain checks: `chain-link`, `chain-budget`,
//! `chain-sampling-feasibility` (Vocabulary v2 §4, chain-aware-mapper
//! spec's "Companion checker rules" section, Phase 44.2).
//!
//! `chains:` (`ros_launch_manifest_types::ChainDecl`) compose per-node
//! `paths:` from possibly-different manifest files into an end-to-end
//! budget with explicit `via:` connecting topics. Parsing (44.1)
//! validates local shape only (first/last segment is a path segment, no
//! adjacent `via`s, ...); the submodule's `chain-shape` rule (44.2)
//! rejects cyclic chains from a single manifest's view. Everything that
//! needs the merged, cross-file view lives here:
//!
//! - **`chain-link`** (error): every `{scope, path}` path segment must
//!   resolve to a declared path somewhere in the merged
//!   [`ManifestIndex`]; every `via:` topic must exist, be an output of
//!   the preceding path segment, and an input of the following one.
//! - **`chain-budget`** (warning): Σ declared `max_latency_ms` of the
//!   chain's non-boundary (event-segment) path segments + `sampling_cost`
//!   (see below) must fit the chain's declared `max_latency_ms`.
//! - **`chain-sampling-feasibility`** (warning): `sampling_cost` alone
//!   meeting or exceeding the chain budget is structural infeasibility —
//!   no scheduling assignment can fix it (chain-aware-mapper spec).
//!
//! `sampling_cost` = Σ over `timer`-triggered ("boundary") path segments
//! of `(1000 / rate_hz) + segment.max_latency_ms.unwrap_or(0)` — one
//! period per clock crossing plus that boundary's own declared execution
//! budget (conservative one-period-per-boundary bound, chain-aware-mapper
//! spec). Non-boundary (`input`-triggered or unclassified) segments
//! contribute their own `max_latency_ms` directly to the `chain-budget`
//! sum instead.
//!
//! A chain segment's `{scope, path}` is resolved against every loaded
//! manifest whose namespace (`ResolvedManifest.ns`) equals `scope`,
//! first searching that scope's node-level `paths:` (any node, matched
//! by path name — chain segments intentionally don't name the owning
//! node, only the scope + path name), then falling back to the scope's
//! own top-level `paths:` (a whole-scope aggregate can also serve as a
//! chain link, e.g. representing an opaque external subtree as one hop).

use super::manifest_loader::ManifestIndex;
use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::{ChainDecl, ChainSegment, EffectiveTrigger};
use std::collections::HashMap;

/// A resolved chain path segment: everything `chain-link`/`chain-budget`
/// need about the path a `{scope, path}` segment points at.
struct ResolvedSegment {
    /// Human-readable label for diagnostics.
    label: String,
    trigger: EffectiveTrigger,
    max_latency_ms: Option<f64>,
    /// Resolved input topic FQNs.
    input_topics: Vec<String>,
    /// Resolved output topic FQNs.
    output_topics: Vec<String>,
}

/// Run all three chain checks over every `chains:` block declared in any
/// loaded manifest, pushing diagnostics into `index.merge_diagnostics`.
pub fn check_chains(index: &mut ManifestIndex) {
    // Endpoint-ref ("<node_fqn>/<endpoint>") -> topic FQN, built once from
    // the merged topic index (mirrors the pattern in `manifest_loader`'s
    // cross-scope QoS/rate checks).
    let mut pub_map: HashMap<String, String> = HashMap::new();
    let mut sub_map: HashMap<String, String> = HashMap::new();
    for (fqn, topic) in &index.topics {
        for p in &topic.publishers {
            pub_map.insert(p.clone(), fqn.clone());
        }
        for s in &topic.subscribers {
            sub_map.insert(s.clone(), fqn.clone());
        }
    }

    // Snapshot chains from every manifest — (chain name, decl) — to avoid
    // borrowing `index` immutably while pushing diagnostics into it. The
    // declaring scope isn't needed for resolution (segments carry their
    // own absolute `scope:`), only for potential future provenance.
    let chains: Vec<(String, ChainDecl)> = index
        .manifests
        .values()
        .flat_map(|resolved| {
            resolved
                .manifest
                .chains
                .iter()
                .map(|(name, decl)| (name.clone(), decl.clone()))
        })
        .collect();

    for (chain_name, chain) in &chains {
        check_one_chain(index, &pub_map, &sub_map, chain_name, chain);
    }
}

fn check_one_chain(
    index: &mut ManifestIndex,
    pub_map: &HashMap<String, String>,
    sub_map: &HashMap<String, String>,
    chain_name: &str,
    chain: &ChainDecl,
) {
    // Resolve every path segment; `None` for a segment that couldn't be
    // resolved (chain-link error already emitted for it) or for `via`
    // placeholders (not path segments).
    let resolved: Vec<Option<ResolvedSegment>> = chain
        .segments
        .iter()
        .map(|seg| match seg {
            ChainSegment::Path { scope, path } => {
                let r = resolve_segment(index, pub_map, sub_map, scope, path);
                if r.is_none() {
                    index.merge_diagnostics.push(Diagnostic {
                        rule_id: "chain-link".to_string(),
                        severity: Severity::Error,
                        message: format!(
                            "chain '{chain_name}' segment (scope='{scope}', path='{path}') \
                             does not resolve to any declared path in the merged manifest tree"
                        ),
                        path: format!("chains.{chain_name}"),
                        span: None,
                    });
                }
                r
            }
            ChainSegment::Via { .. } => None,
        })
        .collect();

    // Verify every `via:` topic against its neighboring path segments.
    for (i, seg) in chain.segments.iter().enumerate() {
        let ChainSegment::Via { via } = seg else {
            continue;
        };

        if !index.topics.contains_key(via) {
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "chain-link".to_string(),
                severity: Severity::Error,
                message: format!(
                    "chain '{chain_name}' via topic '{via}' (segment {i}) does not exist in \
                     the merged manifest tree"
                ),
                path: format!("chains.{chain_name}"),
                span: None,
            });
        }

        if let Some(Some(prev)) = i.checked_sub(1).and_then(|p| resolved.get(p))
            && !prev.output_topics.iter().any(|t| t == via)
        {
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "chain-link".to_string(),
                severity: Severity::Error,
                message: format!(
                    "chain '{chain_name}' via topic '{via}' is not output by the preceding \
                     segment '{}' (outputs: {:?})",
                    prev.label, prev.output_topics
                ),
                path: format!("chains.{chain_name}"),
                span: None,
            });
        }
        if let Some(Some(next)) = resolved.get(i + 1)
            && !next.input_topics.iter().any(|t| t == via)
        {
            index.merge_diagnostics.push(Diagnostic {
                rule_id: "chain-link".to_string(),
                severity: Severity::Error,
                message: format!(
                    "chain '{chain_name}' via topic '{via}' is not consumed by the following \
                     segment '{}' (inputs: {:?})",
                    next.label, next.input_topics
                ),
                path: format!("chains.{chain_name}"),
                span: None,
            });
        }
    }

    // chain-budget / chain-sampling-feasibility: only meaningful when
    // every *path* segment resolved (via placeholders are legitimately
    // `None` in `resolved` — only path segments that failed
    // `resolve_segment` should suppress this) — skip to avoid cascading
    // noise on top of the chain-link errors already emitted above.
    let any_path_segment_unresolved = chain
        .segments
        .iter()
        .zip(resolved.iter())
        .any(|(seg, r)| matches!(seg, ChainSegment::Path { .. }) && r.is_none());
    if any_path_segment_unresolved {
        return;
    }

    let mut declared_sum = 0.0_f64;
    let mut sampling_cost = 0.0_f64;
    let mut boundary_breakdown: Vec<String> = Vec::new();

    for seg in resolved.iter().flatten() {
        match seg.trigger {
            EffectiveTrigger::Timer { rate_hz } if rate_hz > 0.0 => {
                let period_ms = 1000.0 / rate_hz;
                let exec_ms = seg.max_latency_ms.unwrap_or(0.0);
                let contrib = period_ms + exec_ms;
                sampling_cost += contrib;
                boundary_breakdown.push(format!(
                    "{} @ {rate_hz}Hz: period {period_ms:.2}ms + exec {exec_ms:.2}ms = \
                     {contrib:.2}ms",
                    seg.label
                ));
            }
            _ => {
                declared_sum += seg.max_latency_ms.unwrap_or(0.0);
            }
        }
    }

    let total = declared_sum + sampling_cost;

    if sampling_cost >= chain.max_latency_ms {
        index.merge_diagnostics.push(Diagnostic {
            rule_id: "chain-sampling-feasibility".to_string(),
            severity: Severity::Warning,
            message: format!(
                "chain '{chain_name}' sampling_cost ({sampling_cost:.2}ms, from clock \
                 boundaries alone) already meets or exceeds the chain budget \
                 ({}ms) — structurally infeasible, no scheduling assignment can fix this \
                 (period/architecture change required). Per-boundary breakdown: [{}]",
                chain.max_latency_ms,
                boundary_breakdown.join("; ")
            ),
            path: format!("chains.{chain_name}"),
            span: None,
        });
    } else if total > chain.max_latency_ms {
        index.merge_diagnostics.push(Diagnostic {
            rule_id: "chain-budget".to_string(),
            severity: Severity::Warning,
            message: format!(
                "chain '{chain_name}' total ({total:.2}ms = {declared_sum:.2}ms declared \
                 event-segment latency + {sampling_cost:.2}ms sampling_cost) exceeds the \
                 chain budget ({}ms). Per-boundary breakdown: [{}]",
                chain.max_latency_ms,
                boundary_breakdown.join("; ")
            ),
            path: format!("chains.{chain_name}"),
            span: None,
        });
    }
}

/// Resolve a `{scope, path}` chain segment against the merged index.
/// `scope` is matched against every loaded manifest's namespace
/// ([`super::manifest_loader::ResolvedManifest::ns`]); `path` is matched
/// by name, first among that scope's node-level `paths:` (any node —
/// chain segments name only the scope + path, not the owning node), then
/// falling back to the scope's own top-level `paths:`.
fn resolve_segment(
    index: &ManifestIndex,
    pub_map: &HashMap<String, String>,
    sub_map: &HashMap<String, String>,
    scope: &str,
    path_name: &str,
) -> Option<ResolvedSegment> {
    let scope_ids: Vec<usize> = index
        .manifests
        .values()
        .filter(|m| m.ns == scope)
        .map(|m| m.scope_id)
        .collect();
    if scope_ids.is_empty() {
        return None;
    }

    if let Some(np) = index
        .node_paths
        .iter()
        .find(|np| scope_ids.contains(&np.scope_id) && np.path_name == path_name)
    {
        let trigger = np.path.effective_trigger();
        // Input endpoint names come from the *effective* trigger (explicit
        // `trigger: {input: [...]}` or the legacy `input:` derivation),
        // matching the same source of truth `EffectiveTrigger` establishes
        // everywhere else (W1) — not the raw `path.input` field, which is
        // empty whenever the author used the explicit form.
        let input_eps: &[String] = match &trigger {
            EffectiveTrigger::Input(eps) => eps,
            _ => &[],
        };
        let input_topics = input_eps
            .iter()
            .filter_map(|ep| sub_map.get(&format!("{}/{ep}", np.node_fqn)).cloned())
            .collect();
        let output_topics = np
            .path
            .output
            .iter()
            .filter_map(|ep| pub_map.get(&format!("{}/{ep}", np.node_fqn)).cloned())
            .collect();
        return Some(ResolvedSegment {
            label: format!("{}.{}", np.node_fqn, path_name),
            trigger,
            max_latency_ms: np.path.max_latency_ms,
            input_topics,
            output_topics,
        });
    }

    if let Some(sp) = index
        .scope_paths
        .iter()
        .find(|sp| scope_ids.contains(&sp.scope_id) && sp.path_name == path_name)
    {
        return Some(ResolvedSegment {
            label: format!("scope[{scope}].{path_name}"),
            trigger: sp.path.effective_trigger(),
            max_latency_ms: sp.path.max_latency_ms,
            input_topics: sp.input_topics.clone(),
            output_topics: sp.output_topics.clone(),
        });
    }

    None
}

#[cfg(test)]
mod tests {
    use super::super::{
        launch_dump::{LaunchDump, ScopeEntry, ScopeOrigin},
        manifest_loader::{ContractSources, ManifestIndex, load_manifests},
    };
    use ros_launch_manifest_check::Severity;
    use std::{collections::HashMap, path::PathBuf};

    fn fixture_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../ros-launch-manifest/tests/fixtures")
    }

    /// Same materialization pattern as `manifest_loader::tests::overlay_index`
    /// — duplicated locally since that helper is private to
    /// `manifest_loader`'s own test module.
    fn overlay_index(dump: &LaunchDump) -> ManifestIndex {
        let tmp = tempfile::TempDir::new().unwrap();
        for scope in &dump.scopes {
            let Some(origin) = &scope.origin else {
                continue;
            };
            let pkg_dir = origin.pkg.as_deref().unwrap_or("_");
            let stem = origin
                .file
                .strip_suffix(".launch.xml")
                .or_else(|| origin.file.strip_suffix(".launch.py"))
                .or_else(|| origin.file.strip_suffix(".launch.yaml"))
                .or_else(|| origin.file.strip_suffix(".launch"))
                .unwrap_or(&origin.file);
            let src = fixture_dir().join(pkg_dir).join("manifest.yaml");
            if !src.exists() {
                continue;
            }
            let dest_dir = tmp.path().join(pkg_dir).join("launch");
            std::fs::create_dir_all(&dest_dir).unwrap();
            std::fs::copy(&src, dest_dir.join(format!("{stem}.contract.yaml"))).unwrap();
        }
        let sources = ContractSources {
            overlay: Some(tmp.path().to_path_buf()),
            provider: false,
        };
        load_manifests(dump, &sources).unwrap()
    }

    fn make_dump(scopes: Vec<ScopeEntry>) -> LaunchDump {
        LaunchDump {
            node: vec![],
            load_node: vec![],
            container: vec![],
            lifecycle_node: vec![],
            file_data: HashMap::new(),
            variables: HashMap::new(),
            scopes,
        }
    }

    fn scope(id: usize, pkg: &str, file: &str, ns: &str, parent: Option<usize>) -> ScopeEntry {
        ScopeEntry {
            id,
            origin: Some(ScopeOrigin {
                pkg: Some(pkg.to_string()),
                file: file.to_string(),
                path: None,
            }),
            ns: ns.to_string(),
            args: HashMap::new(),
            parent,
        }
    }

    /// The three-scope chain fixture tree shared by all tests below:
    /// scope 0 (root, ns "") declares `chains:`; scope 1 (ns "/a") and
    /// scope 2 (ns "/b") declare the linked node paths.
    fn chain_dump() -> LaunchDump {
        make_dump(vec![
            scope(0, "manifest_chain_root", "root.launch.xml", "", None),
            scope(1, "manifest_chain_a", "a.launch.xml", "/a", Some(0)),
            scope(2, "manifest_chain_b", "b.launch.xml", "/b", Some(0)),
        ])
    }

    fn diags_for<'a>(index: &'a ManifestIndex, rule_id: &str, needle: &str) -> Vec<&'a str> {
        index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == rule_id && d.message.contains(needle))
            .map(|d| d.message.as_str())
            .collect()
    }

    #[test]
    fn ok_chain_has_no_chain_link_or_budget_diagnostics() {
        let index = overlay_index(&chain_dump());
        assert!(
            index
                .merge_diagnostics
                .iter()
                .filter(|d| d.path == "chains.ok_chain")
                .collect::<Vec<_>>()
                .is_empty(),
            "ok_chain should be clean: {:?}",
            index
                .merge_diagnostics
                .iter()
                .filter(|d| d.path == "chains.ok_chain")
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn broken_via_chain_emits_chain_link_errors() {
        let index = overlay_index(&chain_dump());
        let errs: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| {
                d.path == "chains.broken_via_chain"
                    && d.rule_id == "chain-link"
                    && d.severity == Severity::Error
            })
            .collect();
        assert!(
            !errs.is_empty(),
            "expected chain-link errors for broken_via_chain, got: {:?}",
            index.merge_diagnostics
        );
        assert!(errs.iter().any(|d| d.message.contains("nonexistent")));
    }

    #[test]
    fn budget_blown_chain_emits_chain_budget_warning() {
        let index = overlay_index(&chain_dump());
        let warns = diags_for(&index, "chain-budget", "budget_blown_chain");
        assert_eq!(warns.len(), 1, "got: {:?}", index.merge_diagnostics);
    }

    #[test]
    fn sampling_infeasible_chain_emits_sampling_feasibility_warning_not_budget() {
        let index = overlay_index(&chain_dump());
        let feas = diags_for(
            &index,
            "chain-sampling-feasibility",
            "sampling_infeasible_chain",
        );
        assert_eq!(feas.len(), 1, "got: {:?}", index.merge_diagnostics);
        // Sampling-infeasibility supersedes the plain chain-budget diagnostic
        // for the same chain (avoid double-reporting the same overrun).
        let budget = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "chain-budget" && d.path == "chains.sampling_infeasible_chain")
            .count();
        assert_eq!(budget, 0);
    }

    /// Via topic exists in the merged tree but is NOT output by the
    /// preceding path segment (make_slow outputs /link/out2, via is
    /// /link/out) — must emit exactly the "not output by preceding"
    /// chain-link error, and neither the topic-existence nor the
    /// consumed-by-following errors (consume does consume /link/out).
    #[test]
    fn via_exists_but_not_output_by_preceding_segment_errors() {
        let index = overlay_index(&chain_dump());
        let errs: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| {
                d.path == "chains.via_not_output_chain"
                    && d.rule_id == "chain-link"
                    && d.severity == Severity::Error
            })
            .collect();
        assert_eq!(errs.len(), 1, "got: {:?}", index.merge_diagnostics);
        assert!(
            errs[0]
                .message
                .contains("not output by the preceding segment"),
            "wrong branch: {}",
            errs[0].message
        );
        assert!(errs[0].message.contains("/a/producer.make_slow"));
    }

    /// Via topic exists and IS output by the preceding segment (make →
    /// /link/out), but the following segment (consume2) consumes
    /// /link/out2 — must emit exactly the "not consumed by the following"
    /// chain-link error and no others.
    #[test]
    fn via_exists_but_not_consumed_by_following_segment_errors() {
        let index = overlay_index(&chain_dump());
        let errs: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| {
                d.path == "chains.via_not_consumed_chain"
                    && d.rule_id == "chain-link"
                    && d.severity == Severity::Error
            })
            .collect();
        assert_eq!(errs.len(), 1, "got: {:?}", index.merge_diagnostics);
        assert!(
            errs[0]
                .message
                .contains("not consumed by the following segment"),
            "wrong branch: {}",
            errs[0].message
        );
        assert!(errs[0].message.contains("/b/consumer.consume2"));
    }

    #[test]
    fn resolve_segment_returns_none_for_unknown_scope_or_path() {
        let index = overlay_index(&chain_dump());
        let pub_map = HashMap::new();
        let sub_map = HashMap::new();
        // Unknown scope entirely.
        assert!(super::resolve_segment(&index, &pub_map, &sub_map, "/nope", "make").is_none());
        // Known scope, unknown path name.
        assert!(super::resolve_segment(&index, &pub_map, &sub_map, "/a", "nonexistent").is_none());
        // Known scope + path resolves.
        assert!(super::resolve_segment(&index, &pub_map, &sub_map, "/a", "make").is_some());
    }
}
