//! `chain_aware` — [`SchedMapper`] implementing the clock-segmented-chain
//! priority-assignment algorithm (design
//! `docs/superpowers/specs/2026-07-17-chain-aware-mapper-design.md`,
//! Phase 44.3).
//!
//! Summary of the algorithm (see the design doc for the full rationale):
//!
//! 1. Feasibility: each declared chain's `sampling_cost` (sum of boundary
//!    `period_ms + exec_ms`) is subtracted from its `max_latency_ms`
//!    budget; chains with `controllable <= 0` are excluded from priority
//!    shaping (their members fall through to the non-chain path below) and
//!    produce a [`MapWarning::ChainInfeasible`].
//! 2. Feasible chains are ordered: criticality desc, controllable-slack
//!    asc, name asc.
//! 3. Within a chain, elements are walked from sink to source (reverse
//!    declaration order): each `Segment`'s nodes are ranked
//!    drain-toward-sink (its `nodes_in_topo_order` reversed); each maximal
//!    run of consecutive `Boundary` elements is re-ordered by
//!    rate-monotonic (shorter period first) before being emitted — so a
//!    boundary run's *rank position* comes from its place in the reversed
//!    walk, but its *internal order* comes from RM, exactly matching the
//!    design's worked example (`ekf` 50 Hz above `planning` 10 Hz despite
//!    `planning` being chain-adjacent to `follower`).
//! 4. Remaining (non-chain, or infeasible-chain, or no-chain-membership)
//!    paths are grouped by node `criticality` (High/Medium/Low/None), then
//!    ordered within each bucket by a single ascending "time budget"
//!    (`1000/rate_hz` for `Timer` paths, `max_latency_ms` for `Input`
//!    paths — this unifies RM and DM into one comparable unit, exactly
//!    reproducing both orderings without an arbitrary timer-vs-input
//!    split). Paths with *exactly* equal (bucket, budget) collapse into a
//!    single rank (Phase 41 tie-collapse decision — no alphabetical
//!    spread). `once`/`spontaneous`/`unclassified`/fact-less paths never
//!    enter this ranking and their node falls to
//!    [`crate::resolve::DEFAULT_TIER`] unless another of its paths ranked.
//! 5. The whole ranked sequence (chain items, in chain order, followed by
//!    non-chain items) is compressed into the platform's `rt_priority_band`
//!    (dense rank -> priority, order-preserving; when the band is narrower
//!    than the rank count, adjacent ranks collapse to equal priority,
//!    within-segment/boundary-run first, then within the same chain across
//!    segments, never across the chain/non-chain divide or a criticality
//!    bucket boundary).
//! 6. Node projection: a node's final priority is the **max** priority
//!    over all of its ranked paths (design issue #3 — the POSIX apply
//!    layer is per-node, not per-callback; this also transparently
//!    implements the "shared node takes max rank" rule for nodes appearing
//!    in multiple chains, design issue #4).

use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
};

use crate::{
    chain::{
        ChainAwareDetail, ChainElement, MapDiagnostics, MapWarning, MapperPath, ResolvedChain,
    },
    mapper::{
        Criticality, MapError, MapperInput, PlatformFacts, SchedMapper, SchedPlan,
        require_posix_band,
    },
    platform::PriorityBand,
    resolve::{DEFAULT_TIER, ResolvedTier, ResolvedTierTable},
};

pub use crate::chain::EffectiveTrigger;

/// `chain_aware` (design doc, Phase 44.3).
#[derive(Debug, Default)]
pub struct ChainAwareMapper;

impl SchedMapper for ChainAwareMapper {
    fn name(&self) -> &str {
        "chain_aware"
    }

    fn map(&self, input: &MapperInput, facts: &PlatformFacts) -> Result<SchedPlan, MapError> {
        Ok(self.map_with_diagnostics(input, facts)?.0)
    }

    fn map_with_diagnostics(
        &self,
        input: &MapperInput,
        facts: &PlatformFacts,
    ) -> Result<(SchedPlan, MapDiagnostics), MapError> {
        chain_aware_map(input, facts)
    }
}

impl ChainAwareMapper {
    /// Platform-agnostic ranking (design steps 1–4): the priorityless
    /// [`RankedPlan`] an RTOS realizer (nano-ros) consumes instead of the
    /// `posix` band-compressed [`SchedPlan`]. No `PlatformFacts` — no band, no
    /// OS priorities. Thin wrapper over [`chain_aware_rank`].
    pub fn rank(&self, input: &MapperInput) -> RankedPlan {
        chain_aware_rank(input)
    }
}

// --- Step 1/2: feasibility ------------------------------------------------

/// One chain's feasibility facts (design "Model: clock-segmented chains").
#[derive(Clone, Debug, PartialEq)]
struct ChainFeasibility {
    sampling_cost_ms: f64,
    controllable_ms: f64,
    feasible: bool,
    /// `node/path` of every boundary whose WCET was ABSENT and therefore
    /// counted as zero. A verdict computed with these is optimistic by an
    /// unknown amount; the caller reports it rather than presenting a bare
    /// `feasible` (nano-ros issue 0259).
    boundaries_without_wcet: Vec<String>,
}

fn chain_feasibility(chain: &ResolvedChain) -> ChainFeasibility {
    // `exec_ms.unwrap_or(0.0)` is still how an absent WCET is COUNTED — there is
    // nothing better to count — but the absence is now recorded instead of
    // vanishing into the sum. Absent is not zero; a verdict that cannot tell
    // them apart reports headroom nobody measured.
    let mut boundaries_without_wcet = Vec::new();
    let mut sampling_cost_ms = 0.0f64;
    for e in &chain.elements {
        if let ChainElement::Boundary {
            node,
            path,
            period_ms,
            exec_ms,
        } = e
        {
            if exec_ms.is_none() {
                boundaries_without_wcet.push(format!("{node}/{path}"));
            }
            sampling_cost_ms += period_ms + exec_ms.unwrap_or(0.0);
        }
    }
    let controllable_ms = chain.max_latency_ms - sampling_cost_ms;
    ChainFeasibility {
        sampling_cost_ms,
        controllable_ms,
        feasible: controllable_ms > 0.0,
        boundaries_without_wcet,
    }
}

// --- Ranking items ---------------------------------------------------------

/// One (node, path) ranked item — the **priorityless** unit of the
/// platform-agnostic core's output (design steps 1–4). The `Vec<RankItem>`
/// order IS the priority order (highest first); no OS priority numbers are
/// assigned here. Each platform realizer turns this into concrete scheduling:
/// play_launch's `posix` realizer band-compresses into
/// [`crate::resolve::ResolvedTierTable`] (steps 5–6), while nano-ros's RTOS
/// realizer maps segments → executors + kernel features (EDF /
/// preemption-threshold / sporadic / affinity) from the same ordering.
#[derive(Clone, Debug, PartialEq)]
pub struct RankItem {
    pub node: String,
    pub path: String,
    /// The finest mergeable scope (a chain's segment / boundary-run, or a
    /// non-chain criticality+budget bucket). Adjacent items sharing a
    /// `fine_group` are the first to collapse under band scarcity. Doubles as
    /// the **segment membership** an RTOS realizer groups a run-to-completion
    /// executor by.
    pub fine_group: usize,
    /// The next-coarser mergeable scope: the chain name for chain items
    /// (collapse across segments of the *same* chain is allowed under
    /// scarcity), `None` for non-chain items (never merge further — never
    /// across the chain/non-chain divide or a criticality bucket).
    pub coarse_group: Option<String>,
    /// Set only for non-chain items with *exactly* equal (criticality,
    /// budget) facts (Phase 41 tie-collapse decision): items sharing a
    /// `Some` `tie_group` always collapse to one rank, unconditionally
    /// (not just under band scarcity) — unlike `fine_group`/`coarse_group`,
    /// which are scarcity-driven merge *candidates* only. `None` for chain
    /// items (a chain's ranks are always distinct absent scarcity — see
    /// the worked example: `preproc`/`concat`/`detector` get 3 distinct
    /// ranks when the band has room).
    pub tie_group: Option<usize>,
    pub provenance: String,
}

/// The platform-agnostic core's output (design steps 1–4): the priorityless
/// ordered/segmented ranking plus the feasibility warnings. No band, no OS
/// priorities — each runtime's realizer turns this into concrete scheduling.
/// Produced by [`chain_aware_rank`] / [`ChainAwareMapper::rank`]; consumed by
/// the `posix` realizer ([`chain_aware_map`]) and by nano-ros's RTOS realizer.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct RankedPlan {
    /// Ranked (node, path) items, highest-priority-first.
    pub items: Vec<RankItem>,
    /// Feasibility warnings from step 2 (e.g. [`MapWarning::ChainInfeasible`]).
    /// The `posix` realizer appends its own band-scarcity warning; an RTOS
    /// realizer appends whatever its realization surfaces.
    pub warnings: Vec<MapWarning>,
}

/// Walk one feasible chain's elements from sink to source (design step 3),
/// producing items highest-priority-first.
fn chain_item_order(chain: &ResolvedChain, group_counter: &mut usize) -> Vec<RankItem> {
    let mut out = Vec::new();
    let elems = &chain.elements;
    let mut i = elems.len();

    while i > 0 {
        i -= 1;
        match &elems[i] {
            ChainElement::Segment {
                nodes_in_topo_order,
            } => {
                let fine = *group_counter;
                *group_counter += 1;
                let n = nodes_in_topo_order.len();
                for (idx, sn) in nodes_in_topo_order.iter().enumerate().rev() {
                    out.push(RankItem {
                        node: sn.node.clone(),
                        path: sn.path.clone(),
                        fine_group: fine,
                        coarse_group: Some(chain.name.clone()),
                        tie_group: None,
                        provenance: format!(
                            "derived(chain_aware: {} segment drain {}/{})",
                            chain.name,
                            n - idx,
                            n
                        ),
                    });
                }
            }
            ChainElement::Boundary { .. } => {
                // Gather the maximal contiguous run of Boundary elements
                // ending at (and including) index i.
                let mut run_start = i;
                while run_start > 0 && matches!(elems[run_start - 1], ChainElement::Boundary { .. })
                {
                    run_start -= 1;
                }
                let mut run: Vec<(&str, &str, f64)> = elems[run_start..=i]
                    .iter()
                    .filter_map(|e| match e {
                        ChainElement::Boundary {
                            node,
                            path,
                            period_ms,
                            ..
                        } => Some((node.as_str(), path.as_str(), *period_ms)),
                        ChainElement::Segment { .. } => None,
                    })
                    .collect();
                // Rate-monotonic within the run: shorter period first, tie
                // by node name (design step 4: "Boundary timers: ranked
                // among themselves by RM").
                run.sort_by(|a, b| {
                    a.2.partial_cmp(&b.2)
                        .unwrap_or(Ordering::Equal)
                        .then_with(|| a.0.cmp(b.0))
                });
                let fine = *group_counter;
                *group_counter += 1;
                for (node, path, period_ms) in run {
                    out.push(RankItem {
                        node: node.to_string(),
                        path: path.to_string(),
                        fine_group: fine,
                        coarse_group: Some(chain.name.clone()),
                        tie_group: None,
                        provenance: format!(
                            "derived(chain_aware: {} boundary RM period={period_ms}ms)",
                            chain.name
                        ),
                    });
                }
                i = run_start;
            }
        }
    }

    out
}

// --- Step 4: non-chain remainder -------------------------------------------

fn criticality_rank(c: Option<Criticality>) -> u8 {
    match c {
        Some(Criticality::High) => 0,
        Some(Criticality::Medium) => 1,
        Some(Criticality::Low) => 2,
        None => 3,
    }
}

/// The single ascending "time budget" a path contributes to non-chain
/// ranking: a timer's period, or an input path's declared
/// `max_latency_ms`. `None` when the path carries no usable fact (`once`,
/// `spontaneous`, `unclassified`, or an input path with no declared
/// deadline) — these never enter the ranking (design step 4:
/// "once/spontaneous/fact-less -> SCHED_OTHER").
fn path_budget_ms(path: &MapperPath) -> Option<f64> {
    match &path.effective_trigger {
        EffectiveTrigger::Timer { .. } => path.effective_trigger.period_ms(),
        EffectiveTrigger::Input(_) => path.max_latency_ms,
        EffectiveTrigger::Once | EffectiveTrigger::Spontaneous | EffectiveTrigger::Unclassified => {
            None
        }
    }
}

fn non_chain_item_order(
    input: &MapperInput,
    covered: &BTreeSet<(String, String)>,
    group_counter: &mut usize,
) -> Vec<RankItem> {
    struct Fact<'a> {
        node: &'a str,
        path: &'a str,
        criticality: Option<Criticality>,
        budget_ms: f64,
    }

    let mut facts: Vec<Fact> = Vec::new();
    for node in &input.nodes {
        for path in &node.paths {
            if covered.contains(&(node.name.clone(), path.name.clone())) {
                continue;
            }
            if let Some(budget_ms) = path_budget_ms(path) {
                facts.push(Fact {
                    node: &node.name,
                    path: &path.name,
                    criticality: node.criticality,
                    budget_ms,
                });
            }
        }
    }

    // criticality bucket asc (High first), then budget asc (RM/DM unified),
    // then name asc for a deterministic, stable grouping order.
    //
    // Why one unified ascending-millisecond sort instead of separate
    // "timer group by RM" / "input group by DM" sub-orders (design step 5
    // names both but leaves their relative order unstated): under the
    // implicit-deadline assumption (deadline = period), RM *is* DM — a
    // timer path's period is exactly its deadline, so comparing a timer's
    // period against an input path's declared `max_latency_ms` in the same
    // unit is the DM ordering extended over both trigger kinds. This
    // reproduces pure-RM ordering among timers and pure-DM ordering among
    // input paths, and gives mixed buckets a principled interleaving (a
    // 50 ms-deadline input path outranks a 100 ms-period timer; a
    // 10 ms-period timer outranks that same input path) instead of an
    // arbitrary timer-vs-input group split.
    facts.sort_by(|a, b| {
        criticality_rank(a.criticality)
            .cmp(&criticality_rank(b.criticality))
            .then_with(|| {
                a.budget_ms
                    .partial_cmp(&b.budget_ms)
                    .unwrap_or(Ordering::Equal)
            })
            .then_with(|| a.node.cmp(b.node))
    });

    let mut items = Vec::new();
    let mut idx = 0;
    while idx < facts.len() {
        let mut j = idx + 1;
        while j < facts.len()
            && facts[j].criticality == facts[idx].criticality
            && facts[j].budget_ms == facts[idx].budget_ms
        {
            j += 1;
        }
        // Phase 41 tie-collapse: every fact in [idx, j) shares the same
        // (criticality, budget) and gets a single rank slot.
        let fine = *group_counter;
        *group_counter += 1;
        for f in &facts[idx..j] {
            items.push(RankItem {
                node: f.node.to_string(),
                path: f.path.to_string(),
                fine_group: fine,
                coarse_group: None,
                tie_group: Some(fine),
                provenance: format!(
                    "derived(chain_aware: non-chain criticality={:?} budget_ms={})",
                    f.criticality, f.budget_ms
                ),
            });
        }
        idx = j;
    }

    items
}

// --- Step 6: node projection + plan assembly --------------------------------

/// Platform-agnostic core (design steps 1–4): feasibility → chain order →
/// per-chain + non-chain ranking. Produces the priorityless [`RankedPlan`] —
/// no band, no OS priorities. Both play_launch's `posix` realizer
/// ([`chain_aware_map`]) and nano-ros's RTOS realizer consume this same
/// ordering; the split (2026-07-20) lets each apply its own realization
/// without inheriting the other's (see the crate's `docs/scheduling.md`
/// §"Cross-repo design agreement" + play_launch phase-45 §45.10).
pub fn chain_aware_rank(input: &MapperInput) -> RankedPlan {
    let mut warnings = Vec::new();
    let mut group_counter = 0usize;

    // Steps 1-2: feasibility.
    let mut candidates: Vec<(&ResolvedChain, ChainFeasibility)> = Vec::new();
    for chain in &input.chains {
        let feas = chain_feasibility(chain);
        if feas.feasible {
            // Feasible ON THE EVIDENCE AVAILABLE. Say so when some of that
            // evidence was missing, at the point the assumption is made.
            if !feas.boundaries_without_wcet.is_empty() {
                warnings.push(MapWarning::ChainFeasibleWithoutWcet {
                    chain: chain.name.clone(),
                    boundaries_without_wcet: feas.boundaries_without_wcet.clone(),
                });
            }
            candidates.push((chain, feas));
        } else {
            warnings.push(MapWarning::ChainInfeasible {
                chain: chain.name.clone(),
                sampling_cost_ms: feas.sampling_cost_ms,
                budget_ms: chain.max_latency_ms,
            });
        }
    }

    // Step 3: chain order (criticality desc, controllable-slack asc, name).
    candidates.sort_by(|(ca, fa), (cb, fb)| {
        cb.criticality
            .cmp(&ca.criticality)
            .then_with(|| {
                fa.controllable_ms
                    .partial_cmp(&fb.controllable_ms)
                    .unwrap_or(Ordering::Equal)
            })
            .then_with(|| ca.name.cmp(&cb.name))
    });

    // Step 3-4: per-chain items, in chain order.
    let mut items: Vec<RankItem> = Vec::new();
    let mut covered: BTreeSet<(String, String)> = BTreeSet::new();
    for (chain, _feas) in &candidates {
        let chain_items = chain_item_order(chain, &mut group_counter);
        for it in &chain_items {
            covered.insert((it.node.clone(), it.path.clone()));
        }
        items.extend(chain_items);
    }

    // Step 4: non-chain remainder (also covers infeasible-chain members —
    // their paths were never added to `covered`).
    items.extend(non_chain_item_order(input, &covered, &mut group_counter));

    RankedPlan { items, warnings }
}

/// `chain_aware` map: the agnostic core ([`chain_aware_rank`]) followed by the
/// `posix` (Linux) realizer. Backward-compatible with the pre-split combined
/// path — byte-identical output.
fn chain_aware_map(
    input: &MapperInput,
    facts: &PlatformFacts,
) -> Result<(SchedPlan, MapDiagnostics), MapError> {
    realize_posix(&chain_aware_rank(input), input, facts)
}

/// The `posix` (Linux) realizer (design steps 5–6): band-compress the agnostic
/// ranking into OS priorities, project per-node (max over paths), and emit a
/// [`ResolvedTierTable`]. play_launch's Linux apply layer consumes this;
/// nano-ros does NOT — it writes its own RTOS realizer over the same
/// [`RankedPlan`] ([`chain_aware_rank`]).
/// Decide which tied nodes get `SCHED_RR` instead of `SCHED_FIFO`.
///
/// A tie is two or more nodes at the same final priority. Under `SCHED_FIFO`
/// the first to run holds the CPU until it blocks; `SCHED_RR` rotates between
/// them. So a tie is exactly the case RR exists for.
///
/// The catch is that Linux's RR slice is a **global sysctl**
/// (`/proc/sys/kernel/sched_rr_timeslice_ms`), not a per-task value —
/// `TierPlatformSpec::time_slice_us` cannot express it — and its default is
/// **100 ms**. Two tied 10 Hz nodes would get a slice as long as their whole
/// period, so RR would behave exactly like FIFO while appearing to have solved
/// starvation. Deriving it unconditionally would be a cosmetic change dressed
/// as a real one.
///
/// The rule is therefore conditional: derive `Rr` only when the host's actual
/// slice is shorter than the shortest period among the tied nodes. Otherwise
/// keep `Fifo` and warn, naming both numbers. An absent `rr_timeslice_us` is
/// "unknown", never "assume the default".
fn rr_policy_for_ties(
    node_priority: &BTreeMap<String, i64>,
    input: &MapperInput,
    rr_slice_us: Option<u64>,
) -> (BTreeSet<String>, Vec<MapWarning>) {
    let mut by_priority: BTreeMap<i64, Vec<String>> = BTreeMap::new();
    for (node, prio) in node_priority {
        by_priority.entry(*prio).or_default().push(node.clone());
    }

    let mut rr_nodes = BTreeSet::new();
    let mut warnings = Vec::new();

    for (priority, nodes) in by_priority {
        if nodes.len() < 2 {
            continue;
        }

        let shortest_period_us = nodes
            .iter()
            .filter_map(|n| shortest_node_budget_us(input, n))
            .min();

        let slice_is_useful = match (rr_slice_us, shortest_period_us) {
            (Some(slice), Some(period)) => slice < period,
            _ => false,
        };

        if slice_is_useful {
            rr_nodes.extend(nodes);
        } else {
            warnings.push(MapWarning::UnmitigatedPriorityTie {
                priority,
                nodes,
                rr_timeslice_us: rr_slice_us,
                shortest_period_us,
            });
        }
    }

    (rr_nodes, warnings)
}

/// The shortest time budget any of a node's paths carries, in microseconds —
/// a timer's period or an input path's declared latency budget. `None` when
/// the node carries no usable timing fact at all.
fn shortest_node_budget_us(input: &MapperInput, node: &str) -> Option<u64> {
    input
        .nodes
        .iter()
        .find(|n| n.name == node)?
        .paths
        .iter()
        .filter_map(path_budget_ms)
        .map(|ms| (ms * 1000.0).round() as u64)
        .min()
}

fn realize_posix(
    ranked: &RankedPlan,
    input: &MapperInput,
    facts: &PlatformFacts,
) -> Result<(SchedPlan, MapDiagnostics), MapError> {
    let band = require_posix_band(facts, "chain_aware")?;
    let items = &ranked.items;
    let mut warnings = ranked.warnings.clone();

    // Step 5: band compression. When even the legal collapses can't fit
    // the band, the assignment clamps into `band.min` (introducing
    // cross-chain/bucket ties, never inversions) and we surface that as a
    // warning instead of silently violating design issue #5.
    let (priorities, band_overflow) = assign_priorities_compressed(items, &band);
    if let Some(overflow) = band_overflow {
        warnings.push(MapWarning::BandTooNarrow {
            distinct_classes: overflow.distinct_classes,
            band_width: overflow.band_width,
            clamped: overflow.clamped,
        });
    }

    // Step 6: node projection (max over paths) + diagnostics.
    let mut node_priority: BTreeMap<String, i64> = BTreeMap::new();
    let mut details = Vec::with_capacity(items.len());
    for (item, prio) in items.iter().zip(priorities.iter()) {
        node_priority
            .entry(item.node.clone())
            .and_modify(|p| {
                if *prio > *p {
                    *p = *prio;
                }
            })
            .or_insert(*prio);
        details.push(ChainAwareDetail {
            node: item.node.clone(),
            path: Some(item.path.clone()),
            priority: *prio,
            provenance: format!("{} -> prio {}", item.provenance, prio),
        });
    }

    // Step 6b: pick the RT policy per node. Band compression *creates* ties
    // (it collapses adjacent ranks, and clamps into `band.min` when the band
    // is too narrow), and under SCHED_FIFO a tie means one node can starve
    // the others. SCHED_RR would time-slice between them — but only if the
    // host's slice is short enough to matter. See `rr_policy_for_ties`.
    let rr_slice_us = match facts {
        crate::PlatformResources::Posix(p) => p.rr_timeslice_us,
        crate::PlatformResources::Raw(_) => None,
    };
    let (rr_nodes, tie_warnings) = rr_policy_for_ties(&node_priority, input, rr_slice_us);
    warnings.extend(tie_warnings);

    let mut tiers: Vec<ResolvedTier> = node_priority
        .iter()
        .map(|(name, prio)| {
            let sched = if rr_nodes.contains(name.as_str()) {
                crate::posix::PosixSched::Rr {
                    priority: *prio as i32,
                }
            } else {
                crate::posix::PosixSched::Fifo {
                    priority: *prio as i32,
                }
            };
            ResolvedTier {
                name: name.clone(),
                priority: *prio,
                sched_class: Some(sched.kind().as_str().to_string()),
                posix: Some(crate::posix::PosixPlacement {
                    sched,
                    affinity: crate::posix::PosixAffinity::Inherit,
                    uclamp: None,
                }),
                class: Some("real_time".to_string()),
                members: vec![name.clone()],
                ..Default::default()
            }
        })
        .collect();

    let ranked: BTreeSet<&str> = node_priority.keys().map(|s| s.as_str()).collect();
    let rest: Vec<String> = input
        .nodes
        .iter()
        .filter(|n| !ranked.contains(n.name.as_str()))
        .map(|n| n.name.clone())
        .collect();
    if !rest.is_empty() {
        let mut members = rest;
        members.sort();
        tiers.push(ResolvedTier {
            name: DEFAULT_TIER.to_string(),
            members,
            ..Default::default()
        });
    }

    tiers.sort_by(|a, b| b.priority.cmp(&a.priority).then(a.name.cmp(&b.name)));

    Ok((
        ResolvedTierTable { tiers },
        MapDiagnostics { details, warnings },
    ))
}

/// Real band-compression implementation (design step 5 / issue #5): dense
/// rank -> priority, order-preserving; collapses adjacent same-`fine_group`
/// runs first, then adjacent same-`coarse_group` runs, only while
/// `runs.len()` exceeds the band width.
///
/// Band-overflow facts for a [`MapWarning::BandTooNarrow`]: the irreducible
/// class count, the band width, and human-readable labels of the classes
/// clamped into `band.min` (Phase 44.4 review, cosmetic-6).
struct BandOverflow {
    distinct_classes: usize,
    band_width: usize,
    clamped: Vec<String>,
}

/// Returns the per-item priorities plus `Some(BandOverflow)` when the band
/// could not hold the remaining classes after every legal collapse — the
/// low classes were clamped into `band.min` (cross-chain/bucket ties, never
/// inversions) and the caller must surface a [`MapWarning::BandTooNarrow`].
fn assign_priorities_compressed(
    items: &[RankItem],
    band: &PriorityBand,
) -> (Vec<i64>, Option<BandOverflow>) {
    let n = items.len();
    if n == 0 {
        return (Vec::new(), None);
    }
    let width = (band.max - band.min + 1).max(1) as usize;

    // Seed the initial runs: non-chain items with a `Some` `tie_group`
    // collapse *unconditionally* (Phase 41 "equal facts collapse to equal
    // priority" — not just under band scarcity), by merging every maximal
    // run of consecutive items sharing the same `Some(tie_group)` into one
    // run up front. Everything else (chain items, and non-chain items
    // whose facts don't tie with a neighbor) starts as its own singleton
    // run, same as before.
    let mut runs: Vec<(usize, usize)> = Vec::with_capacity(n);
    let mut idx = 0;
    while idx < n {
        let mut end = idx;
        if let Some(tg) = items[idx].tie_group {
            while end + 1 < n && items[end + 1].tie_group == Some(tg) {
                end += 1;
            }
        }
        runs.push((idx, end));
        idx = end + 1;
    }

    let try_merge_tail_first =
        |runs: &mut Vec<(usize, usize)>, can_merge: &dyn Fn(usize, usize) -> bool| -> bool {
            let mut i = runs.len();
            while i > 1 {
                i -= 1;
                let (a_start, a_end) = runs[i - 1];
                let (b_start, _b_end) = runs[i];
                if can_merge(a_end, b_start) {
                    let (_, b_end) = runs[i];
                    runs[i - 1] = (a_start, b_end);
                    runs.remove(i);
                    return true;
                }
            }
            false
        };

    // Pass 1: within fine_group.
    while runs.len() > width {
        let merged = try_merge_tail_first(&mut runs, &|a_end, b_start| {
            items[a_end].fine_group == items[b_start].fine_group
        });
        if !merged {
            break;
        }
    }
    // Pass 2: within coarse_group (same chain, never non-chain).
    while runs.len() > width {
        let merged = try_merge_tail_first(&mut runs, &|a_end, b_start| {
            items[a_end].coarse_group.is_some()
                && items[a_end].coarse_group == items[b_start].coarse_group
        });
        if !merged {
            break;
        }
    }
    // Best-effort fallback: if the band is still narrower than the number
    // of irreducible groups (e.g. many distinct chains/criticality
    // buckets crammed into a tiny band), further compression would violate
    // the "never across chain/non-chain or criticality bucket" rule.
    // Clamp remaining low-priority runs into `band.min` rather than
    // escaping the band (never violate the band itself) — the clamp
    // introduces cross-chain/bucket TIES (never inversions: priorities
    // stay monotone non-increasing along the ranked order), and the
    // overflow is reported to the caller for a `BandTooNarrow` warning
    // rather than silently swallowed.
    let overflow = if runs.len() > width {
        // Labels for the runs that end up clamped into `band.min`: every
        // run at index >= width shares band.min with the width-1-indexed
        // run (the last one still on its own dense-rank priority). Name
        // the chain that owns each clamped run, or the non-chain bucket —
        // deduped, ranked order preserved.
        let mut clamped: Vec<String> = Vec::new();
        for (s, _) in runs.iter().skip(width) {
            let label = match &items[*s].coarse_group {
                Some(chain) => format!("chain '{chain}'"),
                None => "non-chain bucket".to_string(),
            };
            if !clamped.contains(&label) {
                clamped.push(label);
            }
        }
        Some(BandOverflow {
            distinct_classes: runs.len(),
            band_width: width,
            clamped,
        })
    } else {
        None
    };

    let mut priorities = vec![0i64; n];
    for (k, (s, e)) in runs.iter().enumerate() {
        let p = (band.max - k as i64).max(band.min);
        for slot in priorities.iter_mut().take(*e + 1).skip(*s) {
            *slot = p;
        }
    }
    (priorities, overflow)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        chain::{ChainSemantics, SegmentNode},
        mapper::{MapperNode, MapperRegistry},
        platform::PosixResources,
    };

    fn posix_facts(min: i64, max: i64) -> PlatformFacts {
        PlatformFacts::Posix(PosixResources {
            rt_priority_band: Some(PriorityBand { min, max }),
            isolated_cpus: vec![],
            rr_timeslice_us: None,
        })
    }

    fn seg(nodes: &[(&str, &str)]) -> ChainElement {
        ChainElement::Segment {
            nodes_in_topo_order: nodes
                .iter()
                .map(|(n, p)| SegmentNode {
                    node: n.to_string(),
                    path: p.to_string(),
                })
                .collect(),
        }
    }

    fn boundary(node: &str, path: &str, period_ms: f64) -> ChainElement {
        ChainElement::Boundary {
            node: node.to_string(),
            path: path.to_string(),
            period_ms,
            exec_ms: None,
        }
    }

    fn timer_path(name: &str, rate_hz: f64) -> MapperPath {
        MapperPath {
            name: name.to_string(),
            effective_trigger: EffectiveTrigger::Timer { rate_hz },
            max_latency_ms: None,
            exec_ms: None,
            inputs: vec![],
            outputs: vec![],
        }
    }

    fn input_path(name: &str, max_latency_ms: Option<f64>) -> MapperPath {
        MapperPath {
            name: name.to_string(),
            effective_trigger: EffectiveTrigger::Input(vec!["in".to_string()]),
            max_latency_ms,
            exec_ms: None,
            inputs: vec!["in".to_string()],
            outputs: vec![],
        }
    }

    fn once_path(name: &str) -> MapperPath {
        MapperPath {
            name: name.to_string(),
            effective_trigger: EffectiveTrigger::Once,
            max_latency_ms: None,
            exec_ms: None,
            inputs: vec![],
            outputs: vec![],
        }
    }

    /// Two nodes at 10 Hz (100 ms period), which is the case the default
    /// 100 ms RR slice cannot help.
    fn two_tied_nodes() -> MapperInput {
        MapperInput {
            nodes: vec![
                node("/a", None, vec![timer_path("p", 10.0)]),
                node("/b", None, vec![timer_path("p", 10.0)]),
            ],
            legacy: None,
            chains: Vec::new(),
        }
    }

    fn tied_priorities() -> BTreeMap<String, i64> {
        BTreeMap::from([("/a".to_string(), 30), ("/b".to_string(), 30)])
    }

    #[test]
    fn rr_is_derived_only_when_the_slice_is_shorter_than_the_period() {
        // 1 ms slice against a 100 ms period: rotating actually rotates.
        let (rr, warns) = rr_policy_for_ties(&tied_priorities(), &two_tied_nodes(), Some(1_000));
        assert_eq!(rr.len(), 2, "both tied nodes should get SCHED_RR");
        assert!(warns.is_empty(), "no tie left unmitigated: {warns:?}");
    }

    #[test]
    fn a_slice_as_long_as_the_period_derives_fifo_and_warns() {
        // Linux's DEFAULT slice is 100 ms and the nodes run at 10 Hz, so a
        // slice covers the whole period: SCHED_RR would behave exactly like
        // SCHED_FIFO while looking like it fixed starvation.
        let (rr, warns) = rr_policy_for_ties(&tied_priorities(), &two_tied_nodes(), Some(100_000));
        assert!(
            rr.is_empty(),
            "RR must not be derived when it changes nothing"
        );
        assert_eq!(warns.len(), 1);
        let MapWarning::UnmitigatedPriorityTie {
            priority,
            nodes,
            rr_timeslice_us,
            shortest_period_us,
        } = &warns[0]
        else {
            panic!("expected UnmitigatedPriorityTie, got {:?}", warns[0]);
        };
        assert_eq!(*priority, 30);
        assert_eq!(nodes, &vec!["/a".to_string(), "/b".to_string()]);
        // Both numbers reported, so the reader can see WHY it declined.
        assert_eq!(*rr_timeslice_us, Some(100_000));
        assert_eq!(*shortest_period_us, Some(100_000));
    }

    #[test]
    fn an_unknown_slice_never_assumes_the_default() {
        // `rr_timeslice_us` absent means the platform file did not say. That
        // is unknown, not 100 ms, and guessing either way would be inventing
        // a number.
        let (rr, warns) = rr_policy_for_ties(&tied_priorities(), &two_tied_nodes(), None);
        assert!(rr.is_empty());
        assert_eq!(warns.len(), 1);
        let MapWarning::UnmitigatedPriorityTie {
            rr_timeslice_us, ..
        } = &warns[0]
        else {
            panic!("expected UnmitigatedPriorityTie");
        };
        assert_eq!(*rr_timeslice_us, None);
    }

    #[test]
    fn distinct_priorities_are_not_ties() {
        let prios = BTreeMap::from([("/a".to_string(), 30), ("/b".to_string(), 31)]);
        let (rr, warns) = rr_policy_for_ties(&prios, &two_tied_nodes(), Some(1_000));
        assert!(rr.is_empty(), "no tie, so nothing to time-slice");
        assert!(warns.is_empty());
    }

    #[test]
    fn a_tie_with_no_timing_fact_cannot_be_judged_and_warns() {
        // Nothing to compare the slice against, so RR is declined rather than
        // guessed at.
        let input = MapperInput {
            nodes: vec![
                node("/a", None, vec![once_path("p")]),
                node("/b", None, vec![once_path("p")]),
            ],
            legacy: None,
            chains: Vec::new(),
        };
        let (rr, warns) = rr_policy_for_ties(&tied_priorities(), &input, Some(1_000));
        assert!(rr.is_empty());
        assert_eq!(warns.len(), 1);
        let MapWarning::UnmitigatedPriorityTie {
            shortest_period_us, ..
        } = &warns[0]
        else {
            panic!("expected UnmitigatedPriorityTie");
        };
        assert_eq!(*shortest_period_us, None);
    }

    fn node(name: &str, criticality: Option<Criticality>, paths: Vec<MapperPath>) -> MapperNode {
        MapperNode {
            name: name.to_string(),
            scope: "/".to_string(),
            rate_hz: None,
            deadline_us: None,
            criticality,
            path_budget_ms: None,
            paths,
        }
    }

    /// The design doc's worked example, reproduced verbatim: chain
    /// `sensing_to_actuation`, budget 150 ms, band 5-45. S1 =
    /// preproc->concat->detector, B1 = ekf 50 Hz, B2 = planning 10 Hz, S2 =
    /// follower->gate.
    fn worked_example_input() -> MapperInput {
        let chain = ResolvedChain {
            name: "sensing_to_actuation".to_string(),
            criticality: Criticality::High,
            max_latency_ms: 150.0,
            semantics: ChainSemantics::Reaction,
            elements: vec![
                seg(&[
                    ("/preproc", "p_preproc"),
                    ("/concat", "p_concat"),
                    ("/detector", "p_detector"),
                ]),
                boundary("/ekf", "p_ekf", 20.0),            // 50 Hz
                boundary("/planning", "p_planning", 100.0), // 10 Hz
                seg(&[("/follower", "p_follower"), ("/gate", "p_gate")]),
            ],
        };

        let nodes = vec![
            node("/preproc", Some(Criticality::High), vec![]),
            node("/concat", Some(Criticality::High), vec![]),
            node("/detector", Some(Criticality::High), vec![]),
            node("/ekf", Some(Criticality::High), vec![]),
            node("/planning", Some(Criticality::High), vec![]),
            node("/follower", Some(Criticality::High), vec![]),
            node("/gate", Some(Criticality::High), vec![]),
            node(
                "/sim_timer",
                Some(Criticality::Medium),
                vec![timer_path("p_sim", 40.0)],
            ),
            node("/loader", None, vec![once_path("p_loader")]),
        ];

        MapperInput {
            nodes,
            legacy: None,
            chains: vec![chain],
        }
    }

    fn prio_of<'a>(plan: &'a SchedPlan, node: &str) -> Option<(&'a str, i64)> {
        plan.tiers
            .iter()
            .find(|t| t.members.iter().any(|m| m == node))
            .map(|t| (t.name.as_str(), t.priority))
    }

    /// 45.10.b — the agnostic core is callable WITHOUT a platform / band and
    /// produces a priorityless ordering (highest-first), and the `posix`
    /// realizer over that ranking is byte-identical to the pre-split combined
    /// `map` (regression parity).
    #[test]
    fn chain_aware_rank_is_priorityless_and_split_is_parity() {
        let input = worked_example_input();

        // Core: no PlatformFacts, no band — just the ordering.
        let ranked = ChainAwareMapper.rank(&input);
        assert!(!ranked.items.is_empty());
        // Feasible chain → no ChainInfeasible warning at the core stage
        // (band-scarcity warnings only appear in the realizer).
        assert!(
            ranked
                .warnings
                .iter()
                .all(|w| !matches!(w, MapWarning::ChainInfeasible { .. })),
            "feasible worked example must not warn ChainInfeasible: {:?}",
            ranked.warnings
        );
        // Highest-priority item is the last segment's SINK (later segments
        // above earlier; drain-toward-sink within a segment).
        assert_eq!(ranked.items.first().unwrap().node, "/gate");
        // Segment membership rides on `fine_group` for the RTOS realizer.
        assert!(
            ranked
                .items
                .iter()
                .any(|it| it.node == "/follower" && it.fine_group == ranked.items[0].fine_group)
        );

        // Parity: realize_posix(core(input)) == the old combined path.
        let facts = posix_facts(5, 45);
        let (split_plan, split_diag) = realize_posix(&ranked, &input, &facts).expect("realize");
        let (combined_plan, combined_diag) = ChainAwareMapper
            .map_with_diagnostics(&input, &facts)
            .expect("map");
        assert_eq!(split_plan, combined_plan, "split plan must match combined");
        assert_eq!(
            split_diag, combined_diag,
            "split diagnostics must match combined"
        );
    }

    #[test]
    fn worked_example_matches_design_doc_ranks() {
        let mapper = ChainAwareMapper;
        let input = worked_example_input();
        let facts = posix_facts(5, 45);

        let (plan, diag) = mapper.map_with_diagnostics(&input, &facts).expect("maps");

        // Feasibility: sampling_cost = 20 + 100 = 120ms; controllable = 30ms > 0
        // => feasible, no ChainInfeasible warning.
        assert!(
            !diag
                .warnings
                .iter()
                .any(|w| matches!(w, MapWarning::ChainInfeasible { .. })),
            "chain is feasible: {:?}",
            diag.warnings
        );
        // ...but feasible ON INCOMPLETE EVIDENCE. The design doc's worked
        // example declares periods and deadlines, never WCETs, so both of its
        // boundaries were counted as zero execution time. That is worth pinning
        // rather than hiding: the canonical example is itself an instance of
        // the optimism nano-ros issue 0259 describes, and the 30 ms of
        // "controllable" slack above is an upper bound, not a measurement.
        let assumed: Vec<_> = diag
            .warnings
            .iter()
            .filter_map(|w| match w {
                MapWarning::ChainFeasibleWithoutWcet {
                    boundaries_without_wcet,
                    ..
                } => Some(boundaries_without_wcet.clone()),
                _ => None,
            })
            .collect();
        assert_eq!(
            assumed,
            vec![vec![
                "/ekf/p_ekf".to_string(),
                "/planning/p_planning".to_string()
            ]],
            "the worked example carries no WCETs; the verdict must say so"
        );

        // Pinned-exact per the design doc's worked example table.
        assert_eq!(prio_of(&plan, "/gate").map(|(_, p)| p), Some(45));
        assert_eq!(prio_of(&plan, "/follower").map(|(_, p)| p), Some(44));
        assert_eq!(prio_of(&plan, "/ekf").map(|(_, p)| p), Some(43));
        assert_eq!(prio_of(&plan, "/planning").map(|(_, p)| p), Some(42));
        assert_eq!(prio_of(&plan, "/detector").map(|(_, p)| p), Some(41));
        assert_eq!(prio_of(&plan, "/concat").map(|(_, p)| p), Some(40));
        assert_eq!(prio_of(&plan, "/preproc").map(|(_, p)| p), Some(39));

        // Non-chain: sim timer (40 Hz RM bucket) ranks below every chain
        // member but is still real-time; the map loader (`once`) is
        // fact-less -> SCHED_OTHER. The design doc states this region only
        // approximately ("sim 30-ish region"); we assert the relational
        // properties it pins down precisely and leave the exact residual-
        // band value as this implementation's documented interpretation
        // (see the W3 report's "Deviations" section).
        let (sim_tier, sim_prio) = prio_of(&plan, "/sim_timer").expect("sim_timer ranked");
        assert_ne!(sim_tier, DEFAULT_TIER);
        assert!(
            sim_prio < 39,
            "sim_timer must rank below the whole chain: {sim_prio}"
        );
        assert!(
            sim_prio >= 5,
            "sim_timer must stay inside the band: {sim_prio}"
        );

        let loader_tier = plan
            .tiers
            .iter()
            .find(|t| t.name == DEFAULT_TIER)
            .expect("default tier present");
        assert!(loader_tier.members.contains(&"/loader".to_string()));
        assert!(loader_tier.sched_class.is_none());

        // `--explain` detail: every chain member has a provenance string.
        let gate_detail = diag
            .details
            .iter()
            .find(|d| d.node == "/gate")
            .expect("gate has a detail entry");
        assert!(gate_detail.provenance.contains("chain_aware"));
        assert_eq!(gate_detail.priority, 45);
    }

    #[test]
    fn zero_chains_degrades_to_criticality_bucketed_rm_dm_with_tie_collapse() {
        let mapper = ChainAwareMapper;
        let input = MapperInput {
            nodes: vec![
                node("/a", Some(Criticality::High), vec![timer_path("p", 100.0)]),
                // Same rate as /a, same criticality -> must TIE (Phase 41
                // decision), unlike rate_monotonic's name-tiebreak spread.
                node("/b", Some(Criticality::High), vec![timer_path("p", 100.0)]),
                node(
                    "/c",
                    Some(Criticality::Low),
                    vec![input_path("p", Some(5.0))],
                ),
            ],
            legacy: None,
            chains: vec![],
        };
        let facts = posix_facts(10, 40);
        let plan = mapper.map(&input, &facts).expect("maps");

        let (_, prio_a) = prio_of(&plan, "/a").unwrap();
        let (_, prio_b) = prio_of(&plan, "/b").unwrap();
        let (_, prio_c) = prio_of(&plan, "/c").unwrap();

        assert_eq!(
            prio_a, prio_b,
            "equal facts must collapse to equal priority"
        );
        assert!(prio_a > prio_c, "High criticality bucket outranks Low");
    }

    #[test]
    fn shared_node_across_two_chains_takes_max_rank() {
        let mapper = ChainAwareMapper;

        let chain_hi = ResolvedChain {
            name: "hi".to_string(),
            criticality: Criticality::High,
            max_latency_ms: 1000.0,
            semantics: ChainSemantics::Reaction,
            elements: vec![seg(&[("/shared", "p1"), ("/hi_sink", "p2")])],
        };
        let chain_lo = ResolvedChain {
            name: "lo".to_string(),
            criticality: Criticality::Low,
            max_latency_ms: 1000.0,
            semantics: ChainSemantics::Reaction,
            elements: vec![seg(&[("/lo_sink", "p3"), ("/shared", "p4")])],
        };

        let input = MapperInput {
            nodes: vec![
                node("/shared", Some(Criticality::High), vec![]),
                node("/hi_sink", Some(Criticality::High), vec![]),
                node("/lo_sink", Some(Criticality::Low), vec![]),
            ],
            legacy: None,
            chains: vec![chain_hi, chain_lo],
        };
        let facts = posix_facts(5, 45);
        let plan = mapper.map(&input, &facts).expect("maps");

        // /shared appears in both chains: in `hi` (High criticality,
        // processed first) it's the drain-toward-sink tail (highest rank
        // in its segment); in `lo` (processed after `hi`, so strictly
        // lower ranks overall) it's the segment's SOURCE (lowest rank in
        // its segment). Node projection must take the max (the `hi`
        // appearance).
        let (_, shared_prio) = prio_of(&plan, "/shared").unwrap();
        let (_, lo_sink_prio) = prio_of(&plan, "/lo_sink").unwrap();
        assert!(
            shared_prio > lo_sink_prio,
            "/shared must keep its high-chain rank (max over paths), not its low-chain rank"
        );
    }

    /// Helper twin of `boundary` that CARRIES a measured WCET.
    fn boundary_with_wcet(node: &str, path: &str, period_ms: f64, exec_ms: f64) -> ChainElement {
        ChainElement::Boundary {
            node: node.to_string(),
            path: path.to_string(),
            period_ms,
            exec_ms: Some(exec_ms),
        }
    }

    /// nano-ros issue 0259 — a chain judged feasible while a boundary carries
    /// NO WCET says so. The sum still counts the missing hop as zero (there is
    /// nothing better to count), but "absent" and "zero" stop being the same
    /// answer: the verdict is optimistic by an unknown amount and reports it.
    #[test]
    fn feasible_chain_reports_boundaries_counted_as_zero() {
        let mapper = ChainAwareMapper;
        // budget 100ms against one 20ms-period boundary: comfortably feasible
        // ON THE EVIDENCE — but the boundary's execution time is unmeasured.
        let chain = ResolvedChain {
            name: "optimistic".to_string(),
            criticality: Criticality::High,
            max_latency_ms: 100.0,
            semantics: ChainSemantics::Reaction,
            elements: vec![boundary("/ekf", "p_ekf", 20.0)],
        };
        let input = MapperInput {
            nodes: vec![node(
                "/ekf",
                Some(Criticality::High),
                vec![timer_path("p_ekf", 50.0)],
            )],
            legacy: None,
            chains: vec![chain],
        };
        let facts = posix_facts(10, 40);
        let (_plan, diag) = mapper.map_with_diagnostics(&input, &facts).expect("maps");

        let reported: Vec<_> = diag
            .warnings
            .iter()
            .filter_map(|w| match w {
                MapWarning::ChainFeasibleWithoutWcet {
                    chain,
                    boundaries_without_wcet,
                } => Some((chain.clone(), boundaries_without_wcet.clone())),
                _ => None,
            })
            .collect();
        assert_eq!(
            reported,
            vec![("optimistic".to_string(), vec!["/ekf/p_ekf".to_string()])],
            "a feasible verdict computed with an absent WCET must say which hop it assumed away"
        );
    }

    /// The converse, so the warning cannot degrade into noise: once the WCET is
    /// supplied, the verdict rests on measured evidence and stays silent.
    #[test]
    fn feasible_chain_with_measured_wcet_is_silent() {
        let mapper = ChainAwareMapper;
        let chain = ResolvedChain {
            name: "measured".to_string(),
            criticality: Criticality::High,
            max_latency_ms: 100.0,
            semantics: ChainSemantics::Reaction,
            elements: vec![boundary_with_wcet("/ekf", "p_ekf", 20.0, 3.0)],
        };
        let input = MapperInput {
            nodes: vec![node(
                "/ekf",
                Some(Criticality::High),
                vec![timer_path("p_ekf", 50.0)],
            )],
            legacy: None,
            chains: vec![chain],
        };
        let facts = posix_facts(10, 40);
        let (_plan, diag) = mapper.map_with_diagnostics(&input, &facts).expect("maps");
        assert!(
            !diag
                .warnings
                .iter()
                .any(|w| matches!(w, MapWarning::ChainFeasibleWithoutWcet { .. })),
            "a WCET was supplied — nothing was assumed away: {:?}",
            diag.warnings
        );
    }

    #[test]
    fn infeasible_chain_excluded_members_keep_local_facts() {
        let mapper = ChainAwareMapper;
        // budget 10ms, one boundary of period 20ms alone already exceeds it.
        let chain = ResolvedChain {
            name: "doomed".to_string(),
            criticality: Criticality::High,
            max_latency_ms: 10.0,
            semantics: ChainSemantics::Reaction,
            elements: vec![boundary("/ekf", "p_ekf", 20.0)],
        };
        let input = MapperInput {
            nodes: vec![node(
                "/ekf",
                Some(Criticality::High),
                vec![timer_path("p_ekf", 50.0)],
            )],
            legacy: None,
            chains: vec![chain],
        };
        let facts = posix_facts(10, 40);
        let (plan, diag) = mapper.map_with_diagnostics(&input, &facts).expect("maps");

        assert_eq!(diag.warnings.len(), 1);
        assert!(matches!(
            &diag.warnings[0],
            MapWarning::ChainInfeasible { chain, .. } if chain == "doomed"
        ));
        // /ekf's own path (p_ekf, 50Hz timer) was never `covered` by the
        // excluded chain, so it still gets ranked via the non-chain path.
        let (_, ekf_prio) = prio_of(&plan, "/ekf").unwrap();
        assert_eq!(ekf_prio, 40);
    }

    #[test]
    fn band_narrower_than_ranks_collapses_within_segment_first() {
        let mapper = ChainAwareMapper;
        let input = worked_example_input();
        // Band width 7 (39..=45): the 8-item ranked sequence (7 chain items
        // + /sim_timer) can't fit 1:1 by exactly one slot; the merge is
        // resolved within S1 (preproc/concat/detector) before crossing
        // into the boundary run or the other segment (design step 5 /
        // issue #5: within-segment collapse first).
        let facts = posix_facts(39, 45);
        let plan = mapper.map(&input, &facts).expect("maps");

        for (name, prio) in [
            ("/gate", None::<i64>),
            ("/follower", None),
            ("/ekf", None),
            ("/planning", None),
            ("/detector", None),
            ("/concat", None),
            ("/preproc", None),
        ] {
            let (_, p) = prio_of(&plan, name).unwrap();
            assert!(
                (39..=45).contains(&p),
                "{name} priority {p} escaped band 39..=45"
            );
            let _ = prio;
        }

        // Within-segment collapse: two of S1's three members must now tie
        // (the segment absorbed the whole 1-rank deficit), while the other
        // chain groups (S2, the boundary run) keep their distinct ranks —
        // collapse happens within the smallest scope first.
        let (_, preproc) = prio_of(&plan, "/preproc").unwrap();
        let (_, concat) = prio_of(&plan, "/concat").unwrap();
        let (_, detector) = prio_of(&plan, "/detector").unwrap();
        let s1_prios: BTreeSet<i64> = [preproc, concat, detector].into_iter().collect();
        assert_eq!(
            s1_prios.len(),
            2,
            "S1's 3 members must collapse to exactly 2 distinct priorities \
             (one pair tied), not fewer/more: {preproc},{concat},{detector}"
        );

        // Meanwhile S2 (/gate, /follower) and the boundary run (/ekf,
        // /planning) remain fully distinct (they weren't the ones that
        // needed to collapse).
        let (_, gate) = prio_of(&plan, "/gate").unwrap();
        let (_, follower) = prio_of(&plan, "/follower").unwrap();
        let (_, ekf) = prio_of(&plan, "/ekf").unwrap();
        let (_, planning) = prio_of(&plan, "/planning").unwrap();
        assert_ne!(gate, follower);
        assert_ne!(ekf, planning);
    }

    #[test]
    fn deterministic_across_repeated_runs_and_input_permutation() {
        let mapper = ChainAwareMapper;
        let facts = posix_facts(5, 45);

        let input_a = worked_example_input();
        let (plan_a1, diag_a1) = mapper.map_with_diagnostics(&input_a, &facts).unwrap();
        let (plan_a2, diag_a2) = mapper.map_with_diagnostics(&input_a, &facts).unwrap();
        assert_eq!(
            plan_a1, plan_a2,
            "two runs on the same input must be identical"
        );
        assert_eq!(diag_a1, diag_a2);

        // Permute node declaration order.
        let mut input_b = worked_example_input();
        input_b.nodes.reverse();
        let (plan_b, _diag_b) = mapper.map_with_diagnostics(&input_b, &facts).unwrap();
        assert_eq!(
            plan_a1, plan_b,
            "permuting MapperInput.nodes must not change the derived plan"
        );
    }

    #[test]
    fn chain_aware_registered_in_with_builtins() {
        let registry = MapperRegistry::with_builtins();
        assert!(registry.get("chain_aware").is_some());
    }

    /// Mixed timer/input paths in ONE criticality bucket rank by the
    /// unified ascending time budget (RM = DM under implicit deadlines):
    /// 10 ms-period timer > 50 ms-deadline input > 100 ms-period timer.
    #[test]
    fn mixed_timer_and_input_bucket_ranks_by_unified_time_budget() {
        let mapper = ChainAwareMapper;
        let input = MapperInput {
            nodes: vec![
                // 100 ms period (10 Hz) — loosest of the three.
                node(
                    "/t_slow",
                    Some(Criticality::High),
                    vec![timer_path("p", 10.0)],
                ),
                // 50 ms declared deadline — outranks the 100 ms timer.
                node(
                    "/d_mid",
                    Some(Criticality::High),
                    vec![input_path("p", Some(50.0))],
                ),
                // 10 ms period (100 Hz) — outranks the 50 ms input path.
                node(
                    "/t_fast",
                    Some(Criticality::High),
                    vec![timer_path("p", 100.0)],
                ),
            ],
            legacy: None,
            chains: vec![],
        };
        let facts = posix_facts(10, 40);
        let plan = mapper.map(&input, &facts).expect("maps");

        let (_, t_fast) = prio_of(&plan, "/t_fast").unwrap();
        let (_, d_mid) = prio_of(&plan, "/d_mid").unwrap();
        let (_, t_slow) = prio_of(&plan, "/t_slow").unwrap();

        assert!(
            t_fast > d_mid,
            "10ms-period timer must outrank 50ms-deadline input: {t_fast} vs {d_mid}"
        );
        assert!(
            d_mid > t_slow,
            "50ms-deadline input must outrank 100ms-period timer: {d_mid} vs {t_slow}"
        );
    }

    /// A pathologically narrow band (width 1) that cannot hold the
    /// remaining classes after every LEGAL collapse (the worked example's
    /// chain + a Medium-criticality non-chain timer = 2 irreducible
    /// classes, since chain/non-chain never merge): the mapper must emit
    /// `BandTooNarrow` naming the counts, keep every priority in-band, and
    /// never INVERT order (ties only — monotone non-increasing along the
    /// ranked item sequence).
    #[test]
    fn pathologically_narrow_band_warns_and_never_inverts_order() {
        let mapper = ChainAwareMapper;
        let input = worked_example_input();
        let facts = posix_facts(45, 45); // width 1
        let (plan, diag) = mapper.map_with_diagnostics(&input, &facts).expect("maps");

        // The warning fires with the exact counts: after full legal
        // collapse the chain is 1 class and the non-chain sim timer is a
        // 2nd class that may not merge with it (chain/non-chain divide).
        let band_warnings: Vec<_> = diag
            .warnings
            .iter()
            .filter_map(|w| match w {
                MapWarning::BandTooNarrow {
                    distinct_classes,
                    band_width,
                    clamped,
                } => Some((*distinct_classes, *band_width, clamped.clone())),
                _ => None,
            })
            .collect();
        assert_eq!(
            band_warnings,
            vec![(2, 1, vec!["non-chain bucket".to_string()])],
            "expected exactly one BandTooNarrow(2 classes, width 1, clamped = the \
             non-chain sim-timer bucket): {:?}",
            diag.warnings
        );

        // Every ranked node stays inside the band (clamped, never escaped).
        for t in plan.tiers.iter().filter(|t| t.name != DEFAULT_TIER) {
            assert_eq!(
                t.priority, 45,
                "width-1 band: everything clamps to 45, got {} for {}",
                t.priority, t.name
            );
        }

        // Collapse never inverts order: along the diagnostics' ranked item
        // sequence, priorities are monotone non-increasing (cross-boundary
        // clamping introduces TIES only).
        let prios: Vec<i64> = diag.details.iter().map(|d| d.priority).collect();
        assert!(
            prios.windows(2).all(|w| w[0] >= w[1]),
            "priorities must be monotone non-increasing along rank order: {prios:?}"
        );
    }
}
