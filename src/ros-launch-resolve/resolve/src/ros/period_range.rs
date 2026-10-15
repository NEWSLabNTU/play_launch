//! A feasible range for each timer's period (phase 93 S5).
//!
//! A timer's period enters the fault-reaction arithmetic in several places:
//! a sampling hop waits one period, a `reported` detection waits for the
//! detector's next report, a window's owner notices the deadline on its
//! tick, and a measured budget can hold a `tick: { period }` term. Each
//! (hazard, rung) row with an FTTI therefore grows by `count x` any change
//! of the period, and the row's slack bounds how far the period may grow:
//!
//! - **upper bound**: the current period plus the smallest `slack / count`
//!   over the rows, cross-checked by re-running the checks a period reaches
//!   on a probe with that period (as `check_each_mode` does for a mode). A
//!   rule the probe trips first (a subscriber's `min_rate_hz`, a window's
//!   expiry, ...) bounds it instead, and so does a `max_age` the node
//!   evaluates on its own tick: an age limit checked once a period is not
//!   honoured by a tick slower than the limit.
//! - **lower bound**: the executor that runs the timer must keep up, `sum of
//!   work / period <= 1` over the timers it runs, with each timer's work
//!   taken from its measured `work` term (the terms beside a `tick` term of
//!   that period), its own measured or analysed `max_latency`, or the
//!   platform file's `budget_us`. Unknown when a timer on that executor
//!   states none.
//!
//! The counts are not written down per mechanism: they are read off the rows
//! themselves, by re-running the arithmetic with the period one millisecond
//! longer and differencing each row's DETECT, WINDOWS, ROUTE and SETTLE. That
//! keeps them right whenever the walk learns a new way for a period to enter.

use super::{
    manifest_loader::{ManifestIndex, RungBudget, RungRole, recheck_period_probe},
    value_rules::contract_node,
};
use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::{
    DetectMechanism, EffectiveTrigger, Manifest, OptBound, SourceKind, Trigger, duration::Duration,
};
use std::collections::BTreeMap;

/// The step the rows are differenced over, ms.
const STEP_MS: f64 = 1.0;
/// How far the search for a rule-bound upper limit looks when no row
/// charges the period, ms.
const SEARCH_CAP_MS: f64 = 60_000.0;
/// Bisection resolution, relative to the limit.
const RESOLUTION: f64 = 1e-5;

/// One row of the fault-reaction arithmetic the period enters.
#[derive(Debug, Clone)]
pub struct RowUse {
    pub hazard: String,
    pub rung: String,
    /// How many times one period is charged in the row's TOTAL.
    pub count: f64,
    /// Where: `detection 1`, `route 1`, ...
    pub parts: Vec<String>,
    /// The measured budgets with a `tick` term of this period in the row.
    pub tick_terms: Vec<String>,
    pub slack_ms: f64,
}

/// A named limit on a period.
#[derive(Debug, Clone)]
pub struct Limit {
    pub ms: f64,
    pub why: String,
}

/// The feasible period range of one timer path.
#[derive(Debug, Clone)]
pub struct PeriodRange {
    pub node: String,
    pub path: String,
    pub scope_id: usize,
    /// The contract key of the timer's rate (`nodes.n.paths.p.trigger.timer.rate_hz`).
    pub key: String,
    pub period_ms: f64,
    pub rows: Vec<RowUse>,
    /// Every upper limit found, smallest first.
    pub upper: Vec<Limit>,
    /// The lower limit, or why it is unknown.
    pub lower: Result<Limit, String>,
    /// The executor the lower bound was summed over.
    pub executor: String,
    /// The `source: derived` budgets a longer period would make stale (they
    /// follow the period and are recomputed, they do not limit it).
    pub follows: Vec<String>,
}

impl PeriodRange {
    /// The binding upper limit, if any.
    pub fn upper_ms(&self) -> Option<f64> {
        self.upper.first().map(|l| l.ms)
    }

    /// The binding lower limit, if known.
    pub fn lower_ms(&self) -> Option<f64> {
        self.lower.as_ref().ok().map(|l| l.ms)
    }
}

/// A timer path of the merged contracts.
#[derive(Debug, Clone)]
struct Timer {
    node: String,
    path: String,
    scope_id: usize,
    /// The node's name in its contract.
    local: String,
    period_ms: f64,
}

fn timers(index: &ManifestIndex) -> Vec<Timer> {
    let mut out: Vec<Timer> = index
        .node_paths
        .iter()
        .filter_map(|p| match p.path.effective_trigger() {
            EffectiveTrigger::Timer { rate_hz } if rate_hz > 0.0 => {
                let (scope_id, local, _) = contract_node(index, &p.node_fqn)?;
                Some(Timer {
                    node: p.node_fqn.clone(),
                    path: p.path_name.clone(),
                    scope_id,
                    local: local.to_string(),
                    period_ms: 1000.0 / rate_hz,
                })
            }
            _ => None,
        })
        .collect();
    out.sort_by(|a, b| (&a.node, &a.path).cmp(&(&b.node, &b.path)));
    out.dedup_by(|a, b| a.node == b.node && a.path == b.path);
    out
}

fn same(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// The budgets of `t`'s node that hold a `tick` term of `t`'s period: their
/// path names and the work beside the tick (the other terms' budgets).
fn tick_budgets(m: &Manifest, t: &Timer) -> Vec<(String, f64)> {
    let Some(nd) = m.nodes.get(&t.local) else {
        return Vec::new();
    };
    let sources = m.budget_sources();
    nd.paths
        .keys()
        .filter_map(|p| {
            let key = format!("nodes.{}.paths.{p}.max_latency", t.local);
            let obs = sources.get(&key)?.measured.as_ref()?;
            let ticks = obs
                .terms
                .iter()
                .filter(|x| {
                    x.period
                        .is_some_and(|d| same(d.as_millis_f64(), t.period_ms))
                })
                .count();
            (ticks > 0).then(|| {
                let work = obs
                    .terms
                    .iter()
                    .filter(|x| x.period.is_none())
                    .map(|x| x.budget_ms(obs.margin))
                    .sum();
                (p.clone(), work)
            })
        })
        .collect()
}

/// The callback work of a timer, and where it comes from.
fn work_of(
    index: &ManifestIndex,
    t: &Timer,
    platform: &BTreeMap<String, f64>,
) -> Option<(f64, String)> {
    let m = &index.manifests.get(&t.scope_id)?.manifest;
    if let Some((p, w)) = tick_budgets(m, t)
        .into_iter()
        .filter(|(_, w)| *w > 0.0)
        .reduce(|a, b| if b.1 > a.1 { b } else { a })
    {
        return Some((
            w,
            format!(
                "measured work beside the tick in nodes.{}.paths.{p}.max_latency",
                t.local
            ),
        ));
    }
    let key = format!("nodes.{}.paths.{}.max_latency", t.local, t.path);
    let own = m
        .nodes
        .get(&t.local)
        .and_then(|n| n.paths.get(&t.path))
        .and_then(|p| p.max_latency.ms());
    if let (Some(v), Some(src)) = (own, m.budget_sources().get(&key))
        && matches!(src.kind, SourceKind::Measured | SourceKind::Analysed)
    {
        return Some((v, format!("{key} ({})", src.kind)));
    }
    let bare = t.node.rsplit('/').next().unwrap_or(&t.node);
    platform
        .get(&t.node)
        .or_else(|| platform.get(bare))
        .map(|v| (*v, "the platform file's budget_us".to_string()))
}

/// Set `t`'s period to `q` on the probe, with what follows it: the timer's
/// rate in the declaration and the resolved path, a measured budget's tick
/// term of the old period, a derived `max_latency` equal to the period, and a
/// derived `within` equal to a budget that moved. Returns the keys moved.
fn set_period(probe: &mut ManifestIndex, t: &Timer, q: f64) -> Vec<String> {
    let d = q - t.period_ms;
    let mut moved: Vec<String> = Vec::new();
    let set_rate = |p: &mut ros_launch_manifest_types::PathDecl| {
        if let Some(Trigger::Timer { rate_hz, .. }) = &mut p.trigger {
            *rate_hz = 1000.0 / q;
        }
    };
    let shift = |b: &mut Option<ros_launch_manifest_types::Bound>, by: f64| {
        if let Some(b) = b
            && let Some(v) = b.ms()
        {
            b.value = Duration::from_millis_f64(v + by).into();
        }
    };
    let Some(rm) = probe.manifests.get_mut(&t.scope_id) else {
        return moved;
    };
    let m = &mut rm.manifest;
    let ticks: Vec<String> = tick_budgets(m, t).into_iter().map(|(p, _)| p).collect();
    let sources: BTreeMap<String, SourceKind> = m
        .budget_sources()
        .into_iter()
        .map(|(k, s)| (k, s.kind))
        .collect();
    let derived = |k: &str| sources.get(k) == Some(&SourceKind::Derived);
    let Some(nd) = m.nodes.get_mut(&t.local) else {
        return moved;
    };
    // (path, by): every path budget that moves.
    let mut paths: Vec<(String, f64, f64)> = Vec::new();
    for (name, p) in nd.paths.iter_mut() {
        let key = format!("nodes.{}.paths.{name}.max_latency", t.local);
        let old = p.max_latency.ms();
        if *name == t.path {
            set_rate(p);
            if derived(&key) && old.is_some_and(|v| same(v, t.period_ms)) {
                shift(&mut p.max_latency, d);
                moved.push(key);
                paths.push((name.clone(), old.unwrap_or(0.0), d));
                continue;
            }
        }
        if ticks.contains(name) {
            shift(&mut p.max_latency, d);
            moved.push(key);
            paths.push((name.clone(), old.unwrap_or(0.0), d));
        }
    }
    // A derived `within` that is its reaction path's budget follows it.
    for (ep, props) in nd.subscribers.iter_mut() {
        let key = format!("nodes.{}.sub.{ep}.on_violation.within", t.local);
        let Some(ov) = props.on_violation.as_mut() else {
            continue;
        };
        if let Some((_, _, by)) = paths.iter().find(|(p, old, _)| {
            *p == ov.reaction && derived(&key) && ov.within.ms().is_some_and(|w| same(w, *old))
        }) {
            shift(&mut ov.within, *by);
            moved.push(key);
        }
    }
    for np in probe.node_paths.iter_mut().filter(|p| p.node_fqn == t.node) {
        if np.path_name == t.path {
            set_rate(&mut np.path);
        }
        if let Some((_, _, by)) = paths.iter().find(|(p, _, _)| *p == np.path_name) {
            shift(&mut np.path.max_latency, *by);
        }
    }
    moved
}

/// Errors and warnings the probe's checks report, as (rule, path) -> message
/// (a warning such as `min-rate-mismatch` is the contract's own promise
/// broken, so it limits a period too), less the ones about a number the
/// probe moved disagreeing with its own provenance (`timer-param` on the
/// rate, `budget-rule` on a tick budget): a period chosen inside the range
/// moves its parameter and its observations with it.
fn errors(
    index: &ManifestIndex,
    t: &Timer,
    moved: &[String],
) -> BTreeMap<(String, String), String> {
    let trigger = format!("nodes.{}.paths.{}.trigger", t.local, t.path);
    index
        .merge_diagnostics
        .iter()
        .filter(|d| matches!(d.severity, Severity::Error | Severity::Warning))
        .filter(|d| !d.path.starts_with(&trigger) && !moved.contains(&d.path))
        .map(|d| {
            let sev = if d.severity == Severity::Error {
                "error"
            } else {
                "warning"
            };
            (
                (d.rule_id.clone(), d.path.clone()),
                format!("{sev}: {}", d.message),
            )
        })
        .collect()
}

/// The `source: derived` budget a finding at `path` is about, if any: a
/// number computed from others of the contract is recomputed when the period
/// moves, so it follows the period rather than limiting it.
fn derived_at(index: &ManifestIndex, path: &str) -> Option<String> {
    let mut ms: Vec<_> = index.manifests.values().collect();
    ms.sort_by_key(|m| m.scope_id);
    ms.into_iter().find_map(|m| {
        m.manifest
            .budget_sources()
            .into_iter()
            .find(|(k, src)| {
                src.kind == SourceKind::Derived && (k == path || k.starts_with(&format!("{path}.")))
            })
            .map(|(k, _)| k)
    })
}

/// A clone of `index` with the period-reaching checks re-run at period `q`.
fn probe_at(index: &ManifestIndex, t: &Timer, q: f64) -> (ManifestIndex, Vec<String>) {
    let mut probe = index.clone();
    probe.merge_diagnostics.clear();
    let moved = set_period(&mut probe, t, q);
    recheck_period_probe(&mut probe);
    (probe, moved)
}

/// The first finding at `q` that the unchanged probe does not report, and
/// the derived budgets that would have to be recomputed there.
fn new_error(
    index: &ManifestIndex,
    t: &Timer,
    q: f64,
    base: &BTreeMap<(String, String), String>,
) -> (Option<(String, String)>, Vec<String>) {
    let (probe, moved) = probe_at(index, t, q);
    let mut first = None;
    let mut follows = Vec::new();
    for ((rule, path), msg) in errors(&probe, t, &moved) {
        if base.contains_key(&(rule.clone(), path.clone())) {
            continue;
        }
        match derived_at(index, &path) {
            Some(k) => follows.push(k),
            None if first.is_none() => first = Some((rule, msg)),
            None => {}
        }
    }
    follows.sort();
    follows.dedup();
    (first, follows)
}

fn row_id(r: &RungBudget) -> (String, String, RungRole) {
    (r.hazard.clone(), r.rung.clone(), r.role)
}

fn checked(r: &RungBudget) -> bool {
    matches!(r.role, RungRole::Rung | RungRole::Floor)
        && r.total_ms.is_some()
        && r.ftti_ms.is_some()
}

/// How the rows grow with `t`'s period.
fn row_uses(index: &ManifestIndex, t: &Timer) -> (Vec<RowUse>, Vec<String>) {
    let (probe, moved) = probe_at(index, t, t.period_ms + STEP_MS);
    let mut out = Vec::new();
    for r in index.budgets.iter().filter(|r| checked(r)) {
        let Some(p) = probe
            .budgets
            .iter()
            .find(|p| row_id(p) == row_id(r) && checked(p))
        else {
            continue;
        };
        let slope =
            |a: Option<f64>, b: Option<f64>| (b.unwrap_or(0.0) - a.unwrap_or(0.0)) / STEP_MS;
        let count = slope(r.total_ms, p.total_ms);
        if count.abs() < 1e-9 {
            continue;
        }
        let mut parts = Vec::new();
        for (name, n) in [
            ("detection", slope(r.fdti_ms, p.fdti_ms)),
            ("windows", (p.windows_ms - r.windows_ms) / STEP_MS),
            ("route", slope(r.route_ms, p.route_ms)),
            ("settle", slope(r.settle_ms, p.settle_ms)),
        ] {
            if n.abs() >= 1e-9 {
                parts.push(format!("{name} {}", fmt_n(n)));
            }
        }
        let mut tick_terms: Vec<String> = r
            .terms
            .iter()
            .filter(|k| k.0 == t.scope_id && moved.contains(&k.1))
            .map(|k| k.1.clone())
            .collect();
        tick_terms.dedup();
        out.push(RowUse {
            hazard: r.hazard.clone(),
            rung: r.rung.clone(),
            count,
            parts,
            tick_terms,
            slack_ms: r.ftti_ms.unwrap_or(0.0) - r.total_ms.unwrap_or(0.0),
        });
    }
    (out, moved)
}

fn fmt_n(n: f64) -> String {
    if same(n, n.round()) {
        format!("{}", n.round() as i64)
    } else {
        format!("{n:.2}")
    }
}

fn fmt_ms(v: f64) -> String {
    let s = format!("{v:.2}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// The `max_age` limits a node evaluates on its own clock: an age checked
/// once a period is not honoured by a slower tick. The clock is `t` when the
/// node has no other timer, or when the reaction's budget holds a tick of
/// `t`'s period (the reaction is served on that tick).
fn age_limits(index: &ManifestIndex, t: &Timer) -> Vec<Limit> {
    let Some(m) = index.manifests.get(&t.scope_id).map(|m| &m.manifest) else {
        return Vec::new();
    };
    let Some(nd) = m.nodes.get(&t.local) else {
        return Vec::new();
    };
    let only_timer = nd
        .paths
        .values()
        .filter(|p| matches!(p.trigger, Some(Trigger::Timer { .. })))
        .count()
        == 1;
    let ticked: Vec<String> = tick_budgets(m, t).into_iter().map(|(p, _)| p).collect();
    nd.subscribers
        .iter()
        .filter(|(_, props)| {
            only_timer
                || props
                    .on_violation
                    .as_ref()
                    .is_some_and(|ov| ticked.contains(&ov.reaction))
        })
        .filter_map(|(ep, props)| {
            let ov = props.on_violation.as_ref()?;
            if !matches!(
                ov.mechanism,
                DetectMechanism::Application | DetectMechanism::Diagnostics
            ) {
                return None;
            }
            let age = props.max_age.ms()?;
            Some(Limit {
                ms: age,
                why: format!(
                    "max_age detection: nodes.{}.sub.{ep}.max_age {}ms is evaluated by the \
                     node on its own tick (detection up to max_age + one period)",
                    t.local,
                    fmt_ms(age)
                ),
            })
        })
        .collect()
}

/// The largest period in `(lo, hi]` the probe's checks accept, by bisection,
/// with the first new error above it; `None` when `hi` itself passes.
fn search(
    index: &ManifestIndex,
    t: &Timer,
    lo: f64,
    hi: f64,
    base: &BTreeMap<(String, String), String>,
) -> Option<(f64, String, String)> {
    let mut fail = new_error(index, t, hi, base).0?;
    // Bracket by doubling from the current period, so a limit near it is
    // not bisected down from the far end.
    let (mut lo, mut hi) = (lo, hi);
    let mut x = lo * 2.0;
    while x < hi {
        match new_error(index, t, x, base).0 {
            Some(e) => {
                fail = e;
                hi = x;
            }
            None => lo = x,
        }
        if hi == x {
            break;
        }
        x *= 2.0;
    }
    while hi - lo > RESOLUTION * hi {
        let mid = (lo + hi) / 2.0;
        match new_error(index, t, mid, base).0 {
            Some(e) => {
                fail = e;
                hi = mid;
            }
            None => lo = mid,
        }
    }
    Some(((lo + hi) / 2.0, fail.0, fail.1))
}

/// The range of one timer.
fn range_of(
    index: &ManifestIndex,
    t: &Timer,
    all: &[Timer],
    platform: &BTreeMap<String, f64>,
) -> PeriodRange {
    let (rows, _) = row_uses(index, t);
    let mut upper: Vec<Limit> = Vec::new();
    for r in rows.iter().filter(|r| r.count > 1e-9) {
        let via = if r.tick_terms.is_empty() {
            String::new()
        } else {
            format!("; tick term of {}", r.tick_terms.join(", "))
        };
        upper.push(Limit {
            ms: t.period_ms + r.slack_ms / r.count,
            why: format!(
                "FTTI: hazard '{}' rung '{}', slack {}ms / {} period(s) ({}{via})",
                r.hazard,
                r.rung,
                fmt_ms(r.slack_ms),
                fmt_n(r.count),
                r.parts.join(", ")
            ),
        });
    }
    upper.extend(age_limits(index, t));
    upper.sort_by(|a, b| a.ms.total_cmp(&b.ms));

    // Cross-check: the checks a period reaches, re-run at the candidate. A
    // rule that trips first is a limit of its own. What the probes compare
    // against is what holds at half the period too, so a finding the
    // current period already causes (a promise it already breaks) is a
    // limit below it, not background.
    let lo = t.period_ms / 2.0;
    let base = {
        let (now, moved) = probe_at(index, t, t.period_ms);
        let now = errors(&now, t, &moved);
        let (half, moved) = probe_at(index, t, lo);
        let half = errors(&half, t, &moved);
        now.into_iter()
            .filter(|(k, _)| half.contains_key(k))
            .collect::<BTreeMap<_, _>>()
    };
    let hi = upper.first().map_or(SEARCH_CAP_MS, |l| l.ms - 1e-6);
    if hi > lo
        && let Some((ms, rule, msg)) = search(index, t, lo, hi, &base)
    {
        upper.push(Limit {
            ms,
            why: format!("rule `{rule}` above it: {msg}"),
        });
        upper.sort_by(|a, b| a.ms.total_cmp(&b.ms));
    }
    let follows = upper
        .first()
        .filter(|l| l.ms > t.period_ms)
        .map(|l| new_error(index, t, l.ms, &base).1)
        .unwrap_or_default();

    // Lower: the executor keeps up with every timer it runs.
    let exec = |node: &str| {
        index
            .executor_of
            .get(node)
            .cloned()
            .unwrap_or_else(|| node.to_string())
    };
    let executor = exec(&t.node);
    let lower = (|| {
        let (w, from) = work_of(index, t, platform)
            .ok_or_else(|| format!("unknown: {}/{} states no work", t.node, t.path))?;
        let mut others = 0.0;
        for o in all
            .iter()
            .filter(|o| exec(&o.node) == executor && !(o.node == t.node && o.path == t.path))
        {
            let (wo, _) = work_of(index, o, platform).ok_or_else(|| {
                format!(
                    "unknown: {}/{} runs on the same executor and states no work",
                    o.node, o.path
                )
            })?;
            others += wo / o.period_ms;
        }
        if others >= 1.0 {
            return Err(format!(
                "infeasible: the other timers on {executor} already use {:.0}% of it",
                others * 100.0
            ));
        }
        Ok(Limit {
            ms: w / (1.0 - others),
            why: if others > 0.0 {
                format!(
                    "utilization of {executor}: work {}ms ({from}) over the {:.1}% the other \
                     timers leave",
                    fmt_ms(w),
                    (1.0 - others) * 100.0
                )
            } else {
                format!("utilization of {executor}: work {}ms ({from})", fmt_ms(w))
            },
        })
    })();

    PeriodRange {
        node: t.node.clone(),
        path: t.path.clone(),
        scope_id: t.scope_id,
        key: format!("nodes.{}.paths.{}.trigger.timer.rate_hz", t.local, t.path),
        period_ms: t.period_ms,
        rows,
        upper,
        lower,
        executor,
        follows,
    }
}

/// Every timer's range, into `index.period_ranges` (phase 93 S5), and
/// `period-range` on an allocated period outside it.
pub(crate) fn check_period_ranges(index: &mut ManifestIndex) {
    index.period_ranges = ranges(index, &BTreeMap::new());
    refuse_outside(index);
}

/// Whether `r`'s rate states `source: allocated`: a value chosen inside the
/// range this module derives, which the checker therefore holds to it.
fn allocated(index: &ManifestIndex, r: &PeriodRange) -> bool {
    index
        .manifests
        .get(&r.scope_id)
        .and_then(|m| {
            m.manifest
                .budget_sources()
                .get(&r.key)
                .map(|s| s.kind.as_str() == "allocated")
        })
        .unwrap_or(false)
}

/// `period-range`: an allocated period outside its feasible range.
fn refuse_outside(index: &mut ManifestIndex) {
    let mut diags = Vec::new();
    for r in index.period_ranges.iter().filter(|r| allocated(index, r)) {
        let site = super::value_rules::site(index, r.scope_id, &r.key);
        let problem = match (&r.lower, r.upper.first()) {
            (Ok(l), _) if r.period_ms + 1e-6 < l.ms => Some((
                Severity::Error,
                format!(
                    "below the shortest the executor sustains, {}ms ({})",
                    fmt_ms(l.ms),
                    l.why
                ),
            )),
            (_, Some(u)) if r.period_ms > u.ms + 1e-6 => Some((
                Severity::Error,
                format!(
                    "above the longest that holds, {}ms ({})",
                    fmt_ms(u.ms),
                    u.why
                ),
            )),
            // Inside what is known, with the lower end unknown: said, not
            // refused.
            (Err(why), _) => Some((
                Severity::Warning,
                format!(
                    "and the shortest the executor sustains is {why}; state the work of every \
                     timer on {} to check the allocation from below",
                    r.executor
                ),
            )),
            _ => None,
        };
        if let Some((severity, problem)) = problem {
            diags.push(Diagnostic {
                rule_id: "period-range".to_string(),
                severity,
                message: format!(
                    "{site}: {}/{} is allocated a period of {}ms, {problem}",
                    r.node,
                    r.path,
                    fmt_ms(r.period_ms)
                ),
                path: r.key.clone(),
                span: None,
            });
        }
    }
    index.merge_diagnostics.extend(diags);
}

/// The lower bounds again, with a platform file's per-node budgets (ms) for
/// the timers whose contract states no work.
pub fn with_platform_budgets(index: &mut ManifestIndex, budgets_us: &BTreeMap<String, u64>) {
    if budgets_us.is_empty() {
        return;
    }
    let platform: BTreeMap<String, f64> = budgets_us
        .iter()
        .map(|(k, us)| (k.clone(), *us as f64 / 1000.0))
        .collect();
    index.period_ranges = ranges(index, &platform);
    index
        .merge_diagnostics
        .retain(|d| d.rule_id != "period-range");
    refuse_outside(index);
}

fn ranges(index: &ManifestIndex, platform: &BTreeMap<String, f64>) -> Vec<PeriodRange> {
    let all = timers(index);
    all.iter()
        .map(|t| range_of(index, t, &all, platform))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::{launch_dump::LaunchDump, manifest_loader::ContractSources};
    use std::path::{Path, PathBuf};

    fn fixture(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("the repository root")
            .join("tests/fixtures")
            .join(name)
            .join("launch/bringup.launch.xml")
    }

    fn index_of(name: &str) -> ManifestIndex {
        let launch = fixture(name);
        let record = crate::verbs::parse_launch_file(&launch, Default::default())
            .unwrap_or_else(|e| panic!("parse {}: {e}", launch.display()));
        let dump: LaunchDump = serde_json::from_str(&serde_json::to_string(&record).expect("json"))
            .expect("a LaunchDump");
        crate::ros::manifest_loader::load_manifests(
            &dump,
            &ContractSources {
                overlay: None,
                provider: true,
            },
        )
        .unwrap_or_else(|e| panic!("{name}: load manifests: {e}"))
    }

    fn range<'a>(index: &'a ManifestIndex, timer: &str) -> &'a PeriodRange {
        index
            .period_ranges
            .iter()
            .find(|r| format!("{}/{}", r.node, r.path) == timer)
            .unwrap_or_else(|| panic!("no range for {timer}: {:#?}", index.period_ranges))
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 0.01
    }

    /// The watchdog's tick enters the row once, through the tick term of the
    /// reaction's measured budget; the FTTI allows 404, and the `max_age`
    /// it checks on that tick binds first at 300. Its work (24, beside the
    /// tick) over the 90 % `report` leaves gives 26.67.
    #[test]
    fn a_tick_term_counts_once_and_max_age_binds_first() {
        let index = index_of("contract_period_range");
        let r = range(&index, "/safety/watchdog/on_timer");
        assert!(close(r.period_ms, 100.0));
        assert_eq!(r.rows.len(), 1, "{r:#?}");
        let row = &r.rows[0];
        assert!(close(row.count, 1.0), "{row:#?}");
        assert!(close(row.slack_ms, 304.0), "{row:#?}");
        assert_eq!(
            row.tick_terms,
            vec!["nodes.watchdog.paths.react.max_latency".to_string()]
        );
        let max = r.upper.first().expect("an upper limit");
        assert!(close(max.ms, 300.0), "{r:#?}");
        assert!(max.why.starts_with("max_age detection"), "{}", max.why);
        assert!(
            r.upper
                .iter()
                .any(|l| close(l.ms, 404.0) && l.why.starts_with("FTTI")),
            "{r:#?}"
        );
        let min = r.lower.as_ref().expect("a lower limit");
        assert!(close(min.ms, 24.0 / 0.9), "{min:#?}");
    }

    /// The brake's tick is a sampling hop: once in the row, so the FTTI
    /// allows 354; its own `min_rate_hz: 15` promise binds first at 66.67,
    /// found by the probe rather than by the arithmetic.
    #[test]
    fn a_sampling_hop_counts_once_and_a_promise_found_by_the_probe_binds() {
        let index = index_of("contract_period_range");
        let r = range(&index, "/safety/brake/on_timer");
        assert_eq!(r.rows.len(), 1, "{r:#?}");
        assert!(close(r.rows[0].count, 1.0));
        assert_eq!(r.rows[0].parts, vec!["route 1".to_string()]);
        let max = r.upper.first().expect("an upper limit");
        assert!(close(max.ms, 1000.0 / 15.0), "{r:#?}");
        assert!(max.why.contains("`min-rate-mismatch`"), "{}", max.why);
        assert!(r.upper.iter().any(|l| close(l.ms, 354.0)), "{r:#?}");
        assert!(close(r.lower_ms().expect("a lower limit"), 10.0));
    }

    /// A timer no row charges and no check limits has no upper limit, and
    /// still a lower one from its executor.
    #[test]
    fn an_unconstrained_timer_has_no_upper_limit() {
        let index = index_of("contract_period_range");
        let r = range(&index, "/safety/watchdog/report");
        assert!(r.rows.is_empty(), "{r:#?}");
        assert!(r.upper.is_empty(), "{r:#?}");
        assert!(close(r.lower_ms().expect("a lower limit"), 20.0 / 0.76));
    }

    /// A timer whose work nothing states leaves the lower limit unknown, and
    /// so does every other timer on its executor.
    #[test]
    fn a_timer_with_no_work_leaves_its_executor_unknown() {
        let mut index = index_of("contract_period_range");
        let m = index.manifests.values_mut().next().expect("the contract");
        let report = m
            .manifest
            .nodes
            .get_mut("watchdog")
            .and_then(|n| n.paths.get_mut("report"))
            .expect("report");
        report.max_latency = None;
        let ranges = ranges(&index, &BTreeMap::new());
        let r = ranges
            .iter()
            .find(|r| r.path == "on_timer" && r.node.ends_with("watchdog"))
            .expect("the watchdog's tick");
        let why = r.lower.as_ref().expect_err("unknown");
        assert!(why.contains("/safety/watchdog/report"), "{why}");
        // A platform file's budget fills the gap.
        let platform: BTreeMap<String, f64> = [("/safety/watchdog".to_string(), 20.0)].into();
        let ranges = super::ranges(&index, &platform);
        let r = ranges
            .iter()
            .find(|r| r.path == "on_timer" && r.node.ends_with("watchdog"))
            .expect("the watchdog's tick");
        assert!(close(r.lower_ms().expect("known"), 24.0 / 0.9), "{r:#?}");
    }

    /// A period that already breaks a promise is not measured against
    /// itself: the limit the promise sets is found below it.
    #[test]
    fn a_promise_the_period_already_breaks_is_still_a_limit() {
        let mut index = index_of("contract_period_range");
        let brake = timers(&index)
            .into_iter()
            .find(|t| t.node == "/safety/brake")
            .expect("the brake's tick");
        set_period(&mut index, &brake, 100.0);
        let ranges = ranges(&index, &BTreeMap::new());
        let r = ranges
            .iter()
            .find(|r| r.node == "/safety/brake")
            .expect("the brake's range");
        assert!(close(r.period_ms, 100.0), "{r:#?}");
        let max = r.upper.first().expect("an upper limit");
        assert!(close(max.ms, 1000.0 / 15.0), "{r:#?}");
        assert!(max.why.contains("`min-rate-mismatch`"), "{}", max.why);
    }

    /// `source: allocated` holds a rate to its range: the brake's 10 Hz is
    /// past its own 15 Hz promise and refused, the watchdog's 10 Hz is inside
    /// [26.67, 300] and passes.
    #[test]
    fn an_allocated_period_outside_its_range_is_refused() {
        let index = index_of("contract_period_range_allocated");
        let found: Vec<&Diagnostic> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "period-range")
            .collect();
        assert_eq!(found.len(), 1, "{found:#?}");
        let d = found[0];
        assert_eq!(d.severity, Severity::Error);
        assert_eq!(d.path, "nodes.brake.paths.on_timer.trigger.timer.rate_hz");
        assert!(
            d.message.contains(
                "/safety/brake/on_timer is allocated a period of 100ms, above the longest \
                 that holds, 66.67ms"
            ),
            "{}",
            d.message
        );
    }

    /// `--explain` prints the range and the limits by name.
    #[test]
    fn explain_names_the_limits() {
        let index = index_of("contract_period_range");
        let out = crate::verbs::check::render_periods(&index);
        assert!(out.contains("-- Timer periods (--explain, ms) --"), "{out}");
        assert!(
            out.contains("/safety/watchdog/on_timer: MAX 300.00 <- max_age detection"),
            "{out}"
        );
        assert!(out.contains("/safety/brake/on_timer: MAX 66.67"), "{out}");
        assert!(out.contains("/safety/watchdog/report: MAX none"), "{out}");
    }
}
