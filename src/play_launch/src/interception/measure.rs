//! Measured execution cost — the pure core (phase 58 W2).
//!
//! `budget_us` in a platform file is a `SCHED_DEADLINE` **runtime**: CPU time
//! per period. Until W2 there was no way to obtain one, so a contract's
//! `max_latency_ms` — a *budget*, not a cost — was passed in its place, and
//! the mapper derived reservations from a number that never described
//! execution at all.
//!
//! The interception layer already timestamps every `rcl_publish`/`rcl_take`.
//! W2 adds a second reading, `CLOCK_THREAD_CPUTIME_ID`, and this module turns
//! the pair into two quantities that must not be confused:
//!
//! - **cost** — the thread's CPU time between a path's input take and its
//!   output publish. This is what `budget_us` takes.
//! - **response** — wall-clock over the same interval. Carries preemption,
//!   blocking and DDS wakeup latency. Reported beside cost, never as it; the
//!   gap between the two *is* the preemption.
//!
//! Everything here is pure: events and path declarations in, statistics out.
//! Reading files and rendering YAML live in `commands/measure.rs`.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ---------------------------------------------------------------------------
// The on-disk record — `play_log/<ts>/interception/events.jsonl`
// ---------------------------------------------------------------------------

/// One line of `events.jsonl`.
///
/// Two record kinds share the file because topic names are learned lazily:
/// the `.so` ships a topic's name in chunks, which can arrive after messages
/// on that topic have already been recorded. Writing hashes as they happen and
/// names whenever they resolve keeps the writer streaming (no buffering a
/// whole run in memory) at the cost of one map-building pass in the reader.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "r")]
pub enum Record {
    /// A publish or take.
    #[serde(rename = "e")]
    Event(RunEvent),
    /// `topic_hash` → topic FQN, as resolved by the name catalog.
    #[serde(rename = "t")]
    TopicName { h: u64, n: String },
}

/// Which side of the wire an event is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Dir {
    #[serde(rename = "pub")]
    Publish,
    #[serde(rename = "take")]
    Take,
}

/// A single publish or take, as recorded during a run.
///
/// Field names are one or two characters: this is per-message data, and a run
/// of any length writes a lot of lines.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunEvent {
    /// Node that owns the ring this came from. For a container ring this is
    /// the container, not the composable node inside it.
    pub n: String,
    /// Publish or take.
    pub d: Dir,
    /// FNV-1a hash of the topic FQN; resolved via [`Record::TopicName`].
    pub h: u64,
    /// `header.stamp.sec`.
    pub s: i32,
    /// `header.stamp.nanosec`.
    pub ns: u32,
    /// `CLOCK_MONOTONIC` nanoseconds.
    pub t: u64,
    /// `CLOCK_THREAD_CPUTIME_ID` nanoseconds.
    pub c: u64,
    /// Producing thread id — `cpu_ns` is per-thread, so a delta across two
    /// different tids is meaningless.
    pub tid: u32,
}

impl RunEvent {
    /// The message identity shared by a publish and the takes that receive
    /// it. `None` when the message carried no `header.stamp` (stamp 0), which
    /// cannot be correlated with anything.
    pub fn stamp_key(&self) -> Option<u64> {
        ((self.s != 0) || (self.ns != 0)).then(|| ((self.s as u32 as u64) << 32) | self.ns as u64)
    }
}

// ---------------------------------------------------------------------------
// What we are measuring against
// ---------------------------------------------------------------------------

/// One declared node path, with its endpoints already resolved to topic FQNs.
#[derive(Debug, Clone)]
pub struct PathSpec {
    /// `<node fqn>/<path name>`, the model's `contracts.node_paths` key.
    pub key: String,
    /// Owning node FQN.
    pub node: String,
    /// Human-readable path name (the part after the node FQN).
    pub name: String,
    /// Input topic FQNs. Empty means the path is not input-triggered.
    pub inputs: Vec<String>,
    /// Output topic FQNs.
    pub outputs: Vec<String>,
}

// ---------------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------------

/// Percentiles over one measured quantity, in nanoseconds.
#[derive(Debug, Clone, PartialEq)]
pub struct Dist {
    pub p50: u64,
    pub p99: u64,
    pub max: u64,
}

/// A path that produced usable measurements.
#[derive(Debug, Clone, PartialEq)]
pub struct PathStats {
    /// Number of take→publish pairs measured.
    pub samples: usize,
    /// Thread CPU time — the quantity `budget_us` wants.
    pub cost: Dist,
    /// Wall-clock over the same interval — cost plus everything that
    /// happened to the thread in between.
    pub response: Dist,
    /// Pairs matched by stamp but rejected because the take and the publish
    /// ran on different threads, so their CPU counters are unrelated.
    pub cross_thread_rejected: usize,
    /// Pairs rejected because the CPU delta exceeded the wall-clock delta —
    /// impossible for one continuous stretch of one thread, whatever the
    /// thread ids said. Counted separately from `cross_thread_rejected`
    /// because it means something different: same thread, but the two
    /// readings did not bracket a single invocation.
    pub impossible_rejected: usize,
}

/// Why a declared path produced no number — each of these is reported rather
/// than omitted, because an omission reads as "costs nothing".
#[derive(Debug, Clone, PartialEq)]
pub enum NotMeasured {
    /// No input endpoints: nothing to anchor the start of an invocation to.
    /// A timer-triggered path's cost is real but starts inside the callback,
    /// where interception cannot see it.
    NoInputTrigger,
    /// Messages were seen but carried no `header.stamp`, so a publish cannot
    /// be tied to the take that caused it.
    Unstamped { messages: usize },
    /// The path is declared but no matching traffic appeared in this run.
    NotObserved,
    /// Stamped traffic existed on both ends but no publish shared a stamp
    /// with a preceding take — the declared causal relationship did not
    /// happen the way the contract says it does.
    NoPairs { publishes: usize, takes: usize },
}

/// Per-path verdict.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    Measured(PathStats),
    NotMeasured(NotMeasured),
}

/// One path's result.
#[derive(Debug, Clone)]
pub struct PathResult {
    pub key: String,
    pub node: String,
    pub name: String,
    pub outcome: Outcome,
}

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

/// Parsed contents of an `events.jsonl` file.
#[derive(Debug, Default)]
pub struct Run {
    pub events: Vec<RunEvent>,
    /// topic_hash → FQN. Incomplete for topics whose name chunks never
    /// arrived; such events simply never match a declared path.
    pub topics: HashMap<u64, String>,
    /// Lines that failed to parse. A truncated final line is normal — the
    /// file is written streaming and a killed run leaves one — so this is
    /// surfaced as a count, not an error.
    pub malformed_lines: usize,
}

/// Parse an `events.jsonl` body. Never fails: unreadable lines are counted.
pub fn parse_run(body: &str) -> Run {
    let mut run = Run::default();
    for line in body.lines() {
        if line.trim().is_empty() {
            continue;
        }
        match serde_json::from_str::<Record>(line) {
            Ok(Record::Event(e)) => run.events.push(e),
            Ok(Record::TopicName { h, n }) => {
                run.topics.insert(h, n);
            }
            Err(_) => run.malformed_lines += 1,
        }
    }
    run
}

// ---------------------------------------------------------------------------
// Measurement
// ---------------------------------------------------------------------------

/// How far `cost` may exceed `response` before the pair is considered
/// impossible rather than merely imprecise.
///
/// The two clocks are read microseconds apart inside each hook, and thread CPU
/// accounting is not sampled at the same instant as `CLOCK_MONOTONIC`. For a
/// callback that is fully CPU-bound the two deltas are the *same interval*, so
/// the skew shows up as a cost a few hundred nanoseconds over the wall time.
/// Measured on rt_av_demo (43 such pairs on an 8 ms callback): 1 ns minimum,
/// 60 ns median, **621 ns maximum**.
///
/// Discarding those would be actively harmful: a CPU-bound invocation is
/// precisely the one whose cost is highest, so the rejected samples are the
/// tail that sets the budget. Within tolerance the two readings are taken as
/// equal; beyond it — orders of magnitude beyond, which is where genuinely
/// unrelated counters land — the pair is rejected.
const CLOCK_SKEW_TOLERANCE_NS: u64 = 10_000;

/// A publish paired with the take that caused it.
struct Pair {
    cost_ns: u64,
    response_ns: u64,
}

/// Measure every declared path against a run.
///
/// Results come back in `paths` order so the caller controls presentation.
pub fn measure(run: &Run, paths: &[PathSpec]) -> Vec<PathResult> {
    // Resolve topic FQN → hash once; the events carry hashes.
    let by_name: HashMap<&str, u64> = run.topics.iter().map(|(h, n)| (n.as_str(), *h)).collect();

    paths
        .iter()
        .map(|spec| PathResult {
            key: spec.key.clone(),
            node: spec.node.clone(),
            name: spec.name.clone(),
            outcome: measure_one(run, spec, &by_name),
        })
        .collect()
}

fn measure_one(run: &Run, spec: &PathSpec, by_name: &HashMap<&str, u64>) -> Outcome {
    if spec.inputs.is_empty() {
        return Outcome::NotMeasured(NotMeasured::NoInputTrigger);
    }

    let hashes = |topics: &[String]| -> Vec<u64> {
        topics
            .iter()
            .filter_map(|t| by_name.get(t.as_str()).copied())
            .collect()
    };
    let in_hashes = hashes(&spec.inputs);
    let out_hashes = hashes(&spec.outputs);

    // The ring's owning node is how we tell one process's events from
    // another's. Composable nodes share their container's ring, so a name
    // mismatch is not proof of absence — fall back to matching on topics
    // alone, which is still correct as long as two processes don't both
    // publish the same topic (in which case the model is already ambiguous).
    let node_matches: bool = run.events.iter().any(|e| e.n == spec.node);

    let relevant = |e: &RunEvent| !node_matches || e.n == spec.node;

    let mut takes: Vec<&RunEvent> = Vec::new();
    let mut publishes: Vec<&RunEvent> = Vec::new();
    let mut unstamped = 0usize;
    for e in &run.events {
        if !relevant(e) {
            continue;
        }
        let is_in = e.d == Dir::Take && in_hashes.contains(&e.h);
        let is_out = e.d == Dir::Publish && out_hashes.contains(&e.h);
        if !is_in && !is_out {
            continue;
        }
        if e.stamp_key().is_none() {
            unstamped += 1;
            continue;
        }
        if is_in {
            takes.push(e);
        } else {
            publishes.push(e);
        }
    }

    if takes.is_empty() && publishes.is_empty() {
        return Outcome::NotMeasured(if unstamped > 0 {
            NotMeasured::Unstamped {
                messages: unstamped,
            }
        } else {
            NotMeasured::NotObserved
        });
    }

    // Index takes by stamp. A stamp can be taken more than once (several
    // subscriptions, or a re-take); the one that started the invocation that
    // produced a given publish is the LATEST take at or before it.
    let mut takes_by_stamp: HashMap<u64, Vec<&RunEvent>> = HashMap::new();
    for t in &takes {
        takes_by_stamp
            .entry(t.stamp_key().unwrap())
            .or_default()
            .push(t);
    }
    for v in takes_by_stamp.values_mut() {
        v.sort_by_key(|e| e.t);
    }

    let mut pairs: Vec<Pair> = Vec::new();
    let mut cross_thread = 0usize;
    let mut impossible = 0usize;
    for p in &publishes {
        let stamp = p.stamp_key().unwrap();
        let Some(candidates) = takes_by_stamp.get(&stamp) else {
            continue;
        };
        let Some(t) = candidates.iter().rev().find(|t| t.t <= p.t) else {
            continue;
        };
        if t.tid != p.tid || t.tid == 0 {
            // Different threads: the CPU counters are unrelated, so their
            // difference is not a cost. Counted and reported rather than
            // silently dropped — it means the path did not run the way it
            // was declared.
            cross_thread += 1;
            continue;
        }
        let response_ns = p.t.saturating_sub(t.t);
        if p.c < t.c {
            // CPU time going backwards is not skew; it is a different thread's
            // counter, or a reading from before this thread existed.
            impossible += 1;
            continue;
        }
        let raw_cost_ns = p.c - t.c;
        // A thread cannot consume more CPU than the wall time it spans. Past
        // the skew tolerance that means the readings are not one continuous
        // stretch of one thread's execution, whatever the tids say.
        let cost_ns = if raw_cost_ns > response_ns {
            if raw_cost_ns - response_ns > CLOCK_SKEW_TOLERANCE_NS {
                impossible += 1;
                continue;
            }
            response_ns
        } else {
            raw_cost_ns
        };
        pairs.push(Pair {
            cost_ns,
            response_ns,
        });
    }

    if pairs.is_empty() {
        return Outcome::NotMeasured(NotMeasured::NoPairs {
            publishes: publishes.len(),
            takes: takes.len(),
        });
    }

    let mut costs: Vec<u64> = pairs.iter().map(|p| p.cost_ns).collect();
    let mut resps: Vec<u64> = pairs.iter().map(|p| p.response_ns).collect();
    costs.sort_unstable();
    resps.sort_unstable();

    Outcome::Measured(PathStats {
        samples: pairs.len(),
        cost: dist(&costs),
        response: dist(&resps),
        cross_thread_rejected: cross_thread,
        impossible_rejected: impossible,
    })
}

/// Percentiles over a sorted, non-empty slice.
///
/// Nearest-rank: the p-th percentile is the smallest observation at or above
/// p% of the sample. No interpolation — an interpolated value is a number that
/// was never observed, which is the wrong kind of number to hand someone as a
/// measurement.
fn dist(sorted: &[u64]) -> Dist {
    debug_assert!(!sorted.is_empty());
    let pick = |p: f64| -> u64 {
        let rank = (p * sorted.len() as f64).ceil() as usize;
        sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
    };
    Dist {
        p50: pick(0.50),
        p99: pick(0.99),
        max: *sorted.last().unwrap(),
    }
}

/// Sum of per-path maxima for one node, in microseconds.
///
/// `overrides` in a platform file is keyed by node, but cost is per path. A
/// node with two measurable paths has no single correct budget — whether both
/// fire within one period is not knowable from a trace — so this sums them.
/// Conservative, and the caller prints the breakdown so the assumption is
/// visible rather than buried.
pub fn node_budget_us(results: &[&PathResult]) -> Option<u64> {
    let total: u64 = results
        .iter()
        .filter_map(|r| match &r.outcome {
            Outcome::Measured(s) => Some(s.cost.max),
            Outcome::NotMeasured(_) => None,
        })
        .sum();
    (total > 0).then_some(total.div_ceil(1_000))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(n: &str, d: Dir, h: u64, stamp: u32, t: u64, c: u64, tid: u32) -> RunEvent {
        RunEvent {
            n: n.to_string(),
            d,
            h,
            s: stamp as i32,
            ns: 0,
            t,
            c,
            tid,
        }
    }

    fn spec() -> PathSpec {
        PathSpec {
            key: "/detector/detect".into(),
            node: "/detector".into(),
            name: "detect".into(),
            inputs: vec!["/scan".into()],
            outputs: vec!["/obstacles".into()],
        }
    }

    fn run_with(events: Vec<RunEvent>) -> Run {
        let mut topics = HashMap::new();
        topics.insert(1, "/scan".to_string());
        topics.insert(2, "/obstacles".to_string());
        Run {
            events,
            topics,
            malformed_lines: 0,
        }
    }

    #[test]
    fn pairs_take_and_publish_by_stamp() {
        let run = run_with(vec![
            ev("/detector", Dir::Take, 1, 100, 1_000, 500, 7),
            ev("/detector", Dir::Publish, 2, 100, 9_000, 8_500, 7),
        ]);
        let r = measure(&run, &[spec()]);
        let Outcome::Measured(s) = &r[0].outcome else {
            panic!("expected a measurement, got {:?}", r[0].outcome);
        };
        assert_eq!(s.samples, 1);
        // cost is the CPU delta (8_000), response the wall delta (8_000 too
        // here, since the fixture spends all its wall time running).
        assert_eq!(s.cost.max, 8_000);
        assert_eq!(s.response.max, 8_000);
    }

    #[test]
    fn cost_excludes_preemption_that_response_includes() {
        // 8 µs of CPU spread over 30 µs of wall clock: 22 µs preempted.
        let run = run_with(vec![
            ev("/detector", Dir::Take, 1, 100, 1_000, 500, 7),
            ev("/detector", Dir::Publish, 2, 100, 31_000, 8_500, 7),
        ]);
        let r = measure(&run, &[spec()]);
        let Outcome::Measured(s) = &r[0].outcome else {
            panic!("expected a measurement");
        };
        assert_eq!(s.cost.max, 8_000);
        assert_eq!(s.response.max, 30_000);
    }

    #[test]
    fn unrelated_stamps_do_not_pair() {
        let run = run_with(vec![
            ev("/detector", Dir::Take, 1, 100, 1_000, 500, 7),
            ev("/detector", Dir::Publish, 2, 999, 9_000, 8_500, 7),
        ]);
        let r = measure(&run, &[spec()]);
        assert_eq!(
            r[0].outcome,
            Outcome::NotMeasured(NotMeasured::NoPairs {
                publishes: 1,
                takes: 1
            })
        );
    }

    #[test]
    fn cross_thread_pair_is_rejected_not_measured() {
        // Same stamp, different threads: the CPU counters are unrelated, so
        // subtracting them would invent a cost.
        let run = run_with(vec![
            ev("/detector", Dir::Take, 1, 100, 1_000, 500, 7),
            ev("/detector", Dir::Publish, 2, 100, 9_000, 4_000_000, 9),
        ]);
        let r = measure(&run, &[spec()]);
        assert_eq!(
            r[0].outcome,
            Outcome::NotMeasured(NotMeasured::NoPairs {
                publishes: 1,
                takes: 1
            })
        );
    }

    #[test]
    fn cpu_exceeding_wall_time_is_rejected() {
        // Same tid, but 1 ms of CPU across 8 µs of wall clock is impossible —
        // the readings cannot be one continuous stretch of this thread.
        let run = run_with(vec![
            ev("/detector", Dir::Take, 1, 100, 1_000, 500, 7),
            ev("/detector", Dir::Publish, 2, 100, 9_000, 1_000_500, 7),
        ]);
        let r = measure(&run, &[spec()]);
        assert!(matches!(
            r[0].outcome,
            Outcome::NotMeasured(NotMeasured::NoPairs { .. })
        ));
    }

    #[test]
    fn a_cpu_bound_invocation_survives_clock_read_skew() {
        // 8 ms of CPU over 8 ms of wall clock, with the CPU reading 600 ns
        // ahead — the real, measured shape of a fully CPU-bound callback.
        // Rejecting it would throw away the most expensive samples, which are
        // the ones the budget is taken from.
        let run = run_with(vec![
            ev("/detector", Dir::Take, 1, 100, 1_000_000, 1_000_000, 7),
            ev("/detector", Dir::Publish, 2, 100, 9_000_000, 9_000_600, 7),
        ]);
        let r = measure(&run, &[spec()]);
        let Outcome::Measured(s) = &r[0].outcome else {
            panic!("expected a measurement, got {:?}", r[0].outcome);
        };
        assert_eq!(s.samples, 1);
        assert_eq!(s.impossible_rejected, 0);
        // Clamped to the wall time rather than reported above it: cost can
        // equal response, never exceed it.
        assert_eq!(s.cost.max, 8_000_000);
    }

    #[test]
    fn timer_triggered_path_is_named_not_omitted() {
        let mut s = spec();
        s.inputs.clear();
        let run = run_with(vec![ev("/detector", Dir::Publish, 2, 100, 9_000, 8_500, 7)]);
        let r = measure(&run, &[s]);
        assert_eq!(
            r[0].outcome,
            Outcome::NotMeasured(NotMeasured::NoInputTrigger)
        );
    }

    #[test]
    fn unstamped_traffic_is_named_not_omitted() {
        let run = run_with(vec![
            ev("/detector", Dir::Take, 1, 0, 1_000, 500, 7),
            ev("/detector", Dir::Publish, 2, 0, 9_000, 8_500, 7),
        ]);
        let r = measure(&run, &[spec()]);
        assert_eq!(
            r[0].outcome,
            Outcome::NotMeasured(NotMeasured::Unstamped { messages: 2 })
        );
    }

    #[test]
    fn declared_but_unexercised_path_is_named_not_omitted() {
        let run = run_with(vec![]);
        let r = measure(&run, &[spec()]);
        assert_eq!(r[0].outcome, Outcome::NotMeasured(NotMeasured::NotObserved));
    }

    #[test]
    fn latest_take_before_the_publish_wins() {
        // Same stamp taken twice (two subscriptions); the invocation that
        // published started at the second one.
        let run = run_with(vec![
            ev("/detector", Dir::Take, 1, 100, 1_000, 1_000, 7),
            ev("/detector", Dir::Take, 1, 100, 5_000, 5_000, 7),
            ev("/detector", Dir::Publish, 2, 100, 9_000, 9_000, 7),
        ]);
        let r = measure(&run, &[spec()]);
        let Outcome::Measured(s) = &r[0].outcome else {
            panic!("expected a measurement");
        };
        assert_eq!(s.cost.max, 4_000);
    }

    #[test]
    fn percentiles_are_nearest_rank() {
        let v: Vec<u64> = (1..=100).collect();
        let d = dist(&v);
        assert_eq!(d.p50, 50);
        assert_eq!(d.p99, 99);
        assert_eq!(d.max, 100);
    }

    #[test]
    fn max_is_what_a_budget_takes() {
        // 99 cheap invocations and one expensive one: p99 would throttle the
        // slow invocation every time it recurs, so the budget takes the max.
        let mut events = Vec::new();
        for i in 0..100u64 {
            let cost = if i == 42 { 50_000 } else { 1_000 };
            let stamp = (i + 1) as u32;
            events.push(ev(
                "/detector",
                Dir::Take,
                1,
                stamp,
                i * 1_000_000,
                i * 100_000,
                7,
            ));
            events.push(ev(
                "/detector",
                Dir::Publish,
                2,
                stamp,
                i * 1_000_000 + cost,
                i * 100_000 + cost,
                7,
            ));
        }
        let run = run_with(events);
        let r = measure(&run, &[spec()]);
        let Outcome::Measured(s) = &r[0].outcome else {
            panic!("expected a measurement");
        };
        assert_eq!(s.samples, 100);
        assert_eq!(s.cost.max, 50_000);
        assert_eq!(s.cost.p50, 1_000);
    }

    #[test]
    fn multi_path_node_budget_sums_maxima() {
        let mk = |name: &str, max: u64| PathResult {
            key: format!("/n/{name}"),
            node: "/n".into(),
            name: name.into(),
            outcome: Outcome::Measured(PathStats {
                samples: 1,
                cost: Dist {
                    p50: max,
                    p99: max,
                    max,
                },
                response: Dist {
                    p50: max,
                    p99: max,
                    max,
                },
                cross_thread_rejected: 0,
                impossible_rejected: 0,
            }),
        };
        let a = mk("a", 3_000_000);
        let b = mk("b", 4_500_000);
        assert_eq!(node_budget_us(&[&a, &b]), Some(7_500));
    }

    #[test]
    fn node_with_no_measured_path_has_no_budget() {
        let r = PathResult {
            key: "/n/a".into(),
            node: "/n".into(),
            name: "a".into(),
            outcome: Outcome::NotMeasured(NotMeasured::NotObserved),
        };
        assert_eq!(node_budget_us(&[&r]), None);
    }

    #[test]
    fn budget_rounds_up_so_it_never_under_declares() {
        let r = PathResult {
            key: "/n/a".into(),
            node: "/n".into(),
            name: "a".into(),
            outcome: Outcome::Measured(PathStats {
                samples: 1,
                cost: Dist {
                    p50: 1,
                    p99: 1,
                    max: 8_000_001,
                },
                response: Dist {
                    p50: 1,
                    p99: 1,
                    max: 8_000_001,
                },
                cross_thread_rejected: 0,
                impossible_rejected: 0,
            }),
        };
        assert_eq!(node_budget_us(&[&r]), Some(8_001));
    }

    #[test]
    fn parse_run_reads_events_and_names_and_tolerates_a_truncated_tail() {
        let body = concat!(
            r#"{"r":"t","h":1,"n":"/scan"}"#,
            "\n",
            r#"{"r":"e","n":"/detector","d":"take","h":1,"s":5,"ns":0,"t":10,"c":3,"tid":7}"#,
            "\n",
            r#"{"r":"e","n":"/detec"#,
        );
        let run = parse_run(body);
        assert_eq!(run.events.len(), 1);
        assert_eq!(run.topics.get(&1).map(String::as_str), Some("/scan"));
        assert_eq!(run.malformed_lines, 1);
        assert_eq!(run.events[0].d, Dir::Take);
    }

    #[test]
    fn record_roundtrips_through_json() {
        let e = ev("/n", Dir::Publish, 9, 3, 4, 5, 6);
        let line = serde_json::to_string(&Record::Event(e)).unwrap();
        let run = parse_run(&line);
        assert_eq!(run.events.len(), 1);
        assert_eq!(run.events[0].d, Dir::Publish);
        assert_eq!(run.events[0].tid, 6);
    }
}
