//! `play_launch measure` — turn a recorded run into declared costs (phase 58 W2).
//!
//! Reads `play_log/<ts>/interception/events.jsonl` plus the SystemModel that
//! run came from, and prints a platform-file fragment on stdout. It never
//! writes anything back: what a node is allowed to consume is the integrator's
//! decision, and a tool that edited the platform file would be making it.
//!
//! What comes out is what THIS machine did on THIS run. It is not a WCET, and
//! the header says so — the difference matters, because `budget_us` is used
//! downstream as an upper bound.

use eyre::{Context, bail};
use std::{
    collections::{BTreeMap, HashMap},
    path::{Path, PathBuf},
};

use crate::interception::measure::{self, NotMeasured, Outcome, PathResult, PathSpec, Run};
use ros_launch_manifest_model::SystemModel;

/// Arguments for `play_launch measure`.
#[derive(clap::Args)]
pub struct MeasureArgs {
    /// Run directory (`play_log/<timestamp>`), or the `interception/` dir
    /// inside one, or the `events.jsonl` file itself.
    pub run_dir: PathBuf,

    /// SystemModel the run was launched from — supplies the declared paths
    /// the measurements are attributed to.
    #[arg(long, value_name = "PATH")]
    pub model: PathBuf,
}

pub fn handle_measure(args: &MeasureArgs) -> eyre::Result<()> {
    let events_path = resolve_events_path(&args.run_dir)?;
    let body = std::fs::read_to_string(&events_path)
        .wrap_err_with(|| format!("reading {}", events_path.display()))?;
    let run = measure::parse_run(&body);

    let model_text = std::fs::read_to_string(&args.model)
        .wrap_err_with(|| format!("reading {}", args.model.display()))?;
    let model = SystemModel::from_yaml_str(&model_text)
        .wrap_err_with(|| format!("parsing {}", args.model.display()))?;

    let specs = path_specs(&model);
    if specs.is_empty() {
        bail!(
            "{} declares no node paths, so there is nothing to attribute measurements to.\n\
             Costs are measured per declared path (`paths:` on a node in the contract); \
             a model resolved without contracts carries none.",
            args.model.display()
        );
    }

    let results = measure::measure(&run, &specs);
    print!("{}", render(&results, &run, &events_path, &args.model));
    Ok(())
}

/// Accept a run directory, its `interception/` subdirectory, or the file.
fn resolve_events_path(arg: &Path) -> eyre::Result<PathBuf> {
    let candidates = [
        arg.to_path_buf(),
        arg.join("events.jsonl"),
        arg.join("interception/events.jsonl"),
    ];
    for c in candidates {
        if c.is_file() {
            return Ok(c);
        }
    }
    bail!(
        "no events.jsonl under {}.\n\
         `measure` reads the per-message record the interception layer writes; \
         enable it for the run with `interception.enabled: true` (and `stats: true`, \
         which produces the publish/take events) in the config passed to `--config`.",
        arg.display()
    )
}

/// Lower the model's node paths into measurable specs: endpoint refs resolved
/// to the topic FQNs the interception layer actually sees.
fn path_specs(model: &SystemModel) -> Vec<PathSpec> {
    // Endpoint ref (`/node/endpoint`) → topic FQN. The contract names
    // endpoints; the wire carries topics; remapping sits between them, and
    // `structure.topics` is where the resolver recorded the outcome.
    let mut endpoint_topic: HashMap<&str, &str> = HashMap::new();
    for (topic, wiring) in &model.structure.topics {
        for ep in wiring.publishers.iter().chain(wiring.subscribers.iter()) {
            endpoint_topic.insert(ep.as_str(), topic.as_str());
        }
    }

    let topics_of = |endpoints: &[String]| -> Vec<String> {
        endpoints
            .iter()
            .filter_map(|e| endpoint_topic.get(e.as_str()).map(|t| t.to_string()))
            .collect()
    };

    model
        .contracts
        .node_paths
        .iter()
        .map(|(key, path)| {
            let (node, name) = split_path_key(key);
            PathSpec {
                key: key.clone(),
                node: node.to_string(),
                name: name.to_string(),
                inputs: topics_of(&path.input),
                outputs: topics_of(&path.output),
            }
        })
        .collect()
}

/// `/safety/detector/detect` → (`/safety/detector`, `detect`).
fn split_path_key(key: &str) -> (&str, &str) {
    match key.rsplit_once('/') {
        Some((node, name)) => (node, name),
        None => ("", key),
    }
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

fn ms(ns: u64) -> String {
    format!("{:.2}", ns as f64 / 1_000_000.0)
}

fn render(results: &[PathResult], run: &Run, events_path: &Path, model_path: &Path) -> String {
    let mut out = String::new();
    out.push_str(&header(run, events_path, model_path));

    // Group by node: `overrides` is node-keyed, costs are per path.
    let mut by_node: BTreeMap<&str, Vec<&PathResult>> = BTreeMap::new();
    for r in results {
        by_node.entry(r.node.as_str()).or_default().push(r);
    }

    let mut measured_any = false;
    let mut body = String::new();
    for (node, paths) in &by_node {
        let Some(budget_us) = measure::node_budget_us(paths) else {
            continue;
        };
        measured_any = true;
        body.push_str(&format!("  {}:\n", node_key(node)));
        for r in paths {
            if let Outcome::Measured(s) = &r.outcome {
                body.push_str(&format!(
                    "    # {}: cost p50 {} p99 {} max {} ms   n={}\n",
                    r.name,
                    ms(s.cost.p50),
                    ms(s.cost.p99),
                    ms(s.cost.max),
                    s.samples,
                ));
                body.push_str(&format!(
                    "    #   response p50 {} p99 {} max {} ms (cost + preemption + wakeup)\n",
                    ms(s.response.p50),
                    ms(s.response.p99),
                    ms(s.response.max),
                ));
                if s.impossible_rejected > 0 {
                    // Same thread, but more CPU than wall clock — the two
                    // readings did not bracket one invocation. Usually a take
                    // whose message waited before the publish that answered it.
                    body.push_str(&format!(
                        "    #   {} pair(s) ignored: CPU time exceeded elapsed time, so the take \
                         and publish did not bracket a single invocation\n",
                        s.impossible_rejected
                    ));
                }
                if s.cross_thread_rejected > 0 {
                    body.push_str(&format!(
                        "    #   {} pair(s) ignored: take and publish ran on different threads, \
                         so their CPU counters do not subtract\n",
                        s.cross_thread_rejected
                    ));
                }
            }
        }
        let measured_paths = paths
            .iter()
            .filter(|r| matches!(r.outcome, Outcome::Measured(_)))
            .count();
        if measured_paths > 1 {
            body.push_str(
                "    # budget_us is the SUM of the per-path maxima: this node has more than one\n\
                 \x20   # measurable path and a trace cannot say whether they fire in the same\n\
                 \x20   # period. Conservative; split it if you know better.\n",
            );
        }
        // Phase 59: emit the canonical spelling. `measure` writes a fragment
        // an integrator pastes into a platform file, so emitting the
        // deprecated name would keep seeding new files with the form the
        // deprecation lint then flags.
        body.push_str(&format!("    budget: {budget_us}us\n"));
    }

    if measured_any {
        out.push_str("\noverrides:\n");
        out.push_str(&body);
    }

    let unmeasured: Vec<&PathResult> = results
        .iter()
        .filter(|r| matches!(r.outcome, Outcome::NotMeasured(_)))
        .collect();
    if !unmeasured.is_empty() {
        // Listed rather than omitted: an absent path reads as a path that
        // costs nothing, which is the one reading that must not happen.
        out.push_str("\n# Not measured — these carry real cost, it just wasn't observable:\n");
        for r in &unmeasured {
            let Outcome::NotMeasured(why) = &r.outcome else {
                continue;
            };
            out.push_str(&format!("#   {}: {}\n", r.key, explain(why)));
        }
    }

    if !measured_any {
        out.push_str("\n# Nothing was measurable in this run, so there is no fragment to paste.\n");
    }
    out
}

fn explain(why: &NotMeasured) -> String {
    match why {
        NotMeasured::NoInputTrigger => "timer-triggered — no input take to measure from. Its cost \
             starts inside the callback, where interception cannot see it."
            .to_string(),
        NotMeasured::Unstamped { messages } => format!(
            "{messages} message(s) carried no header.stamp, so publishes cannot be tied to the \
             takes that caused them"
        ),
        NotMeasured::NotObserved => {
            "declared, but no matching traffic in this run — not exercised".to_string()
        }
        NotMeasured::NoPairs { publishes, takes } => format!(
            "{publishes} publish(es) and {takes} take(s) seen, but none shared a stamp — the \
             declared cause-and-effect did not happen this way"
        ),
    }
}

/// Quote a node key when YAML would otherwise read it as something else.
/// Node FQNs start with `/`, which is plain-scalar-safe, but a name with a
/// colon is not.
fn node_key(node: &str) -> String {
    if node.contains(':') || node.is_empty() {
        format!("\"{node}\"")
    } else {
        node.to_string()
    }
}

fn header(run: &Run, events_path: &Path, model_path: &Path) -> String {
    let span_ns = {
        let times: Vec<u64> = run.events.iter().map(|e| e.t).collect();
        match (times.iter().min(), times.iter().max()) {
            (Some(lo), Some(hi)) => hi - lo,
            _ => 0,
        }
    };
    let mut h = format!(
        "# play_launch measure\n\
         #   run:   {}\n\
         #   model: {}\n\
         #   {} message events over {:.1} s\n",
        events_path.display(),
        model_path.display(),
        run.events.len(),
        span_ns as f64 / 1e9,
    );
    if run.malformed_lines > 0 {
        // One is normal — the file is written streaming, so a run that was
        // killed leaves a half-written final line.
        h.push_str(&format!(
            "#   {} unparseable line(s) skipped (a truncated last line is expected)\n",
            run.malformed_lines
        ));
    }
    h.push_str(
        "#\n\
         # budget_us is the observed MAXIMUM thread-CPU time from a path's input take to its\n\
         # output publish — a SCHED_DEADLINE runtime is CPU time, not elapsed time. p50/p99 are\n\
         # shown for judgement, not for pasting: under CBS an invocation that exceeds its runtime\n\
         # is throttled to the next replenishment, so a p99 budget turns the slowest 1% of\n\
         # invocations into a full-period stall.\n\
         #\n\
         # This is what this machine did on this run, under this load. It is NOT a WCET.\n\
         # Thread-CPU time also misses work a node hands to another thread.\n",
    );
    h
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interception::measure::{Dist, PathStats};

    fn measured(node: &str, name: &str, max_ns: u64) -> PathResult {
        PathResult {
            key: format!("{node}/{name}"),
            node: node.into(),
            name: name.into(),
            outcome: Outcome::Measured(PathStats {
                samples: 10,
                cost: Dist {
                    p50: max_ns / 2,
                    p99: max_ns,
                    max: max_ns,
                },
                response: Dist {
                    p50: max_ns,
                    p99: max_ns * 2,
                    max: max_ns * 2,
                },
                cross_thread_rejected: 0,
                impossible_rejected: 0,
            }),
        }
    }

    fn render_of(results: &[PathResult]) -> String {
        render(
            results,
            &Run::default(),
            Path::new("play_log/x/interception/events.jsonl"),
            Path::new("m.yaml"),
        )
    }

    #[test]
    fn fragment_is_pasteable_yaml_under_overrides() {
        let out = render_of(&[measured("/detector", "detect", 9_100_000)]);
        assert!(out.contains("overrides:\n  /detector:\n"), "{out}");
        assert!(out.contains("budget: 9100us"), "{out}");
        // The pasteable part must parse as YAML on its own.
        let yaml_only: String = out
            .lines()
            .filter(|l| !l.trim_start().starts_with('#'))
            .collect::<Vec<_>>()
            .join("\n");
        let v: serde_yaml_ng::Value = serde_yaml_ng::from_str(&yaml_only).unwrap();
        // Canonical spelling: the value is a duration string now, not a bare
        // number, which is the phase-59 point — a reader cannot mistake its
        // unit.
        assert_eq!(
            v["overrides"]["/detector"]["budget"].as_str(),
            Some("9100us")
        );
    }

    #[test]
    fn header_says_it_is_not_a_wcet() {
        let out = render_of(&[measured("/n", "p", 1_000_000)]);
        assert!(out.contains("NOT a WCET"), "{out}");
        assert!(out.contains("MAXIMUM"), "{out}");
    }

    #[test]
    fn response_is_reported_but_never_pasted() {
        let out = render_of(&[measured("/n", "p", 1_000_000)]);
        let response_line = out
            .lines()
            .find(|l| l.contains("response"))
            .expect("response reported");
        assert!(
            response_line.trim_start().starts_with('#'),
            "response must stay a comment — no field takes it: {response_line}"
        );
    }

    #[test]
    fn unmeasurable_paths_are_listed_with_reasons() {
        let out = render_of(&[
            measured("/n", "p", 1_000_000),
            PathResult {
                key: "/timer_node/tick".into(),
                node: "/timer_node".into(),
                name: "tick".into(),
                outcome: Outcome::NotMeasured(NotMeasured::NoInputTrigger),
            },
            PathResult {
                key: "/quiet/idle".into(),
                node: "/quiet".into(),
                name: "idle".into(),
                outcome: Outcome::NotMeasured(NotMeasured::NotObserved),
            },
        ]);
        assert!(out.contains("/timer_node/tick: timer-triggered"), "{out}");
        assert!(
            out.contains("/quiet/idle: declared, but no matching traffic"),
            "{out}"
        );
        // ...and they must NOT appear as an override with no budget.
        assert!(!out.contains("/timer_node:\n"), "{out}");
    }

    #[test]
    fn multi_path_node_sums_and_says_so() {
        let out = render_of(&[
            measured("/n", "a", 3_000_000),
            measured("/n", "b", 4_500_000),
        ]);
        assert!(out.contains("budget: 7500us"), "{out}");
        assert!(out.contains("SUM of the per-path maxima"), "{out}");
    }

    #[test]
    fn a_run_with_nothing_measurable_says_so_instead_of_printing_empty() {
        let out = render_of(&[PathResult {
            key: "/n/p".into(),
            node: "/n".into(),
            name: "p".into(),
            outcome: Outcome::NotMeasured(NotMeasured::Unstamped { messages: 42 }),
        }]);
        assert!(!out.contains("overrides:"), "{out}");
        assert!(out.contains("Nothing was measurable"), "{out}");
        assert!(
            out.contains("42 message(s) carried no header.stamp"),
            "{out}"
        );
    }

    #[test]
    fn path_key_splits_into_node_and_name() {
        assert_eq!(
            split_path_key("/safety/detector/detect"),
            ("/safety/detector", "detect")
        );
    }

    #[test]
    fn missing_events_file_names_the_config_that_produces_it() {
        let dir = tempfile::tempdir().unwrap();
        let err = resolve_events_path(dir.path()).unwrap_err().to_string();
        assert!(err.contains("interception.enabled: true"), "{err}");
    }
}
