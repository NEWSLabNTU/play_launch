//! Attribute a `DiagnosticStatus` to a member play_launch spawned.
//!
//! Phase 62 W2 wants a diagnostics badge on each node card, which needs a join
//! from a diagnostic to a member. Nothing in ROS requires one to exist:
//! `DiagnosticStatus.name` is conventionally `"<node>: <check>"` and
//! `hardware_id` is free text. So this is a heuristic, and the rule below was
//! chosen by measuring two real corpora rather than by assumption
//! (`scripts/diag_join_analysis.py`; results in the phase-62 roadmap).
//!
//! What the measurement decided:
//!
//! - **Name only.** `hardware_id` is 59-71% hostname-or-empty and rescued
//!   ZERO names the name itself could not match, on either corpus. It stays in
//!   the registry key for uniqueness and plays no part here.
//! - **Composables match exactly** — 161 of 173 attributions on the vehicle
//!   corpus. The container-versus-composable question W2 raised does not arise
//!   for them.
//! - **Absolute paths resolve on their last segment**
//!   (`/adapi/node/localization: state`), which is 2 more.
//! - **Relative `node/subcheck` names resolve on their FIRST segment**
//!   (`vehicle_interface/can_tx`, `vehicle_interface/brake`). The node owns
//!   the sub-check, so the node is the head of the prefix, not the tail.
//!   These only appear when the vehicle interface is connected to the VCU
//!   over CAN, which is why a bench corpus does not contain them.
//! - **The `-N` ordinal is play_launch's, not the node's** (issue 0018), so it
//!   is stripped before comparing.
//!
//! Measured attribution: 173/181 (96%) on the vehicle log, 143/151 (95%) on a
//! bench run. What remains unmatched is not a join problem — three names are
//! issue 0017 (a node the launch did not name, keyed by its executable while
//! the process registers its compiled-in name) and are recovered through the
//! interception identity map when it is available; the rest are sub-checks of
//! a hardware bridge that correspond to no member at all.
//!
//! Unmatched is a first-class outcome. A badge that guesses is worse than a
//! badge that says it does not know.

use std::collections::HashMap;

/// How a diagnostic was attributed, kept so the UI can be honest about it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Attribution {
    /// The diagnostic's prefix is a member name.
    Direct,
    /// Matched on the last segment of an ABSOLUTE ROS path prefix
    /// (`/adapi/node/localization`).
    PathLeaf,
    /// Matched on the FIRST segment of a relative `node/subcheck` prefix
    /// (`vehicle_interface/can_tx`), where the node owns the sub-check.
    SubCheck,
    /// Matched through the interception identity map: the diagnostic names the
    /// node's REAL ROS name, which differs from the model key because the
    /// launch file never named it (issue 0017).
    IdentityMap,
    /// No member corresponds. Shown as unmatched, never guessed at.
    None,
}

/// Resolves diagnostics to spawned members.
pub struct Attributor {
    /// Member name (ordinal stripped) -> the model key the UI addresses it by.
    members: HashMap<String, String>,
    /// Real ROS node name -> model key, from
    /// `interception/node_identity.tsv`. Empty when interception is off, which
    /// costs three attributions on the measured corpus and nothing else.
    identity: HashMap<String, String>,
}

/// Strip play_launch's collision ordinal (`<exec>-1`), which is ours and not
/// part of any name the node publishes.
fn strip_ordinal(name: &str) -> &str {
    match name.rsplit_once('-') {
        Some((head, tail)) if !tail.is_empty() && tail.bytes().all(|b| b.is_ascii_digit()) => head,
        _ => name,
    }
}

impl Attributor {
    pub fn new<I, S>(member_keys: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: AsRef<str>,
    {
        let mut members = HashMap::new();
        for key in member_keys {
            let key = key.as_ref();
            // Address by the full model key, match on the leaf: a diagnostic
            // says `ekf_localizer`, never `/localization/.../ekf_localizer`.
            let leaf = key.rsplit('/').next().unwrap_or(key);
            members.insert(strip_ordinal(leaf).to_string(), key.to_string());
        }
        Self {
            members,
            identity: HashMap::new(),
        }
    }

    /// Add the interception identity map: real ROS FQN -> model key.
    ///
    /// Recovers the issue-0017 cases, where the diagnostic uses the name the
    /// process actually registered and the model key was built from the
    /// executable.
    pub fn with_identity<I, K, V>(mut self, pairs: I) -> Self
    where
        I: IntoIterator<Item = (K, V)>,
        K: AsRef<str>,
        V: AsRef<str>,
    {
        for (fqn, key) in pairs {
            let fqn = fqn.as_ref();
            let leaf = fqn.rsplit('/').next().unwrap_or(fqn);
            self.identity
                .insert(leaf.to_string(), key.as_ref().to_string());
        }
        self
    }

    /// The member a diagnostic belongs to, and how that was decided.
    pub fn attribute(&self, diagnostic_name: &str) -> (Option<&str>, Attribution) {
        let prefix = diagnostic_name
            .split_once(':')
            .map(|(head, _)| head)
            .unwrap_or(diagnostic_name)
            .trim();
        if prefix.is_empty() {
            return (None, Attribution::None);
        }

        // Which end of a slash-separated prefix names the node depends on
        // whether it is an absolute ROS path or a relative sub-check:
        //
        //   /adapi/node/localization  ->  localization   (LAST segment)
        //   vehicle_interface/can_tx  ->  vehicle_interface (FIRST segment)
        //
        // Taking the last segment of the second shape yields `can_tx`, which
        // matches no member, and left five real diagnostics of a node that
        // does exist sitting in the unmatched bucket.
        let is_path = prefix.starts_with('/');
        let (candidate, how) = if is_path {
            (
                prefix.rsplit('/').next().unwrap_or(prefix),
                Attribution::PathLeaf,
            )
        } else if let Some((head, _)) = prefix.split_once('/') {
            (head, Attribution::SubCheck)
        } else {
            (prefix, Attribution::Direct)
        };
        let leaf = strip_ordinal(candidate);

        if let Some(key) = self.members.get(leaf) {
            return (Some(key.as_str()), how);
        }
        if let Some(key) = self.identity.get(leaf) {
            return (Some(key.as_str()), Attribution::IdentityMap);
        }
        (None, Attribution::None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn attributor() -> Attributor {
        Attributor::new([
            "/localization/pose_twist_fusion_filter/autoware_ekf_localizer_node-1",
            "/system/system_monitor/voltage_monitor",
            "/control/controller_node_exe",
            "/adapi/node/localization",
            "/vehicle/vehicle_interface",
        ])
    }

    /// The shapes actually present in the corpus, each with the count it
    /// accounted for there.
    #[test]
    fn attributes_the_shapes_the_corpus_contains() {
        let a = attributor();

        // 161 of 173: a composable, matched exactly on its own name.
        assert_eq!(
            a.attribute("voltage_monitor: CMOS Battery Status"),
            (
                Some("/system/system_monitor/voltage_monitor"),
                Attribution::Direct
            )
        );
        // 10 of 173: a plain node.
        assert_eq!(
            a.attribute("controller_node_exe: control_state"),
            (Some("/control/controller_node_exe"), Attribution::Direct)
        );
        // 2 of 173: a ROS path, resolved on its last segment.
        assert_eq!(
            a.attribute("/adapi/node/localization: state"),
            (Some("/adapi/node/localization"), Attribution::PathLeaf)
        );
    }

    /// Issue 0017: the diagnostic uses the name the process registered, the
    /// model key was built from the executable. Unmatched without the identity
    /// map, recovered with it — and recovered as IdentityMap, not Direct, so
    /// the UI can say how it knows.
    #[test]
    fn identity_map_recovers_the_issue_0017_names() {
        let plain = attributor();
        assert_eq!(
            plain.attribute("ekf_localizer: status"),
            (None, Attribution::None)
        );

        let with_id = attributor().with_identity([(
            "/localization/pose_twist_fusion_filter/ekf_localizer",
            "/localization/pose_twist_fusion_filter/autoware_ekf_localizer_node-1",
        )]);
        assert_eq!(
            with_id.attribute("ekf_localizer: status"),
            (
                Some("/localization/pose_twist_fusion_filter/autoware_ekf_localizer_node-1"),
                Attribution::IdentityMap
            )
        );
    }

    /// The hardware-bridge sub-checks. They must stay unmatched rather than
    /// being attached to something plausible — `vehicle_interface/can_tx` is
    /// not a member, and guessing that it belongs to `vehicle_interface` is a
    /// vehicle-side judgement this code must not make silently.
    #[test]
    fn vehicle_interface_subchecks_attribute_to_their_node() {
        // These are the vehicle interface's OWN sub-checks, not a separate
        // device: the node publishes one diagnostic per CAN frame family and
        // only runs at all when connected to the VCU over CAN. So the node is
        // the head of the prefix, and taking the tail (`can_tx`) matches
        // nothing and hides five real diagnostics of a member that exists.
        //
        // An earlier revision asserted the opposite, on the reading that these
        // were a hardware bridge with no corresponding member. They are the
        // member.
        let a = attributor();
        for n in [
            "vehicle_interface/can_tx",
            "vehicle_interface/frame_brk",
            "vehicle_interface/frame_eps",
            "vehicle_interface/frame_mtr",
            "vehicle_interface/brake",
            "vehicle_interface/motor",
            "vehicle_interface/system",
        ] {
            let (key, how) = a.attribute(n);
            assert_eq!(how, Attribution::SubCheck, "{n}");
            assert!(
                key.is_some_and(|k| k.ends_with("vehicle_interface")),
                "{n} -> {key:?}"
            );
        }
        // The bare name still resolves, by the plain rule.
        assert_eq!(a.attribute("vehicle_interface").1, Attribution::Direct);
    }

    #[test]
    fn an_absolute_path_still_resolves_on_its_last_segment() {
        // The two shapes pull in opposite directions, so this guards the
        // first-segment rule from swallowing ROS paths.
        let a = attributor();
        let (key, how) = a.attribute("/adapi/node/localization: state");
        assert_eq!(how, Attribution::PathLeaf);
        assert!(key.is_some_and(|k| k.ends_with("localization")), "{key:?}");
    }

    /// The ordinal is play_launch's own (issue 0018) and no node publishes it.
    #[test]
    fn the_model_ordinal_is_not_part_of_any_published_name() {
        assert_eq!(strip_ordinal("imu_corrector_node-1"), "imu_corrector_node");
        assert_eq!(strip_ordinal("imu_corrector_node-12"), "imu_corrector_node");
        // Not an ordinal: a name that merely ends in a hyphenated word.
        assert_eq!(strip_ordinal("front-left"), "front-left");
        assert_eq!(strip_ordinal("plain"), "plain");
    }

    #[test]
    fn a_name_with_no_prefix_is_unmatched_not_a_panic() {
        let a = attributor();
        assert_eq!(a.attribute(""), (None, Attribution::None));
        assert_eq!(a.attribute(": bare"), (None, Attribution::None));
    }
}

#[cfg(test)]
mod corpus_tests {
    use super::*;

    /// Cross-check against `scripts/diag_join_analysis.py`, name by name.
    ///
    /// The Rust join and the Python analysis that justified it must agree. An
    /// earlier version of this test asserted only that attribution stayed
    /// above 95%, which is a one-sided floor: it passes when the two
    /// implementations attribute entirely DIFFERENT sets, and it passed
    /// silently when the two did diverge. It also hardcoded a corpus path that
    /// is not on every machine, so it skipped rather than ran.
    ///
    /// This runs the Python over the same corpus and compares the unmatched
    /// SET. Skips cleanly when the sibling repo, a corpus, or python3 is
    /// absent, since none of them are this repo's to guarantee.
    #[test]
    fn matches_the_python_analysis_name_by_name() {
        let Some(home) = std::env::var_os("HOME") else {
            return;
        };
        let logs = std::path::PathBuf::from(&home).join("repos/2026-golf-cart/play_log");
        let script = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../scripts/diag_join_analysis.py");
        if !logs.is_dir() || !script.is_file() {
            eprintln!("SKIP: corpus or analysis script absent");
            return;
        }
        // Any run that has both a diagnostics CSV and spawned members will do;
        // pinning one path is how the old test came to skip everywhere.
        let Some(run) = std::fs::read_dir(&logs).ok().and_then(|rd| {
            let mut runs: Vec<_> = rd
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.join("diagnostics.csv").is_file() && p.join("node").is_dir())
                .collect();
            runs.sort();
            runs.pop()
        }) else {
            eprintln!("SKIP: no run with diagnostics.csv and members");
            return;
        };

        let out = match std::process::Command::new("python3")
            .arg(&script)
            .arg("--json")
            .arg(&run)
            .output()
        {
            Ok(o) if o.status.success() => o.stdout,
            _ => {
                eprintln!("SKIP: python3 or the analysis script would not run");
                return;
            }
        };
        let text = String::from_utf8_lossy(&out);
        let Some(json) = text.split("---JSON---").nth(1) else {
            eprintln!("SKIP: analysis produced no JSON block");
            return;
        };
        let py: serde_json::Value = serde_json::from_str(json.trim()).expect("analysis JSON");
        let py_unmatched: std::collections::BTreeSet<String> = py["unmatched"]
            .as_array()
            .expect("unmatched array")
            .iter()
            .map(|v| v.as_str().unwrap_or_default().to_string())
            .collect();

        let mut keys = Vec::new();
        for sub in ["node", "load_node"] {
            if let Ok(rd) = std::fs::read_dir(run.join(sub)) {
                for e in rd.flatten() {
                    if e.path().is_dir()
                        && let Some(n) = e.file_name().to_str()
                    {
                        keys.push(n.to_string());
                    }
                }
            }
        }
        let a = Attributor::new(&keys);

        let csv = std::fs::read_to_string(run.join("diagnostics.csv")).expect("read corpus");
        let mut names = std::collections::BTreeSet::new();
        for line in csv.lines().skip(1) {
            let mut it = line.splitn(4, ',');
            let (_ts, _hw, name) = (it.next(), it.next(), it.next());
            if let Some(n) = name
                && !n.is_empty()
            {
                names.insert(n.to_string());
            }
        }
        let rust_unmatched: std::collections::BTreeSet<String> = names
            .iter()
            .filter(|n| a.attribute(n).1 == Attribution::None)
            .cloned()
            .collect();

        let only_rust: Vec<_> = rust_unmatched.difference(&py_unmatched).collect();
        let only_py: Vec<_> = py_unmatched.difference(&rust_unmatched).collect();
        assert!(
            only_rust.is_empty() && only_py.is_empty(),
            "the Rust join and scripts/diag_join_analysis.py disagree on {}\n  \
             unmatched only in Rust: {only_rust:?}\n  \
             unmatched only in Python: {only_py:?}",
            run.display()
        );
        eprintln!(
            "corpus {}: {}/{} attributed, both implementations agree",
            run.file_name().unwrap_or_default().to_string_lossy(),
            names.len() - rust_unmatched.len(),
            names.len()
        );
    }
}
