//! v2 platform-file schema (YAML): `target` + `mapper` + `resources` +
//! `overrides`. One file names one target; `resources`/`overrides` value
//! vocabulary is per-target. `posix` (Linux RT) is typed concretely here;
//! any other target (`zephyr`, `freertos`, ...) is kept as a raw
//! [`serde_yaml_ng::Value`] passthrough — nano-ros validates its own
//! target vocabularies, this crate only guarantees the file parses and the
//! `posix` shape is well-formed.
//!
//! Legacy `system.toml` documents (see [`crate::types::SystemSched`]) parse
//! through a separate path ([`crate::bridge`]) into the same [`PlatformFile`]
//! shape, with `mapper: "manual"` and the legacy spec carried in
//! [`PlatformFile::legacy`].

use std::{collections::BTreeMap, path::Path};

use serde::{Deserialize, Serialize};

use crate::types::SystemSched;

/// Errors parsing a platform file (either schema).
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PlatformError {
    #[error("failed to read platform file {path}: {reason}")]
    Io { path: String, reason: String },
    #[error("failed to parse platform file as YAML: {0}")]
    Yaml(String),
    #[error(
        "failed to parse platform file's `resources`/`overrides` for target `{target}`: {reason}"
    )]
    TargetShape { target: String, reason: String },
    #[error("platform file `target` must be non-empty")]
    EmptyTarget,
    #[error("platform file `mapper` must be non-empty")]
    EmptyMapper,
    #[error(
        "unsupported platform file extension: {0:?} (expected `.yaml`/`.yml` or legacy `.toml`)"
    )]
    UnknownExtension(Option<String>),
    #[error("legacy TOML bridge failed to parse: {0}")]
    Legacy(String),
    #[error(
        "`resources.rt_priority_band` {{ min: {min}, max: {max} }} is invalid for target \
         `posix`: {reason}"
    )]
    InvalidPriorityBand { min: i64, max: i64, reason: String },
    #[error("overrides.{node}: {reason}")]
    InvalidOverride { node: String, reason: String },
    #[error("`reservations` must be `off` or `required`, got {value:?}")]
    UnknownReservationMode { value: String },
}

/// `posix` (Linux RT) platform facts — `resources:` for `target: posix`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PosixResources {
    /// The RT priority band the mapper must spread derived priorities
    /// within (and the band `--sched-apply` clamps/violations are checked
    /// against).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rt_priority_band: Option<PriorityBand>,
    /// CPU cores reserved for isolated (RT) workloads (advisory; consumed by
    /// play_launch/nano-ros placement policy, not enforced by this crate).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub isolated_cpus: Vec<u32>,
    /// The host's `SCHED_RR` time slice, in microseconds.
    ///
    /// A platform *fact*, which is why it lives here: on Linux the slice is a
    /// global sysctl (`/proc/sys/kernel/sched_rr_timeslice_ms`, default
    /// **100 ms**), not a per-task value, so `TierPlatformSpec::time_slice_us`
    /// cannot express it. The mapper needs it to decide whether `SCHED_RR` is
    /// worth deriving at all: at 100 ms, two tied 10 Hz nodes get a slice as
    /// long as their entire period, so RR degenerates to FIFO while looking
    /// like it solved starvation.
    ///
    /// Absent means "unknown" — the mapper then declines to derive RR rather
    /// than assuming the default, since assuming is how a cosmetic change gets
    /// presented as a real one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rr_timeslice_us: Option<u64>,
}

/// An inclusive RT priority band, e.g. `{ min: 10, max: 40 }`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PriorityBand {
    pub min: i64,
    pub max: i64,
}

/// The legal POSIX RT priority range for `SCHED_FIFO`/`SCHED_RR` on Linux
/// (`sched_get_priority_min(2)`/`max` are 1 and 99 for both policies).
pub const POSIX_RT_PRIORITY_MIN: i64 = 1;
/// See [`POSIX_RT_PRIORITY_MIN`].
pub const POSIX_RT_PRIORITY_MAX: i64 = 99;

impl PriorityBand {
    /// `true` when `min <= max` (a non-degenerate, orderable band).
    pub fn is_valid(&self) -> bool {
        self.min <= self.max
    }

    /// `true` when `priority` falls within `[min, max]`.
    pub fn contains(&self, priority: i64) -> bool {
        priority >= self.min && priority <= self.max
    }

    /// Validate this band for the `posix` target: ordered (`min <= max`) and
    /// entirely inside Linux's legal `SCHED_FIFO`/`SCHED_RR` priority range
    /// (1..=99). Returns a human-readable reason on failure. Only meaningful
    /// for `posix` — other targets have their own priority vocabularies and
    /// are not validated by this crate (raw passthrough).
    pub fn validate_posix(&self) -> Result<(), String> {
        if self.min > self.max {
            return Err(format!("min ({}) > max ({})", self.min, self.max));
        }
        if self.min < POSIX_RT_PRIORITY_MIN || self.max > POSIX_RT_PRIORITY_MAX {
            return Err(format!(
                "band must lie within the POSIX SCHED_FIFO/SCHED_RR real-time priority range \
                 {POSIX_RT_PRIORITY_MIN}..={POSIX_RT_PRIORITY_MAX}"
            ));
        }
        Ok(())
    }
}

/// `posix` override entry — `overrides.<node>` for `target: posix`.
///
/// An override always beats a derived value. Validation is at parse time
/// ([`validate_posix_override`]) so a malformed override is a schema error
/// naming the node, not an `EINVAL` from a syscall after spawn.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PosixOverride {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub priority: Option<i64>,
    /// Single-CPU pin. Kept as a one-element alias for [`Self::cpus`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub core: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sched_class: Option<String>,
    /// CPU mask. Mutually exclusive with `core`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub cpus: Vec<u32>,
    /// CFS nice value, `-20..=19`. Only meaningful for `SCHED_OTHER` and
    /// `SCHED_BATCH`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nice: Option<i32>,
    /// Utilization-clamp floor, `0..=1024`.
    ///
    /// A **no-op on RT policies**: they default to `1024`/`1024` and already
    /// request the maximum performance point under `schedutil`. Setting it on
    /// `SCHED_FIFO`/`RR` is accepted and warned about, because silently doing
    /// nothing is worse than saying so. The system-wide equivalent is
    /// `sched_util_clamp_min_rt_default`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub uclamp_min: Option<u32>,
    /// Utilization-clamp ceiling, `0..=1024`. The useful RT knob: it lets an
    /// RT thread run *below* the maximum performance point.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub uclamp_max: Option<u32>,
    /// Declared execution cost, in microseconds.
    ///
    /// The field `TierDef::budget_us` has always meant "execution-time budget
    /// — EDF/sporadic", but only the deprecated v1 TOML bridge could populate
    /// it, so on every v2 path it was `None` and the derivation substituted a
    /// path's *deadline* for its cost. This is what makes cost authorable, and
    /// it is the only legitimate source for a `SCHED_DEADLINE` reservation's
    /// runtime.
    ///
    /// Not a proven WCET — there is no static analysis here. It is a declared
    /// high-percentile observed cost, used *as* an upper bound.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub budget_us: Option<u64>,
}

/// The parsed `resources:` block, typed for known targets and raw otherwise.
#[derive(Clone, Debug, PartialEq)]
pub enum PlatformResources {
    Posix(PosixResources),
    /// Unknown target: kept as the raw parsed YAML value, untouched.
    Raw(serde_yaml_ng::Value),
}

impl Default for PlatformResources {
    fn default() -> Self {
        PlatformResources::Posix(PosixResources::default())
    }
}

/// One parsed `overrides.<node>` entry, typed for known targets and raw
/// otherwise (mirrors [`PlatformResources`]).
#[derive(Clone, Debug, PartialEq)]
pub enum PlatformOverrideEntry {
    Posix(PosixOverride),
    Raw(serde_yaml_ng::Value),
}

/// Whether the mapper may derive `SCHED_DEADLINE` reservations.
///
/// Opt-in, and deliberately not a member of `resources:`. `resources:` holds
/// platform *facts* — what the machine has; `mapper:` holds *policy* — what to
/// do with them. Whether to reserve is a policy choice, so it sits beside
/// `mapper:` rather than among the facts.
///
/// The opt-in exists because reservations are all-or-nothing within a band: a
/// node with a reservation preempts every fixed-priority thread regardless of
/// priority, so a band containing both loses the ordering the mapper computed.
/// Without an explicit switch, adding a single `budget_us` would turn that
/// rule into a hard error the author did not ask for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReservationMode {
    /// Budgets are parsed, carried and shown, but never select a policy.
    #[default]
    Off,
    /// Every node in the RT band that carries a timing fact must also carry a
    /// budget, or resolution fails naming the ones that do not.
    Required,
}

/// A parsed platform file — either authored directly as v2 YAML, or
/// produced by the legacy TOML bridge ([`crate::bridge::parse_legacy_toml`]).
#[derive(Clone, Debug, PartialEq)]
pub struct PlatformFile {
    pub target: String,
    pub mapper: String,
    /// Whether reservations may be derived. See [`ReservationMode`].
    pub reservations: ReservationMode,
    pub resources: PlatformResources,
    /// Explicit per-node pins, keyed by node selector (same selector
    /// vocabulary as `[[assign]]`'s `nodes`: full FQN or bare name).
    pub overrides: BTreeMap<String, PlatformOverrideEntry>,
    /// Present only when this file originated from the legacy `.toml`
    /// bridge: the `manual` mapper consumes this to reproduce today's
    /// `resolve()` output exactly. `None` for files authored directly in
    /// the v2 YAML schema.
    pub legacy: Option<SystemSched>,
}

/// Raw top-level shape, deserialized before per-target validation of
/// `resources`/`overrides` (whose vocabulary depends on the sibling
/// `target` field, which plain derived `Deserialize` can't express).
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawPlatformFile {
    target: String,
    mapper: String,
    #[serde(default)]
    reservations: ReservationMode,
    #[serde(default)]
    resources: serde_yaml_ng::Value,
    #[serde(default)]
    overrides: serde_yaml_ng::Value,
}

fn is_posix(target: &str) -> bool {
    matches!(target, "posix" | "native")
}

/// Reject override combinations Linux cannot execute, at parse time.
///
/// Every rule here was previously unenforced, so a malformed override reached
/// the syscall layer and came back as an `EINVAL` with no node name attached.
/// Failing at parse means the message can name the file, the node and the
/// field.
pub fn validate_posix_override(node: &str, ov: &PosixOverride) -> Result<(), PlatformError> {
    use crate::posix::{PosixPolicyKind, UCLAMP_MAX};

    let bad = |reason: String| PlatformError::InvalidOverride {
        node: node.to_string(),
        reason,
    };

    // Fails closed: an unrecognized class used to become SCHED_OTHER in
    // silence, so `SCHED_FIF0` dropped a node out of real-time.
    let kind = match ov.sched_class.as_deref() {
        Some(sc) => Some(PosixPolicyKind::parse(sc).map_err(|e| bad(e.to_string()))?),
        None => None,
    };

    if let Some(kind) = kind {
        // A priority on a CFS policy, or a nice value on an RT one, is not a
        // harmless extra field — it is a statement about scheduling that the
        // kernel cannot honour, and it is how `SCHED_OTHER 10` reached
        // system_model.yaml once already.
        if ov.priority.is_some() && !matches!(kind, PosixPolicyKind::Fifo | PosixPolicyKind::Rr) {
            return Err(bad(format!(
                "`priority` is only meaningful for SCHED_FIFO/SCHED_RR, not {}",
                kind.as_str()
            )));
        }
        if ov.nice.is_some() && !matches!(kind, PosixPolicyKind::Other | PosixPolicyKind::Batch) {
            return Err(bad(format!(
                "`nice` is only meaningful for SCHED_OTHER/SCHED_BATCH, not {}",
                kind.as_str()
            )));
        }
        if matches!(kind, PosixPolicyKind::Deadline) && (ov.core.is_some() || !ov.cpus.is_empty()) {
            return Err(bad(
                "SCHED_DEADLINE cannot take a CPU pin: a deadline thread's affinity may not \
                 be narrower than the root domain it was created on (sched_setattr returns \
                 EPERM). Use an exclusive cpuset partition."
                    .to_string(),
            ));
        }
    }

    if let Some(p) = ov.priority
        && !(POSIX_RT_PRIORITY_MIN..=POSIX_RT_PRIORITY_MAX).contains(&p)
    {
        return Err(bad(format!(
            "`priority` {p} is outside the POSIX real-time range \
             {POSIX_RT_PRIORITY_MIN}..={POSIX_RT_PRIORITY_MAX}"
        )));
    }

    if let Some(n) = ov.nice
        && !(-20..=19).contains(&n)
    {
        return Err(bad(format!("`nice` {n} is outside -20..=19")));
    }

    if ov.core.is_some() && !ov.cpus.is_empty() {
        return Err(bad(
            "`core` and `cpus` are mutually exclusive (`core` is the single-CPU alias)".to_string(),
        ));
    }

    for (field, v) in [("uclamp_min", ov.uclamp_min), ("uclamp_max", ov.uclamp_max)] {
        if let Some(v) = v
            && v > UCLAMP_MAX
        {
            return Err(bad(format!(
                "`{field}` {v} is outside 0..={UCLAMP_MAX} (the kernel's scale is not a percentage)"
            )));
        }
    }
    if let (Some(min), Some(max)) = (ov.uclamp_min, ov.uclamp_max)
        && min > max
    {
        return Err(bad(format!(
            "`uclamp_min` {min} exceeds `uclamp_max` {max}"
        )));
    }

    if ov.budget_us == Some(0) {
        return Err(bad(
            "`budget_us` 0 is not a cost — omit the field to say the cost is unknown. \
             Absent and zero are different answers."
                .to_string(),
        ));
    }

    Ok(())
}

/// Parse a v2 platform file from a YAML string.
pub fn parse_platform_file_yaml(input: &str) -> Result<PlatformFile, PlatformError> {
    let raw: RawPlatformFile =
        serde_yaml_ng::from_str(input).map_err(|e| PlatformError::Yaml(e.to_string()))?;

    if raw.target.trim().is_empty() {
        return Err(PlatformError::EmptyTarget);
    }
    if raw.mapper.trim().is_empty() {
        return Err(PlatformError::EmptyMapper);
    }

    let resources = if is_posix(&raw.target) {
        let posix: PosixResources =
            serde_yaml_ng::from_value(raw.resources).map_err(|e| PlatformError::TargetShape {
                target: raw.target.clone(),
                reason: e.to_string(),
            })?;
        if let Some(band) = &posix.rt_priority_band {
            band.validate_posix()
                .map_err(|reason| PlatformError::InvalidPriorityBand {
                    min: band.min,
                    max: band.max,
                    reason,
                })?;
        }
        PlatformResources::Posix(posix)
    } else {
        PlatformResources::Raw(raw.resources)
    };

    let mut overrides = BTreeMap::new();
    if let serde_yaml_ng::Value::Mapping(mapping) = raw.overrides {
        for (key, value) in mapping {
            let key = key.as_str().ok_or_else(|| PlatformError::TargetShape {
                target: raw.target.clone(),
                reason: "`overrides` keys must be strings".to_string(),
            })?;
            let entry = if is_posix(&raw.target) {
                let posix: PosixOverride =
                    serde_yaml_ng::from_value(value).map_err(|e| PlatformError::TargetShape {
                        target: raw.target.clone(),
                        reason: format!("overrides.{key}: {e}"),
                    })?;
                validate_posix_override(key, &posix)?;
                PlatformOverrideEntry::Posix(posix)
            } else {
                PlatformOverrideEntry::Raw(value)
            };
            overrides.insert(key.to_string(), entry);
        }
    } else if !matches!(raw.overrides, serde_yaml_ng::Value::Null) {
        return Err(PlatformError::TargetShape {
            target: raw.target.clone(),
            reason: "`overrides` must be a mapping".to_string(),
        });
    }

    Ok(PlatformFile {
        target: raw.target,
        mapper: raw.mapper,
        reservations: raw.reservations,
        resources,
        overrides,
        legacy: None,
    })
}

/// Parse a platform file from disk, dispatching on extension: `.yaml`/`.yml`
/// selects the v2 schema, `.toml` selects the legacy bridge
/// ([`crate::bridge::parse_legacy_toml`]).
pub fn parse_platform_file(path: &Path) -> Result<PlatformFile, PlatformError> {
    let text = std::fs::read_to_string(path).map_err(|e| PlatformError::Io {
        path: path.display().to_string(),
        reason: e.to_string(),
    })?;

    match path.extension().and_then(|e| e.to_str()) {
        Some("yaml") | Some("yml") => parse_platform_file_yaml(&text),
        Some("toml") => crate::bridge::parse_legacy_toml(&text)
            .map_err(|e| PlatformError::Legacy(e.to_string())),
        other => Err(PlatformError::UnknownExtension(other.map(str::to_string))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_valid_posix_platform_file() {
        let src = r#"
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 40 }
  isolated_cpus: [0]
overrides:
  control_node: { priority: 20, core: 0 }
"#;
        let file = parse_platform_file_yaml(src).expect("must parse");
        assert_eq!(file.target, "posix");
        assert_eq!(file.mapper, "rate_monotonic");
        let PlatformResources::Posix(res) = &file.resources else {
            panic!("expected posix resources");
        };
        assert_eq!(
            res.rt_priority_band,
            Some(PriorityBand { min: 10, max: 40 })
        );
        assert_eq!(res.isolated_cpus, vec![0]);
        let PlatformOverrideEntry::Posix(ov) = file.overrides.get("control_node").unwrap() else {
            panic!("expected posix override");
        };
        assert_eq!(ov.priority, Some(20));
        assert_eq!(ov.core, Some(0));
        assert!(file.legacy.is_none());
    }

    #[test]
    fn override_vocabulary_parses() {
        let src = r#"
target: posix
mapper: chain_aware
reservations: required
resources:
  rt_priority_band: { min: 10, max: 40 }
  isolated_cpus: [2, 3]
  rr_timeslice_us: 100000
overrides:
  obstacle_detector:
    budget_us: 8000
    uclamp_max: 800
  telemetry_logger:
    sched_class: SCHED_BATCH
    nice: 10
    uclamp_min: 256
  planner:
    cpus: [4, 5]
"#;
        let file = parse_platform_file_yaml(src).expect("must parse");
        assert_eq!(file.reservations, ReservationMode::Required);
        let PlatformResources::Posix(res) = &file.resources else {
            panic!("expected posix resources");
        };
        assert_eq!(res.rr_timeslice_us, Some(100_000));

        let PlatformOverrideEntry::Posix(det) = file.overrides.get("obstacle_detector").unwrap()
        else {
            panic!("posix override");
        };
        assert_eq!(det.budget_us, Some(8000));
        assert_eq!(det.uclamp_max, Some(800));

        let PlatformOverrideEntry::Posix(log) = file.overrides.get("telemetry_logger").unwrap()
        else {
            panic!("posix override");
        };
        assert_eq!(log.sched_class.as_deref(), Some("SCHED_BATCH"));
        assert_eq!(log.nice, Some(10));

        let PlatformOverrideEntry::Posix(pl) = file.overrides.get("planner").unwrap() else {
            panic!("posix override");
        };
        assert_eq!(pl.cpus, vec![4, 5]);
    }

    #[test]
    fn reservations_defaults_to_off() {
        let file = parse_platform_file_yaml("target: posix\nmapper: chain_aware\n").unwrap();
        assert_eq!(file.reservations, ReservationMode::Off);
    }

    /// Parse a one-override file and return the error text, so each rejection
    /// below reads as the YAML that causes it.
    fn override_err(body: &str) -> String {
        let src = format!("target: posix\nmapper: chain_aware\noverrides:\n  n:\n{body}");
        parse_platform_file_yaml(&src)
            .expect_err("expected this override to be rejected")
            .to_string()
    }

    #[test]
    fn unknown_sched_class_is_an_error_not_a_silent_sched_other() {
        // The whole point of failing closed: this used to become SCHED_OTHER
        // and drop the node out of real-time with no diagnostic at all.
        let e = override_err("    sched_class: SCHED_FIF0\n");
        assert!(e.contains("SCHED_FIF0"), "{e}");
        assert!(
            e.contains("SCHED_FIFO"),
            "should list the legal values: {e}"
        );
    }

    #[test]
    fn priority_and_nice_must_match_the_policy() {
        let e = override_err("    sched_class: SCHED_OTHER\n    priority: 20\n");
        assert!(e.contains("priority"), "{e}");
        assert!(e.contains("SCHED_OTHER"), "{e}");

        let e = override_err("    sched_class: SCHED_FIFO\n    nice: 5\n");
        assert!(e.contains("nice"), "{e}");
        assert!(e.contains("SCHED_FIFO"), "{e}");
    }

    #[test]
    fn deadline_override_cannot_pin_cpus() {
        let e = override_err("    sched_class: SCHED_DEADLINE\n    core: 2\n");
        assert!(e.contains("EPERM") || e.contains("cpuset"), "{e}");
    }

    #[test]
    fn out_of_range_values_are_rejected() {
        assert!(override_err("    sched_class: SCHED_FIFO\n    priority: 0\n").contains("1..=99"));
        assert!(override_err("    sched_class: SCHED_BATCH\n    nice: 20\n").contains("-20..=19"));
        assert!(override_err("    uclamp_min: 2000\n").contains("0..=1024"));
        assert!(override_err("    uclamp_min: 900\n    uclamp_max: 100\n").contains("exceeds"));
    }

    #[test]
    fn core_and_cpus_are_mutually_exclusive() {
        let e = override_err("    core: 1\n    cpus: [2, 3]\n");
        assert!(e.contains("mutually exclusive"), "{e}");
    }

    #[test]
    fn zero_budget_is_rejected_because_absent_and_zero_differ() {
        let e = override_err("    budget_us: 0\n");
        assert!(e.contains("Absent and zero"), "{e}");
    }

    #[test]
    fn empty_target_is_rejected() {
        let src = "target: \"\"\nmapper: manual\n";
        let err = parse_platform_file_yaml(src).unwrap_err();
        assert_eq!(err, PlatformError::EmptyTarget);
    }

    #[test]
    fn empty_mapper_is_rejected() {
        let src = "target: posix\nmapper: \"\"\n";
        let err = parse_platform_file_yaml(src).unwrap_err();
        assert_eq!(err, PlatformError::EmptyMapper);
    }

    #[test]
    fn posix_band_outside_rt_range_is_rejected() {
        // max beyond 99
        let src = r#"
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 10, max: 200 }
"#;
        let err = parse_platform_file_yaml(src).unwrap_err();
        let PlatformError::InvalidPriorityBand { min, max, reason } = &err else {
            panic!("expected InvalidPriorityBand, got: {err:?}");
        };
        assert_eq!((*min, *max), (10, 200));
        assert!(
            reason.contains("1..=99"),
            "reason should cite the range: {reason}"
        );

        // min below 1 (0 is not a valid RT priority)
        let src = r#"
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 0, max: 40 }
"#;
        let err = parse_platform_file_yaml(src).unwrap_err();
        assert!(matches!(err, PlatformError::InvalidPriorityBand { .. }));
    }

    #[test]
    fn posix_band_min_above_max_is_rejected_at_parse() {
        let src = r#"
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 40, max: 10 }
"#;
        let err = parse_platform_file_yaml(src).unwrap_err();
        let PlatformError::InvalidPriorityBand { reason, .. } = &err else {
            panic!("expected InvalidPriorityBand, got: {err:?}");
        };
        assert!(
            reason.contains("min") && reason.contains("max"),
            "got: {reason}"
        );
    }

    #[test]
    fn posix_band_at_full_rt_range_is_accepted() {
        let src = r#"
target: posix
mapper: rate_monotonic
resources:
  rt_priority_band: { min: 1, max: 99 }
"#;
        let file = parse_platform_file_yaml(src).expect("full 1..=99 band is legal");
        let PlatformResources::Posix(res) = &file.resources else {
            panic!("expected posix resources");
        };
        assert_eq!(res.rt_priority_band, Some(PriorityBand { min: 1, max: 99 }));
    }

    #[test]
    fn unknown_target_band_is_not_range_validated() {
        // Zephyr priorities can be negative (coop) and use a different
        // range entirely — an unknown target's "band" must pass through raw,
        // unvalidated (the consumer validates its own vocabulary).
        let src = r#"
target: zephyr
mapper: rate_monotonic
resources:
  rt_priority_band: { min: -16, max: 200 }
"#;
        let file = parse_platform_file_yaml(src).expect("unknown target is never range-validated");
        assert!(matches!(file.resources, PlatformResources::Raw(_)));
    }

    #[test]
    fn unknown_target_keeps_raw_passthrough() {
        let src = r#"
target: zephyr
mapper: manual
resources:
  stack_bytes: 8192
  custom_key: [1, 2, 3]
overrides:
  control_node: { anything: "goes" }
"#;
        let file = parse_platform_file_yaml(src).expect("must parse");
        assert_eq!(file.target, "zephyr");
        assert!(matches!(file.resources, PlatformResources::Raw(_)));
        assert!(matches!(
            file.overrides.get("control_node"),
            Some(PlatformOverrideEntry::Raw(_))
        ));
    }

    #[test]
    fn minimal_posix_file_with_no_resources_or_overrides_parses() {
        let src = "target: posix\nmapper: manual\n";
        let file = parse_platform_file_yaml(src).expect("must parse");
        let PlatformResources::Posix(res) = &file.resources else {
            panic!("expected posix resources");
        };
        assert_eq!(res.rt_priority_band, None);
        assert!(file.overrides.is_empty());
    }

    #[test]
    fn unknown_field_in_posix_resources_is_rejected() {
        let src = r#"
target: posix
mapper: manual
resources:
  bogus_field: 1
"#;
        let err = parse_platform_file_yaml(src).unwrap_err();
        assert!(matches!(err, PlatformError::TargetShape { .. }));
    }

    #[test]
    fn dispatches_by_extension() {
        let dir = std::env::temp_dir().join(format!(
            "sched_platform_test_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();

        let yaml_path = dir.join("bringup.system.posix.yaml");
        std::fs::write(&yaml_path, "target: posix\nmapper: manual\n").unwrap();
        let file = parse_platform_file(&yaml_path).expect("yaml parses");
        assert_eq!(file.mapper, "manual");
        assert!(file.legacy.is_none());

        let toml_path = dir.join("system.toml");
        std::fs::write(&toml_path, "").unwrap();
        let file = parse_platform_file(&toml_path).expect("toml parses via bridge");
        assert_eq!(file.mapper, "manual");
        assert_eq!(file.target, "posix");
        assert!(file.legacy.is_some());

        let bogus_path = dir.join("bringup.json");
        std::fs::write(&bogus_path, "{}").unwrap();
        let err = parse_platform_file(&bogus_path).unwrap_err();
        assert!(matches!(err, PlatformError::UnknownExtension(_)));

        std::fs::remove_dir_all(&dir).ok();
    }
}
