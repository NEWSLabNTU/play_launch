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
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PosixOverride {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub priority: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub core: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sched_class: Option<String>,
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

/// A parsed platform file — either authored directly as v2 YAML, or
/// produced by the legacy TOML bridge ([`crate::bridge::parse_legacy_toml`]).
#[derive(Clone, Debug, PartialEq)]
pub struct PlatformFile {
    pub target: String,
    pub mapper: String,
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
    resources: serde_yaml_ng::Value,
    #[serde(default)]
    overrides: serde_yaml_ng::Value,
}

fn is_posix(target: &str) -> bool {
    matches!(target, "posix" | "native")
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
