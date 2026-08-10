//! SystemModel — the resolved, checked system artifact.
//!
//! One `SystemModel` describes ONE concrete system variant (early binding:
//! args bound, conditions evaluated, every name fully qualified), produced by
//! `play_launch resolve` from the launch tree + contract manifests + the
//! integrator's system config, and consumed by
//!
//! - the play_launch Linux runtime (spawn/supervise + contract monitors), and
//! - the nano-ros build system (bake each `mcu:*` node's slice into its image).
//!
//! Design: play_launch `docs/design/system-model.md` (producer side) and
//! nano-ros RFC-0050 (consumer side). This crate is PURE SCHEMA — types,
//! serde, and YAML round-trip; no resolver logic. The resolver refuses to
//! emit a model when the manifest checker reports Error severity, so a model
//! in hand is always a checked one; warnings ride along in
//! [`Meta::diagnostics`].
//!
//! All collections are `BTreeMap`/sorted so serialization is deterministic —
//! the YAML form is hashed for provenance and caching.

use indexmap::IndexMap;
use std::collections::BTreeMap;

pub mod system_config;

use serde::{Deserialize, Serialize};

/// Schema version this crate reads and writes.
///
/// Versioned independently of the manifest format version: the manifest is
/// the authoring surface, the model is a derived artifact.
pub const SCHEMA_VERSION: u32 = 1;

/// Errors loading or saving a model.
#[derive(Debug, thiserror::Error)]
pub enum ModelError {
    #[error("YAML: {0}")]
    Yaml(#[from] serde_yaml_ng::Error),
    #[error("unsupported SystemModel schema version {found} (this reader supports <= {supported})")]
    UnsupportedVersion { found: u32, supported: u32 },
}

/// The resolved system artifact. See the crate docs for the contract.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SystemModel {
    pub meta: Meta,
    /// Layer 1 — from the launch tree: who exists and how it is wired.
    pub structure: Structure,
    /// Layer 2 — from the manifests, post-merge: what timing/QoS is promised.
    #[serde(default, skip_serializing_if = "Contracts::is_empty")]
    pub contracts: Contracts,
    /// Layer 3 — from the integrator's system config: where and how it runs.
    #[serde(default, skip_serializing_if = "Execution::is_empty")]
    pub execution: Execution,
}

impl SystemModel {
    /// Parse from YAML, rejecting schema versions newer than this reader.
    pub fn from_yaml_str(s: &str) -> Result<Self, ModelError> {
        let model: SystemModel = serde_yaml_ng::from_str(s)?;
        if model.meta.version > SCHEMA_VERSION {
            return Err(ModelError::UnsupportedVersion {
                found: model.meta.version,
                supported: SCHEMA_VERSION,
            });
        }
        Ok(model)
    }

    /// Canonical YAML form (deterministic: all maps are ordered).
    pub fn to_yaml_string(&self) -> Result<String, ModelError> {
        Ok(serde_yaml_ng::to_string(self)?)
    }
}

// ---------------------------------------------------------------------------
// meta
// ---------------------------------------------------------------------------

/// Provenance + diagnostics. Everything needed to answer "what exactly was
/// resolved, from what inputs, by what tool" — and the cache key.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Meta {
    /// SystemModel schema version (see [`SCHEMA_VERSION`]).
    pub version: u32,
    /// The exact argument binding this model was resolved from.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub args: BTreeMap<String, String>,
    /// Content hashes of every input file (manifests, launch files, system
    /// config). Sorted by path; the resolver's cache key.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub inputs: Vec<InputHash>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub resolver: Option<ResolverInfo>,
    /// Checker warnings embedded at resolve time (Errors refuse emission).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub diagnostics: Vec<String>,
    // nano-ros issue 0320 — the `record` field (a bound `record.json`
    // companion) is retired: the producer only ever wrote `None`, the files it
    // named were never committed, and its sole reader was a round-trip test.
    // Old models still carry `record:` on disk; without `deny_unknown_fields`
    // it deserializes and is dropped, so removal is backward-compatible on read.
}

/// Render an input path for [`InputHash::path`], relative to `base` when the
/// path lies under it.
///
/// These were `canonicalize`d absolute paths, baked into a COMMITTED artifact.
/// That is not merely untidy — consumers read them back (nano-ros's
/// `nros::main!` looks for the `.toml` input to find the system.toml the model
/// was resolved against), and an absolute path from another machine fails its
/// existence check and silently falls back to a fixed filename, reintroducing
/// the per-target leak that recording the input was meant to prevent
/// (nano-ros issue 0293).
///
/// Relative to the bringup package root the model is portable, and the
/// consumer resolves it against its own root. An input OUTSIDE that root (an
/// include from a sibling package) keeps its absolute path: no relative form
/// is meaningful across packages, and a `../..` chain would be worse than
/// useless in an install space.
pub fn input_path_string(path: &std::path::Path, base: Option<&std::path::Path>) -> String {
    let canon = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
    if let Some(base) = base {
        let base = std::fs::canonicalize(base).unwrap_or_else(|_| base.to_path_buf());
        if let Ok(rel) = canon.strip_prefix(&base) {
            return rel.display().to_string();
        }
    }
    canon.display().to_string()
}

/// One input file's content hash.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct InputHash {
    pub path: String,
    pub sha256: String,
}

/// The tool that produced this model.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ResolverInfo {
    pub tool: String,
    pub version: String,
}

// ---------------------------------------------------------------------------
// layer 1 — structure
// ---------------------------------------------------------------------------

/// The resolved graph: node instances, entities, and the scope tree (kept
/// for diagnostics and budget attribution). All names fully qualified.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Structure {
    /// Scope id (e.g. `autoware_launch/planning_simulator.launch.xml`) → info.
    /// The scope id is STRUCTURAL — the launch file (`<pkg>/<file>`) the scope
    /// corresponds to, disambiguated with `#<id>` when a file is included more
    /// than once. It is NOT a namespace (Phase 48: namespace is a per-node
    /// property, not a scope property). `<group>` scopes are folded into their
    /// launch file, so this map is the include tree.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub scopes: BTreeMap<String, ScopeInfo>,
    /// Node FQN (`/ns/node_name`) → instance.
    ///
    /// Issue 0382 — an ORDER-PRESERVING map, not a `BTreeMap`. Construct order
    /// is semantic: components initialize (and `configure()`) in the order the
    /// emitter walks this mapping, and the launch file is where the user
    /// expresses it. A `BTreeMap` alphabetized that away at resolve time, so a
    /// launch declaring `talker` before `listener` produced an entry that built
    /// `listener` first. The resolver already inserts in launch-traversal
    /// order; this keeps it.
    #[serde(default, skip_serializing_if = "IndexMap::is_empty")]
    pub nodes: IndexMap<String, NodeInstance>,
    /// Topic FQN → wiring. Endpoint refs are `"<node FQN>/<endpoint>"`.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub topics: BTreeMap<String, TopicWiring>,
    /// Service FQN → wiring.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub services: BTreeMap<String, ServiceWiring>,
    /// Action FQN → wiring.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub actions: BTreeMap<String, ServiceWiring>,
}

/// One launch-file scope in the resolved include tree (Phase 48: group scopes
/// are folded into their file scope).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ScopeInfo {
    /// Parent scope id; `None` for the root.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parent: Option<String>,
    /// Source manifest path, when this scope had one (diagnostics only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub manifest: Option<String>,
    /// ROS package of the launch file, when known.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub package: Option<String>,
    /// Launch file basename (e.g. `control.launch.xml`), when known.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub file: Option<String>,
    /// Resolved launch arguments passed to this include (`<include>`'s
    /// `<arg>`s), for launch-tree inspection. File scopes only.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub args: BTreeMap<String, String>,
}

/// One resolved node instance.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct NodeInstance {
    /// Owning scope id.
    pub scope: String,
    /// ROS package.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pkg: Option<String>,
    /// Executable name (plain node).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exec: Option<String>,
    /// Composable plugin type (composable node) — mutually exclusive with
    /// `exec`; `container` names the hosting container node FQN.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plugin: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub container: Option<String>,
    /// ROS 2 lifecycle (managed) node — contracts are runtime-gated on the
    /// Active state.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub lifecycle: bool,
    /// R1-M6 — managed-node boot autostart policy (`none` = services
    /// registered, externally driven; `configure`; `active`). `None` =
    /// unspecified (consumer default `none`). Only meaningful when
    /// `lifecycle` is true.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lifecycle_autostart: Option<Autostart>,
    /// R1-M4 — RESOLVED ROS parameter values for this node. Parameters
    /// are system semantics (not spawn info — the two-artifact split
    /// excludes cmd/env/param FILES, not values); the embedded consumer
    /// has no record.json to read them from. Phase 46.3a: the producer
    /// pre-merges scope-wide `SetParameter`/global params (lower
    /// precedence) with node-specific inline `<param>` values (higher
    /// precedence, matching `node_cmdline.rs`'s `global` → `node_specific`
    /// chain order) BEFORE lowering into this map — global params have no
    /// separate field here, they're already folded in with the right
    /// precedence. See [`Self::params_files`] for externally-referenced
    /// YAML, which is NOT folded into this map (kept verbatim).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub params: BTreeMap<String, ParamValue>,
    /// Advisory scheduling criticality (`high` | `medium` | `low`) carried
    /// through from the manifest (RT config v2 §2.1). The resolver's
    /// `SchedMapper` already consumed it when deriving
    /// [`Execution::bindings`]; it rides along for runtime diagnosis
    /// dashboards. Unrecognized values are ignored, never an error.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub criticality: Option<String>,

    // -- Phase 46.1b — launch spawn inputs (docs/design/unified-system-model.md
    // decision (a)) -------------------------------------------------------
    //
    // The following fields are launch-derived spawn INPUTS play_launch's
    // Linux runtime consumes to derive argv/env at spawn time (Phase 46.3).
    // nano-ros consumes `remaps` (its entry codegen bakes the rules into
    // entity creation — nano-ros issue 0255 / phase-305 W3) and IGNORES the
    // other three — it has no argv/process/respawn model for its embedded
    // targets. They ride along per the "all launch info in the shared model"
    // principle rather than a play_launch-private side channel. Additive: old
    // models with none of these keys parse unchanged (all default to
    // empty/`None`).
    /// Topic/service name remappings (`<remap from= to=/>`), in launch
    /// declaration order. Regular nodes, containers, and composable nodes
    /// (`load_node`) may all carry remaps.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub remaps: Vec<Remap>,
    /// Extra CLI args appended after `--ros-args` (the launch API's
    /// `ros_arguments`/`ros_args`, distinct from the plain `args` a raw
    /// executable gets). Only regular nodes and containers have a process
    /// to append these to; composable nodes don't spawn one.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub ros_args: Vec<String>,
    /// Respawn-on-exit policy. `None` = launch default (no respawn).
    /// Nodes/containers only — composable nodes have no process lifecycle
    /// independent of their container.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub respawn: Option<bool>,
    /// Delay in seconds before a respawn attempt. Only meaningful when
    /// `respawn` is `Some(true)`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub respawn_delay: Option<f64>,
    /// Launch-declared environment variables (`<env name= value=/>`), in
    /// declaration order — a `Vec` (not `BTreeMap`) because launch env lists
    /// may legitimately repeat a name (last one spawn-time wins) and order
    /// is part of the launch author's intent. Nodes/containers only.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub env: Vec<EnvVar>,

    // -- Phase 46.3a — close the spawn-completeness gaps found while
    // auditing whether the model can drive `play_launch replay` (Wave B)
    // without `record.json` (`docs/design/unified-system-model.md`,
    // `.superpowers/sdd/p46-w3-analysis.md`). Additive: old models with
    // none of these keys parse unchanged (all default to empty). --------
    /// Extra non-ROS CLI arguments appended before `--ros-args` (the launch
    /// API's `arguments=[...]` / `<node><arg .../></node>`), distinct from
    /// [`Self::ros_args`] which lands *after* `--ros-args`. Regular nodes
    /// and containers only — composable nodes have no process to append
    /// these to.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub args: Vec<String>,
    /// Resolved content of externally referenced param YAML files
    /// (`<param from="…"/>`, or the launch API's `parameters=[...]` file
    /// list), one string per file, in declaration order. Already
    /// `$(var …)`-resolved by the parser (`load_and_resolve_param_file`) —
    /// carried VERBATIM, not flattened into [`Self::params`]: nested/
    /// wildcard YAML (`/**: ros__parameters: {...}`) cannot survive a flat
    /// `String -> ParamValue` map without losing the wildcard-vs-node
    /// namespace distinction. The consumer materializes each string to a
    /// temp file and passes `--params-file <path>` at spawn — the exact
    /// pattern `node_cmdline.rs` already uses for `NodeRecord::params_files`
    /// today. Regular nodes and containers only (composable nodes take
    /// parameters only via [`Self::params`] in their `LoadNode` request).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub params_files: Vec<String>,
    /// phase-54 / play_launch issue 0007 — the ORDERED parameter-source list.
    /// [`Self::params`] and [`Self::params_files`] are the legacy split views,
    /// which cannot express ROS's ordering (they force files-then-inline).
    /// When this is non-empty it is AUTHORITATIVE: fold it in order.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_sources: Vec<ParamSource>,
    /// Raw command line for `<executable>` tags — a non-ROS process with no
    /// package/executable identity ([`Self::pkg`] is `None` and
    /// [`Self::exec`] carries no meaningful ament-resolvable value in this
    /// case). Mirrors `NodeRecord::cmd` verbatim: the first element is the
    /// full command string (split on whitespace at spawn time, the same way
    /// `node_cmdline.rs::from_raw_executable` does it today), remaining
    /// elements are extra args appended as-is. Empty for every regular ROS
    /// node/container (`pkg` is `Some`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub raw_cmd: Vec<String>,
    /// Composable-node LoadNode extra arguments (⊃ `use_intra_process_comms`),
    /// forwarded verbatim into the `LoadNode` service request's
    /// `extra_arguments`. The C++ container reads only
    /// `use_intra_process_comms` from it today (per project docs); the rest
    /// rides along for forward-compat. Composable nodes only — regular
    /// nodes/containers have no equivalent load-time argument channel
    /// (their extra CLI input is [`Self::args`]/[`Self::ros_args`]).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub extra_args: BTreeMap<String, String>,

    // -- Phase 46.3b — provenance the model previously inferred and lost,
    // needed for a faithful model-driven spawn (`.superpowers/sdd/
    // p46-w3b-review.md`). Additive: old models without these keys parse
    // unchanged (`node_name` defaults to `None`, `is_container` to
    // `false`). ----------------------------------------------------------
    /// The node name AS DECLARED in the launch file (`<node name="…">` /
    /// `Node(name=…)`), distinct from [`Self::exec`] — `None` when the
    /// launch declared no `name=` at all.
    ///
    /// Why this can't be recovered from the `structure.nodes` FQN key: the
    /// key is `name.or(exec_name)` (Phase 46.3a's GAP-1 fallback, so
    /// `name=None` nodes aren't dropped from the model), which erases
    /// whether the last FQN segment came from an explicit `name=` or the
    /// `exec_name` fallback. The Linux spawn path needs that distinction to
    /// decide whether to emit the `-r __node:=<name>` remap: forcing
    /// `__node` onto a `name=None` node silently renames it away from its
    /// own internally-hardcoded default name, breaking service/action
    /// discovery for e.g. LifecycleNodes whose lifecycle services register
    /// under the internal name — the exact regression play_launch's
    /// `af7c524` ("Use None for the node name if it's not set instead of
    /// exec_name fallback") fixed on the record path. With this field the
    /// model path reproduces that conditional exactly (emit `__node` iff
    /// `node_name.is_some()`). Containers and composable nodes always carry
    /// `Some` (their record types have a non-optional name).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub node_name: Option<String>,
    /// Whether this instance is a `<node_container>` (as opposed to a plain
    /// `<node>` or a composable `<composable_node>`). The producer knows
    /// this unambiguously (it comes from the record's `dump.container[]`
    /// array); a consumer CANNOT reconstruct it from the other fields —
    /// `model_builder` emits containers with `plugin: None, container:
    /// None`, structurally identical to a plain node. The Linux spawn path
    /// keys the `--container-mode` package/executable override
    /// (`play_launch_container` + `--isolated`/`--use_multi_threaded_executor`)
    /// off this bit; inferring it by reverse-resolving composable
    /// `container=` references (the pre-46.3b heuristic) could misclassify
    /// a regular node whose name coincides with a dangling/ambiguous
    /// composable target and then corrupt its executable. Plain nodes and
    /// composable nodes carry `false`.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub is_container: bool,
}

/// One `<remap from= to=/>` pair. Named struct (not a bare tuple) so the
/// YAML reads as `- from: … / to: …` — same "named struct over tuple"
/// precedent as [`ros_launch_manifest_sched::SegmentNode`].
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Remap {
    pub from: String,
    pub to: String,
}

/// One `<env name= value=/>` pair.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct EnvVar {
    pub name: String,
    pub value: String,
}

/// R1-M6 — lifecycle boot autostart policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Autostart {
    None,
    Configure,
    Active,
}

/// R1-M4 — a resolved ROS parameter value. Untagged: YAML scalars map
/// naturally; order matters (Bool before Int before Float before Str so
/// `true`/`1`/`1.5`/`"x"` each hit the right arm; a float-typed param
/// authored as `1` must be written `1.0`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ParamValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
    StrList(Vec<String>),
}

impl ParamValue {
    /// Render for a consumer that bakes parameters as STRINGS — nano-ros's
    /// compile-time entry codegen, where the value is re-typed at runtime by
    /// inference over this text.
    ///
    /// Single-sourced because the inference is unforgiving: `1.0f64.to_string()`
    /// is `"1"`, which re-types as an INTEGER and silently changes a launch
    /// double's type. `{:?}` keeps the `.0`. Every consumer that hand-rolled
    /// this match got at least one arm wrong.
    pub fn to_bake_string(&self) -> String {
        match self {
            ParamValue::Bool(b) => b.to_string(),
            ParamValue::Int(i) => i.to_string(),
            ParamValue::Float(f) => format!("{f:?}"),
            ParamValue::Str(s) => s.clone(),
            ParamValue::StrList(l) => l.join(","),
        }
    }
}

/// phase-54 / play_launch issue 0007 — one parameter source, in ROS's ORDERED
/// model.
///
/// `launch_ros` keeps a single list per node: `parse_nested_parameters` appends
/// one entry per `<param>` child in document order, and `execute` emits
/// `--params-file` / `-p` in that order — materializing an inline dict into a
/// temp params FILE first. Kind therefore carries NO precedence; position does,
/// and a file written after an inline value wins.
///
/// A consumer that cannot delegate to rcl (the nano-ros compile-time bake) must
/// fold this list IN ORDER, applying [`Self::File`] entries with FILE semantics
/// (`ros__parameters` sections, `/**` and partial wildcards) rather than as a
/// flat key/value overlay.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ParamSource {
    /// An inline `<param name= value=/>`, or a global param — globals occupy
    /// the HEAD of the list, mirroring ROS applying them before the node's own.
    Inline { name: String, value: ParamValue },
    /// A `<param from=/>` file: the resolved YAML CONTENT, same shape as the
    /// entries of [`NodeInstance::params_files`].
    File { content: String },
}

impl NodeInstance {
    /// nano-ros #276 — project `params_files` YAML into concrete parameter
    /// values, merged under the inline [`Self::params`].
    ///
    /// Embedded consumers have no launch runtime to pass `--params-file` to:
    /// they bake parameters into the image, so an upstream node that declares
    /// a parameter with NO default and receives its value from a
    /// `config/*.param.yaml` needs those values resolved at bake time. This is
    /// that projection — the same precedence a spawn would produce
    /// (param files in declaration order, each overriding the previous; inline
    /// `<param>` values win over all of them).
    ///
    /// Section matching follows ROS 2: a file maps NODE KEYS to a
    /// `ros__parameters` block. A key matches this node when it is the
    /// wildcard `/**`, the node's fully-qualified name, or its bare name
    /// (with or without a leading `/`). Nested maps under `ros__parameters`
    /// flatten with `.` (`qos.depth: 10` → `"qos.depth"`), matching rclcpp's
    /// nested-parameter naming. Sequences become [`ParamValue::StrList`];
    /// nulls and unmatched sections are skipped. A file that fails to parse
    /// is skipped (the resolver already validated it; a consumer must not
    /// hard-fail a bake on a YAML dialect it does not model).
    pub fn resolved_params(&self, fqn: &str) -> BTreeMap<String, ParamValue> {
        let bare = fqn.rsplit('/').next().unwrap_or(fqn);
        let mut out: BTreeMap<String, ParamValue> = BTreeMap::new();

        // phase-54 — when the ORDERED list is present it is authoritative: fold
        // in list order, so a file written AFTER an inline value wins (ROS's
        // rule; the legacy split below cannot express it).
        if !self.param_sources.is_empty() {
            for src in &self.param_sources {
                match src {
                    ParamSource::Inline { name, value } => {
                        out.insert(name.clone(), value.clone());
                    }
                    ParamSource::File { content } => {
                        merge_param_file(content, fqn, bare, &mut out);
                    }
                }
            }
            return out;
        }

        for raw in &self.params_files {
            merge_param_file(raw, fqn, bare, &mut out);
        }
        // Legacy split view: inline `<param>` values are highest precedence.
        for (k, v) in &self.params {
            out.insert(k.clone(), v.clone());
        }
        out
    }
}

/// Merge one param-file YAML's matching sections into `out`.
///
/// Section precedence inside a file is by SPECIFICITY, not by textual order:
/// rcl buckets a file's entries per node and lets a node-specific block
/// override what `/**` set, however the two are ordered in the YAML. So a file
/// that writes `/ctrl/planner:` above `/**:` still resolves `planner`'s own
/// value — ordering only decides between sources ([`ParamSource`]), never
/// between sections of one file.
fn merge_param_file(raw: &str, fqn: &str, bare: &str, out: &mut BTreeMap<String, ParamValue>) {
    let Ok(doc) = serde_yaml_ng::from_str::<serde_yaml_ng::Value>(raw) else {
        return;
    };
    let serde_yaml_ng::Value::Mapping(sections) = doc else {
        return;
    };
    let mut matched: Vec<(u8, &serde_yaml_ng::Value)> = Vec::new();
    for (key, body) in &sections {
        let Some(k) = key.as_str() else { continue };
        if !node_key_matches(k, fqn, bare) {
            continue;
        }
        let Some(params) = body.get("ros__parameters") else {
            continue;
        };
        matched.push((key_specificity(k), params));
    }
    // Stable: equally-specific sections keep file order, so two `/**` blocks
    // (or two spellings of the same node) still resolve last-wins.
    matched.sort_by_key(|(rank, _)| *rank);
    for (_, params) in matched {
        flatten_params("", params, out);
    }
}

/// How specific a param-file node key is: pure wildcard < partial wildcard <
/// literal. Only the ORDER matters, not the numbers.
fn key_specificity(key: &str) -> u8 {
    match key.trim().trim_start_matches('/') {
        "**" | "*" => 0,
        k if k.contains('*') => 1,
        _ => 2,
    }
}

/// Does a param-file node key select this node?
///
/// Mirrors `rcl_yaml_param_parser`'s matching so a bake-time projection sees
/// the same sections a spawned node would: an exact fully-qualified name, or a
/// pattern using `**` (matches any number of namespace segments, including
/// none) and `*` (exactly one segment) — e.g. `/**`, `/sensing/**`,
/// `/*/planner`, `/foo/*/bar`. A bare node name (no leading `/`) is also
/// accepted as the convenience form launch files commonly use for
/// root-namespace nodes.
fn node_key_matches(key: &str, fqn: &str, bare: &str) -> bool {
    if key == fqn {
        return true;
    }
    let segs: Vec<&str> = key.trim_start_matches('/').split('/').collect();
    // Bare-name convenience: a single literal segment naming the node.
    if segs.len() == 1 && !segs[0].contains('*') {
        return segs[0] == bare;
    }
    let target: Vec<&str> = fqn.trim_start_matches('/').split('/').collect();
    glob_match(&segs, &target)
}

/// Segment-wise glob: `**` consumes any number of segments, `*` exactly one.
fn glob_match(pat: &[&str], target: &[&str]) -> bool {
    match pat.split_first() {
        None => target.is_empty(),
        Some((&"**", rest)) => {
            // `**` may consume 0..=target.len() segments.
            (0..=target.len()).any(|skip| glob_match(rest, &target[skip..]))
        }
        Some((&head, rest)) => match target.split_first() {
            Some((&t, t_rest)) if head == "*" || head == t => glob_match(rest, t_rest),
            _ => false,
        },
    }
}

/// Flatten a `ros__parameters` subtree into dotted keys.
fn flatten_params(
    prefix: &str,
    node: &serde_yaml_ng::Value,
    out: &mut BTreeMap<String, ParamValue>,
) {
    let serde_yaml_ng::Value::Mapping(map) = node else {
        return;
    };
    for (k, v) in map {
        let Some(name) = k.as_str() else { continue };
        let full = if prefix.is_empty() {
            name.to_string()
        } else {
            format!("{prefix}.{name}")
        };
        match v {
            serde_yaml_ng::Value::Mapping(_) => flatten_params(&full, v, out),
            serde_yaml_ng::Value::Bool(b) => {
                out.insert(full, ParamValue::Bool(*b));
            }
            serde_yaml_ng::Value::Number(n) => {
                if let Some(i) = n.as_i64() {
                    out.insert(full, ParamValue::Int(i));
                } else if let Some(f) = n.as_f64() {
                    out.insert(full, ParamValue::Float(f));
                }
            }
            serde_yaml_ng::Value::String(st) => {
                out.insert(full, ParamValue::Str(st.clone()));
            }
            serde_yaml_ng::Value::Sequence(seq) => {
                let items: Vec<String> = seq
                    .iter()
                    .map(|e| match e {
                        serde_yaml_ng::Value::String(s) => s.clone(),
                        serde_yaml_ng::Value::Bool(b) => b.to_string(),
                        serde_yaml_ng::Value::Number(n) => n.to_string(),
                        _ => String::new(),
                    })
                    .collect();
                out.insert(full, ParamValue::StrList(items));
            }
            _ => {}
        }
    }
}

/// Topic wiring: type + endpoint refs (`"<node FQN>/<endpoint>"`).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TopicWiring {
    /// ROS message type (`pkg/msg/Name`).
    #[serde(rename = "type")]
    pub msg_type: String,
    #[serde(rename = "pub", default, skip_serializing_if = "Vec::is_empty")]
    pub publishers: Vec<String>,
    #[serde(rename = "sub", default, skip_serializing_if = "Vec::is_empty")]
    pub subscribers: Vec<String>,
}

/// Service/action wiring: type + server/client endpoint refs.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ServiceWiring {
    #[serde(rename = "type")]
    pub srv_type: String,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub server: Vec<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub client: Vec<String>,
}

// ---------------------------------------------------------------------------
// layer 2 — contracts
// ---------------------------------------------------------------------------

/// Post-merge contract numbers. Keys mirror layer 1: endpoint refs are
/// `"<node FQN>/<endpoint>"`, path keys are `"<owner>/<path name>"` where
/// the owner is a node FQN (node paths) or scope id (scope paths).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Contracts {
    /// Publisher endpoint guarantees.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub pub_endpoints: BTreeMap<String, PubContract>,
    /// Subscriber endpoint assumptions.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub sub_endpoints: BTreeMap<String, SubContract>,
    /// Service server guarantees.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub srv_endpoints: BTreeMap<String, SrvContract>,
    /// Node processing paths (take → publish).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub node_paths: BTreeMap<String, PathContract>,
    /// Scope end-to-end paths (entry topic → exit topic within the subtree).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub scope_paths: BTreeMap<String, PathContract>,
    /// Per-topic channel contracts (rate, transport, drops, QoS).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub topics: BTreeMap<String, TopicContract>,
    /// Externally-provided topics: FQN → which side is external
    /// (`pub` | `sub` | `both`). Dangling-entity and runtime graph checks
    /// skip the external side.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub externals: BTreeMap<String, ExternalSide>,
}

impl Contracts {
    pub fn is_empty(&self) -> bool {
        self.pub_endpoints.is_empty()
            && self.sub_endpoints.is_empty()
            && self.srv_endpoints.is_empty()
            && self.node_paths.is_empty()
            && self.scope_paths.is_empty()
            && self.topics.is_empty()
            && self.externals.is_empty()
    }
}

/// Which side of an external topic lives outside the modeled system.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ExternalSide {
    Pub,
    Sub,
    Both,
}

/// Publisher guarantee.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PubContract {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_rate_hz: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_rate_hz: Option<f64>,
    /// Max deviation from the ideal period.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jitter_ms: Option<f64>,
    /// R1-M5 — per-endpoint QoS (overrides the topic-level profile for
    /// this endpoint; manifest-side per-endpoint QoS + the retired 211.H
    /// launch-param overlay both land here).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub qos: Option<Qos>,
}

/// Subscriber assumption.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SubContract {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min_rate_hz: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_rate_hz: Option<f64>,
    /// Max data age at receive: `now - header.stamp` at take.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_age_ms: Option<f64>,
    /// Polled (read-latest), not causal — breaks cycles in the dataflow DAG.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub state: bool,
    /// Must receive at least once before the node is operational.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub required: bool,
    /// R1-M5 — per-endpoint QoS (see [`PubContract::qos`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub qos: Option<Qos>,
}

/// Service server guarantee.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SrvContract {
    /// Max request-to-response time.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_response_ms: Option<f64>,
}

/// A causal path with a latency budget. Node paths: input/output are
/// endpoint refs and the budget is pure processing (take → publish). Scope
/// paths: input/output are topic FQNs and the budget is E2E across the
/// scope's subtree (includes internal transport); only scope paths carry
/// drop budgets.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PathContract {
    /// Empty = periodic (timer-driven).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub input: Vec<String>,
    pub output: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_latency_ms: Option<f64>,
    /// Multi-input stamp matching.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub correlation: Option<Correlation>,
    /// Max `header.stamp` spread between correlated inputs; required when
    /// `correlation` is `timestamp`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tolerance_ms: Option<f64>,
    /// E2E drop budget (scope paths only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub drop: Option<DropContract>,
}

/// Multi-input correlation mode.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Correlation {
    Timestamp,
    Latest,
}

/// Per-topic channel contract.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TopicContract {
    /// Negotiated channel rate.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rate_hz: Option<f64>,
    /// Worst-case transport latency for this hop; undeclared hops contribute
    /// 0 to scope budgets (absorbed into the residual).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_transport_ms: Option<f64>,
    /// Transport drop budget.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub drop: Option<DropContract>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub qos: Option<Qos>,
}

/// Drop budget, resolved to a fraction (the manifest's `N / W` form is
/// normalized by the resolver).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct DropContract {
    /// Long-run drop fraction in `[0, 1]`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_drop_rate: Option<f64>,
    /// Never lose more than this many in a row (runtime-only check).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_consecutive: Option<u32>,
}

/// QoS profile (string forms match the manifest format reference).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Qos {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reliability: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub durability: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub history: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub depth: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lifespan_ms: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub liveliness: Option<String>,
}

// ---------------------------------------------------------------------------
// layer 3 — execution / deployment
// ---------------------------------------------------------------------------

/// Integrator-owned placement + scheduling. play_launch's runtime consumes
/// the `linux` deploy subset; the nano-ros build system consumes `mcu:*`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Execution {
    /// Node FQN → placement.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub deploy: BTreeMap<String, Deploy>,
    /// Tier name → definition (sched crate schema — see [`TierDef`]).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub tiers: BTreeMap<String, TierDef>,
    /// `"<node FQN>"` or `"<node FQN>/<callback group>"` → tier name.
    /// RESOLVED form: the resolver flattens the authoring-side sparse
    /// `assign` rules (`sched::AssignRule`, scope prefixes + node lists)
    /// into explicit per-node bindings — early binding, like everything
    /// else in the model.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub bindings: BTreeMap<String, String>,
    /// R1-M2 — transport/session declarations (network identity + the
    /// (rmw, locator, domain) session tuple). Folds multi-domain
    /// routing: a node binds a transport by `id`, each transport is one
    /// session. The embedded boot bake reads its board's transport here.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub transports: Vec<Transport>,
    /// R1-M3 — in-binary topic-relay bridges (nano-ros RFC-0009 shape).
    /// Topic types resolve from layer 1 wiring, never written here.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bridges: Vec<Bridge>,
    /// R1-M3 — system capability axes (`safety`, `param_services`, …).
    /// Unknown names are a consumer bake-time error.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<String>,
    // NOTE: the resolved scheduling plan is NOT carried in the model
    // (2026-07-20 maintainer decision — reverted the 45.2/45.3 `sched`
    // embedding). The model is INPUT only; causality + execution modeling is
    // each consumer's job (nano-ros RFC-0050/0052; scheduling.md §"Cross-repo
    // design agreement"). The chain/DAG algorithm is a standalone reusable
    // crate both runtimes call, not an embedded output.
}

/// R1-M2 — one transport/session declaration.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Transport {
    /// `ethernet` | `wifi` | `serial` | `can` | `loopback`.
    pub kind: String,
    /// Stable id nodes/bridges bind by; `None` ⇒ keyed by `rmw`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    /// IPv4 CIDR (`"10.0.2.50/24"`) or `"dhcp"` — ethernet/wifi.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ip: Option<String>,
    /// Ethernet MAC; `None` ⇒ board fused MAC.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mac: Option<String>,
    /// Default IPv4 gateway — ethernet.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gateway: Option<String>,
    /// NIC names this session multi-homes over (one discovery graph).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub interfaces: Vec<String>,
    /// WiFi SSID.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ssid: Option<String>,
    /// WiFi passphrase.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub password: Option<String>,
    /// Device handle (`"UART0"`, `"CAN0"`) — serial/can.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub device: Option<String>,
    /// Serial baud / CAN bitrate.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub baudrate: Option<u32>,
    /// RMW riding this transport; `None` ⇒ deploy/system default.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rmw: Option<String>,
    /// Session locator; `None` ⇒ platform default.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub locator: Option<String>,
    /// ROS domain this session joins; `None` ⇒ system default.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain: Option<u8>,
}

/// R1-M3 — one in-binary bridge.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Bridge {
    pub name: String,
    /// Source transport id (or rmw key).
    pub from: String,
    /// Destination transport id (or rmw key).
    pub to: String,
    /// Forwarded topic FQNs; empty ⇒ every declared topic.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub topics: Vec<String>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub bidirectional: bool,
}

impl Execution {
    pub fn is_empty(&self) -> bool {
        self.deploy.is_empty()
            && self.tiers.is_empty()
            && self.bindings.is_empty()
            && self.transports.is_empty()
            && self.bridges.is_empty()
            && self.features.is_empty()
    }
}

/// Where a node runs.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Deploy {
    /// `linux` or `mcu:<board>` (see [`Target`]). `None` = **board-agnostic**
    /// — the node is placed, but a multi-board system (one that also declares
    /// `kind = "embedded"` board builds) runs the same nodes on every board,
    /// so the consuming entry's own board decides (nano-ros issue 0356).
    /// Single-board placements are always `Some`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target: Option<Target>,
    /// R1-M1 — ROS domain for this node's session (RFC-0045 baked rung
    /// on embedded). `None` = system default.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub domain: Option<u8>,
    /// R1-M1 — session locator override. `None` = backend default.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub locator: Option<String>,
    /// R1-M1 — RMW backend for this node's session (`zenoh` | `xrce` |
    /// `cyclonedds` | …). `None` = system default. The embedded bake
    /// cannot pick a backend without one of the two.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rmw: Option<String>,
    /// R1-M1 — consumer-defined extras (build tuning, runner knobs:
    /// `profile`, `optimize`, `kind`, `framework`, cargo `features`…).
    /// Open map by design: these are per-consumer build semantics, not
    /// cross-runtime system semantics — but they ride the model so the
    /// consumer never parses `system.toml` directly (canonical-path
    /// decision).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub extra: BTreeMap<String, ExtraValue>,
}

/// R1-M1 — a deploy-extra value (untagged; Bool before Int before Float
/// before Str, then string lists).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ExtraValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
    StrList(Vec<String>),
}

/// Deployment target. Serialized as `linux` or `mcu:<board>`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum Target {
    #[default]
    Linux,
    Mcu {
        board: String,
    },
}

impl std::fmt::Display for Target {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Target::Linux => write!(f, "linux"),
            Target::Mcu { board } => write!(f, "mcu:{board}"),
        }
    }
}

impl std::str::FromStr for Target {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s == "linux" {
            return Ok(Target::Linux);
        }
        if let Some(board) = s.strip_prefix("mcu:") {
            if board.is_empty() {
                return Err("empty board in `mcu:<board>` target".into());
            }
            return Ok(Target::Mcu {
                board: board.to_string(),
            });
        }
        Err(format!(
            "unknown target `{s}` (expected `linux` or `mcu:<board>`)"
        ))
    }
}

impl Serialize for Target {
    fn serialize<S: serde::Serializer>(&self, ser: S) -> Result<S::Ok, S::Error> {
        ser.serialize_str(&self.to_string())
    }
}

impl<'de> Deserialize<'de> for Target {
    fn deserialize<D: serde::Deserializer<'de>>(de: D) -> Result<Self, D::Error> {
        let s = String::deserialize(de)?;
        s.parse().map_err(serde::de::Error::custom)
    }
}

/// Tier definitions reuse the sched crate's schema (RT config v2, phase 41):
/// a portable head (`class`, `deadline_us`, `period_us`, `budget_us`,
/// `deadline_policy`, `spin_period_us`) plus fixed per-platform placement
/// sub-tables (`posix`/`freertos`/`zephyr`/`threadx`/`nuttx` — `priority`,
/// `stack_bytes`, `core`, `sched_class`, `preempt_threshold`, per-platform
/// `deadline_us` override). One schema for the authoring form
/// (`system.toml`), the model, and the `SchedMapper` pipeline.
pub use ros_launch_manifest_sched::types::{TierDef, TierPlatformSpec};

#[cfg(test)]
mod order_tests {
    use super::*;

    /// Issue 0382 — `structure.nodes` keeps DECLARATION order through a
    /// serialize/deserialize round trip. A `BTreeMap` alphabetized it away, so
    /// a launch declaring `talker` before `listener` emitted an entry that
    /// constructed `listener` first. Names chosen so alphabetical != insertion.
    #[test]
    fn nodes_keep_declaration_order_through_yaml() {
        let mut st = Structure::default();
        for fqn in ["/talker", "/listener", "/aaa_last"] {
            st.nodes.insert(fqn.to_string(), NodeInstance::default());
        }
        let before: Vec<&str> = st.nodes.keys().map(String::as_str).collect();
        assert_eq!(before, vec!["/talker", "/listener", "/aaa_last"]);

        let yaml = serde_yaml_ng::to_string(&st).expect("serialize");
        // The wire form must not be alphabetized either — the emitter reads it
        // in file order.
        let t = yaml.find("/talker").expect("talker in yaml");
        let l = yaml.find("/listener").expect("listener in yaml");
        let a = yaml.find("/aaa_last").expect("aaa_last in yaml");
        assert!(t < l && l < a, "yaml lost declaration order:\n{yaml}");

        let back: Structure = serde_yaml_ng::from_str(&yaml).expect("deserialize");
        let after: Vec<&str> = back.nodes.keys().map(String::as_str).collect();
        assert_eq!(after, before, "round trip lost declaration order");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// nano-ros issue 0293 — `meta.inputs` paths must be portable.
    ///
    /// They used to be canonicalised into a COMMITTED artifact, so a model was
    /// only correct on the machine that produced it: everywhere else the
    /// consumer's existence check failed and it silently fell back to a fixed
    /// filename.
    #[test]
    fn input_paths_are_recorded_relative_to_the_base() {
        let tmp = tempfile::tempdir().unwrap();
        let bringup = tmp.path().join("src").join("demo_bringup");
        std::fs::create_dir_all(bringup.join("launch")).unwrap();
        let launch = bringup.join("launch").join("system.launch.xml");
        std::fs::write(&launch, "<launch/>").unwrap();
        let sys = bringup.join("system.toml");
        std::fs::write(&sys, "[system]\n").unwrap();

        assert_eq!(
            input_path_string(&launch, Some(&bringup)),
            "launch/system.launch.xml"
        );
        assert_eq!(input_path_string(&sys, Some(&bringup)), "system.toml");

        // No base -> absolute, the pre-0293 behaviour, kept for callers that
        // have no meaningful root.
        assert!(
            std::path::Path::new(&input_path_string(&launch, None)).is_absolute(),
            "without a base the path stays absolute"
        );

        // Outside the root (an include from a sibling package) has no useful
        // relative form, so it stays absolute rather than growing a `../..`
        // chain that means nothing in an install space.
        let elsewhere = tmp.path().join("other_pkg").join("extra.launch.xml");
        std::fs::create_dir_all(elsewhere.parent().unwrap()).unwrap();
        std::fs::write(&elsewhere, "<launch/>").unwrap();
        assert!(
            std::path::Path::new(&input_path_string(&elsewhere, Some(&bringup))).is_absolute(),
            "an input outside the base stays absolute"
        );
    }

    #[test]
    fn target_string_forms() {
        assert_eq!("linux".parse::<Target>().unwrap(), Target::Linux);
        assert_eq!(
            "mcu:stm32f4".parse::<Target>().unwrap(),
            Target::Mcu {
                board: "stm32f4".into()
            }
        );
        assert_eq!(
            Target::Mcu {
                board: "stm32f4".into()
            }
            .to_string(),
            "mcu:stm32f4"
        );
        assert!("mcu:".parse::<Target>().is_err());
        assert!("cloud".parse::<Target>().is_err());
    }

    #[test]
    fn version_gate_rejects_newer_schema() {
        let yaml = "meta:\n  version: 999\nstructure: {}\n";
        match SystemModel::from_yaml_str(yaml) {
            Err(ModelError::UnsupportedVersion {
                found: 999,
                supported,
            }) => {
                assert_eq!(supported, SCHEMA_VERSION)
            }
            other => panic!("expected UnsupportedVersion, got {other:?}"),
        }
    }

    #[test]
    fn empty_layers_are_omitted_from_yaml() {
        let model = SystemModel {
            meta: Meta {
                version: SCHEMA_VERSION,
                ..Default::default()
            },
            ..Default::default()
        };
        let yaml = model.to_yaml_string().unwrap();
        assert!(
            !yaml.contains("contracts:"),
            "empty contracts serialized: {yaml}"
        );
        assert!(
            !yaml.contains("execution:"),
            "empty execution serialized: {yaml}"
        );
    }

    /// Phase 46.1b — `NodeInstance` launch spawn fields round-trip with
    /// clean named-struct YAML shapes (not bare tuples): `remaps` as
    /// `- from: … / to: …` and `env` as `- name: … / value: …`, mirroring
    /// the `SegmentNode` precedent for pair lists in this crate.
    #[test]
    fn node_instance_launch_fields_roundtrip() {
        let node = NodeInstance {
            scope: "/perception".into(),
            pkg: Some("lidar_centerpoint".into()),
            exec: Some("detector_node".into()),
            remaps: vec![
                Remap {
                    from: "/points".into(),
                    to: "/sensing/lidar/points".into(),
                },
                Remap {
                    from: "/diagnostics".into(),
                    to: "/diag/detector".into(),
                },
            ],
            ros_args: vec!["--log-level".into(), "detector_node:=debug".into()],
            respawn: Some(true),
            respawn_delay: Some(2.5),
            env: vec![EnvVar {
                name: "CUDA_VISIBLE_DEVICES".into(),
                value: "0".into(),
            }],
            ..Default::default()
        };

        let yaml = serde_yaml_ng::to_string(&node).unwrap();
        // named struct shapes, not bare tuples.
        assert!(yaml.contains("from: /points"), "{yaml}");
        assert!(yaml.contains("to: /sensing/lidar/points"), "{yaml}");
        assert!(yaml.contains("name: CUDA_VISIBLE_DEVICES"), "{yaml}");
        assert!(yaml.contains("value: '0'"), "{yaml}");
        assert!(yaml.contains("respawn: true"), "{yaml}");
        assert!(yaml.contains("respawn_delay: 2.5"), "{yaml}");

        let reparsed: NodeInstance = serde_yaml_ng::from_str(&yaml).unwrap();
        assert_eq!(node, reparsed);
    }

    /// Backward-compat: a `NodeInstance` with none of the Phase 46.1b launch
    /// fields (pre-46 artifact) parses with every new field defaulting to
    /// empty/`None`, and re-emitting it invents none of the new keys.
    #[test]
    fn node_instance_without_launch_fields_parses_with_defaults() {
        let yaml = "\
scope: /perception
pkg: lidar_centerpoint
exec: detector_node
";
        let node: NodeInstance = serde_yaml_ng::from_str(yaml).unwrap();
        assert!(node.remaps.is_empty());
        assert!(node.ros_args.is_empty());
        assert_eq!(node.respawn, None);
        assert_eq!(node.respawn_delay, None);
        assert!(node.env.is_empty());

        let re_emitted = serde_yaml_ng::to_string(&node).unwrap();
        assert!(!re_emitted.contains("remaps:"), "{re_emitted}");
        assert!(!re_emitted.contains("ros_args:"), "{re_emitted}");
        assert!(!re_emitted.contains("respawn:"), "{re_emitted}");
        assert!(!re_emitted.contains("env:"), "{re_emitted}");
    }

    /// Phase 46.3a — the six spawn-completeness gap fields (`args`,
    /// `params_files`, `raw_cmd`, `extra_args`) round-trip. `args`/
    /// `params_files` on a regular node, `extra_args` on a composable node,
    /// `raw_cmd` on a raw-executable-shaped instance (no `pkg`/`exec`).
    #[test]
    fn node_instance_gap_fields_roundtrip() {
        let node = NodeInstance {
            scope: "/perception".into(),
            pkg: Some("lidar_centerpoint".into()),
            exec: Some("detector_node".into()),
            args: vec!["--verbose".into(), "--config".into(), "a.yaml".into()],
            params_files: vec!["/**:\n  ros__parameters:\n    a: 1\n".into()],
            ..Default::default()
        };
        let yaml = serde_yaml_ng::to_string(&node).unwrap();
        assert!(yaml.contains("args:"), "{yaml}");
        assert!(yaml.contains("--verbose"), "{yaml}");
        assert!(yaml.contains("params_files:"), "{yaml}");
        assert!(yaml.contains("ros__parameters"), "{yaml}");
        let reparsed: NodeInstance = serde_yaml_ng::from_str(&yaml).unwrap();
        assert_eq!(node, reparsed);

        let composable = NodeInstance {
            scope: "/perception".into(),
            plugin: Some("tracker::TrackerNode".into()),
            container: Some("/perception/pipeline_container".into()),
            extra_args: BTreeMap::from([(
                "use_intra_process_comms".to_string(),
                "True".to_string(),
            )]),
            ..Default::default()
        };
        let yaml = serde_yaml_ng::to_string(&composable).unwrap();
        assert!(yaml.contains("extra_args:"), "{yaml}");
        assert!(yaml.contains("use_intra_process_comms"), "{yaml}");
        let reparsed: NodeInstance = serde_yaml_ng::from_str(&yaml).unwrap();
        assert_eq!(composable, reparsed);

        let raw_exec = NodeInstance {
            scope: "/".into(),
            raw_cmd: vec!["/usr/bin/carla-server".into(), "-quality-level=Low".into()],
            ..Default::default()
        };
        let yaml = serde_yaml_ng::to_string(&raw_exec).unwrap();
        assert!(yaml.contains("raw_cmd:"), "{yaml}");
        assert!(yaml.contains("carla-server"), "{yaml}");
        let reparsed: NodeInstance = serde_yaml_ng::from_str(&yaml).unwrap();
        assert_eq!(raw_exec, reparsed);
    }

    /// Backward-compat: the Phase 46.3a fields default to empty and are
    /// omitted from re-emitted YAML when unset (pre-46.3a artifacts parse
    /// unchanged, and models that never touch these gaps stay noise-free).
    #[test]
    fn node_instance_without_gap_fields_parses_with_defaults() {
        let yaml = "\
scope: /perception
pkg: lidar_centerpoint
exec: detector_node
";
        let node: NodeInstance = serde_yaml_ng::from_str(yaml).unwrap();
        assert!(node.args.is_empty());
        assert!(node.params_files.is_empty());
        assert!(node.raw_cmd.is_empty());
        assert!(node.extra_args.is_empty());

        let re_emitted = serde_yaml_ng::to_string(&node).unwrap();
        assert!(!re_emitted.contains("args:"), "{re_emitted}");
        assert!(!re_emitted.contains("params_files:"), "{re_emitted}");
        assert!(!re_emitted.contains("raw_cmd:"), "{re_emitted}");
        assert!(!re_emitted.contains("extra_args:"), "{re_emitted}");
    }

    /// Phase 46.3b — `node_name` (declared name, distinct from `exec`) and
    /// `is_container` round-trip, and a `name=None` node emits neither
    /// key (the whole point: the model must be able to say "no declared
    /// name").
    #[test]
    fn node_instance_provenance_fields_roundtrip() {
        let named = NodeInstance {
            scope: "/perception".into(),
            pkg: Some("lidar_centerpoint".into()),
            exec: Some("detector_node".into()),
            node_name: Some("detector".into()),
            ..Default::default()
        };
        let yaml = serde_yaml_ng::to_string(&named).unwrap();
        assert!(yaml.contains("node_name: detector"), "{yaml}");
        assert!(!yaml.contains("is_container"), "{yaml}");
        let reparsed: NodeInstance = serde_yaml_ng::from_str(&yaml).unwrap();
        assert_eq!(named, reparsed);

        let container = NodeInstance {
            scope: "/perception".into(),
            pkg: Some("rclcpp_components".into()),
            exec: Some("component_container".into()),
            node_name: Some("pipeline_container".into()),
            is_container: true,
            ..Default::default()
        };
        let yaml = serde_yaml_ng::to_string(&container).unwrap();
        assert!(yaml.contains("is_container: true"), "{yaml}");
        let reparsed: NodeInstance = serde_yaml_ng::from_str(&yaml).unwrap();
        assert_eq!(container, reparsed);

        // name=None node: node_name is None, so no `node_name:` key at all
        // (that absence IS the "no declared name" signal the spawn path
        // reads).
        let unnamed = NodeInstance {
            scope: "/perception".into(),
            pkg: Some("lidar_centerpoint".into()),
            exec: Some("detector_node".into()),
            node_name: None,
            ..Default::default()
        };
        let yaml = serde_yaml_ng::to_string(&unnamed).unwrap();
        assert!(!yaml.contains("node_name"), "{yaml}");
        let reparsed: NodeInstance = serde_yaml_ng::from_str(&yaml).unwrap();
        assert_eq!(unnamed, reparsed);
        assert_eq!(reparsed.node_name, None);
    }

    /// Backward-compat: pre-46.3b artifacts (no `node_name`/`is_container`)
    /// parse with `node_name: None`, `is_container: false`.
    #[test]
    fn node_instance_without_provenance_fields_parses_with_defaults() {
        let yaml = "\
scope: /perception
pkg: lidar_centerpoint
exec: detector_node
";
        let node: NodeInstance = serde_yaml_ng::from_str(yaml).unwrap();
        assert_eq!(node.node_name, None);
        assert!(!node.is_container);

        let re_emitted = serde_yaml_ng::to_string(&node).unwrap();
        assert!(!re_emitted.contains("node_name"), "{re_emitted}");
        assert!(!re_emitted.contains("is_container"), "{re_emitted}");
    }
}
