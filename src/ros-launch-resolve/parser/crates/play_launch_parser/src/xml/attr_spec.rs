//! Per-element attribute allowlists, mirroring `launch_xml`'s
//! `assert_entity_completely_parsed()`.
//!
//! ROS 2's XML frontend rejects any attribute no action consumed. This
//! parser previously ignored unknown attributes silently, which let ROS 1
//! syntax (`<node machine="…">`) through and silently dropped six
//! attributes ROS 2 genuinely accepts.
//!
//! The tables below were measured against Humble by injecting one candidate
//! attribute at a time per element, not inferred from reading ROS 2 source.
//!
//! # Distro policy: the tables are the UNION across supported distros
//!
//! The justfile targets Humble (22.04) and Jazzy (24.04), and their frontend
//! attribute surfaces differ. Encoding one distro's surface exactly would
//! make valid launch files on the other a hard `UnexpectedAttribute` — the
//! precise failure this module exists to prevent, inverted (issue 0012).
//!
//! So an attribute accepted by ANY supported distro is listed here. On a
//! distro that lacks it we are then more permissive than that distro: the
//! attribute warns instead of erroring, and stock `ros2 launch` will reject
//! it there. That direction fails safe — a warning on a file the local ROS
//! would refuse, rather than a hard error on a file it would accept. It is
//! the same call already made for the legacy aliases (`<group ns=>`).
//!
//! Diffed from source (`external/diff_attrs.sh`), Humble vs Jazzy:
//!   - `execute_process` (so `<node>`, `<node_container>`, `<executable>`):
//!     Jazzy adds `respawn_max_retries`, `sigkill_timeout`, `sigterm_timeout`
//!   - `<include>`: Jazzy adds a `let` CHILD (`data_type=List[Entity]`)
//!   - `<node>`: Humble has `node-name`, dropped in Jazzy
//!   - `lifecycle_node`: Jazzy adds `autostart`, but this parser does not
//!     dispatch that element at all, so there is no spec to update

use crate::{
    error::{ParseError, Result},
    xml::entity::Entity,
};

/// What one launch element accepts.
#[derive(Debug)]
pub struct AttrSpec {
    /// Canonical element name.
    pub element: &'static str,
    /// Attributes this parser consumes. Silent.
    pub supported: &'static [&'static str],
    /// Attributes ROS 2 accepts that this parser does not implement.
    /// Warned about, not rejected — rejecting them would fail launch files
    /// that are valid ROS 2.
    pub known_unsupported: &'static [&'static str],
    /// Legal CHILD element names. In XML these are separate elements with
    /// their own specs and never appear as attributes; in YAML they are keys
    /// of the same mapping, so [`validate_yaml_keys`] accepts them and
    /// [`validate_attrs`] does not.
    pub children: &'static [&'static str],
}

/// `<node>` and `<node_container>` share ROS 2's attribute set exactly.
const NODE_SUPPORTED: &[&str] = &[
    "if",
    "unless",
    "pkg",
    "exec",
    "name",
    "namespace",
    "args",
    "output",
    "respawn",
    "respawn_delay",
];
const NODE_KNOWN_UNSUPPORTED: &[&str] = &[
    "exec_name",
    "ros_args",
    "launch-prefix",
    "cwd",
    "emulate_tty",
    "shell",
    // JAZZY-ONLY (union policy, see the module doc). Added to
    // `ExecuteProcess.parse` after Humble, and reached from `<node>` via
    // `super().parse(...)`. Humble rejects all three; we warn on both
    // distros rather than hard-erroring on Jazzy, where they are valid.
    "respawn_max_retries",
    "sigkill_timeout",
    "sigterm_timeout",
    // ROS 2 accepts and fully honors this (`ExecuteProcess.parse`, read via
    // `Node.parse`'s `super().parse(entity, parser, ignore=['cmd'])` —
    // `launch_ros/actions/node.py:309`; `on_exit="shutdown"` registers a
    // `Shutdown()` handler). Measured: `<node pkg=… exec=… on_exit=
    // "shutdown"/>` and the same on `<node_container>` both parse and
    // construct cleanly against Humble. Not implemented here — warn, don't
    // reject.
    "on_exit",
    // ROS 2's frontend consumes this without complaint — `node.py:316`
    // (`entity.get_attr('node-name', optional=True)`) unconditionally reads
    // it, so `assert_entity_completely_parsed()` never flags it as
    // "Unexpected attribute". BUT measured against Humble: giving it a
    // value always crashes construction with
    // `TypeError: Action.__init__() got an unexpected keyword argument
    // 'node_name'` — `Node.__init__` (node.py:119) has no `node_name`
    // parameter, only `name`; `kwargs['node_name']` is forwarded all the
    // way to `Action.__init__` and rejected there. This looks like dead/
    // broken code upstream (a pre-rename leftover), not a working alias —
    // so "ROS 2 accepts" here means only "the attribute name doesn't
    // produce an Unexpected-attribute error", not "the launch file works".
    // Warned, not rejected, to match that: rejecting it would still be
    // wrong (it's not the `UnexpectedAttribute` class of failure ROS 2
    // reports), and no launch file can be depending on `node-name`
    // functioning since it never has in Humble.
    "node-name",
];
const NODE_CHILDREN: &[&str] = &["param", "remap", "env"];

static SPECS: &[AttrSpec] = &[
    AttrSpec {
        element: "node",
        supported: NODE_SUPPORTED,
        known_unsupported: NODE_KNOWN_UNSUPPORTED,
        children: NODE_CHILDREN,
    },
    AttrSpec {
        element: "node_container",
        supported: NODE_SUPPORTED,
        known_unsupported: NODE_KNOWN_UNSUPPORTED,
        children: &[
            "param",
            "remap",
            "env",
            "composable_node",
            "composable-node",
        ],
    },
    AttrSpec {
        element: "composable_node",
        supported: &["if", "unless", "pkg", "plugin", "name", "namespace"],
        known_unsupported: &[],
        children: &["param", "remap", "extra_arg"],
    },
    AttrSpec {
        element: "load_composable_node",
        supported: &["if", "unless", "target"],
        known_unsupported: &[],
        children: &["composable_node", "composable-node"],
    },
    AttrSpec {
        element: "executable",
        supported: &[
            "if",
            "unless",
            "name",
            "output",
            "respawn",
            "respawn_delay",
            "cmd",
        ],
        // `on_exit`: same story as `<node>`'s — read unconditionally in
        // `ExecuteProcess.parse` (`execute_process.py:335`) and fully
        // honored (unlike `node-name`, which `ExecuteProcess.parse` never
        // reads at all — `<executable node-name=…>` is genuinely rejected
        // by ROS 2 with "Unexpected attribute", measured, so it stays out
        // of this table).
        known_unsupported: &[
            "launch-prefix",
            "cwd",
            "emulate_tty",
            "shell",
            "on_exit",
            // Jazzy-only, same union rationale as `<node>`.
            "respawn_max_retries",
            "sigkill_timeout",
            "sigterm_timeout",
        ],
        children: &["env", "arg"],
    },
    AttrSpec {
        element: "executable-arg",
        // A bare positional argument under `<executable>` — Rust-only: real
        // ROS 2 rejects `<arg>` as a child of `<executable>` entirely
        // (measured: `Unexpected nested tag(s) found in 'executable':
        // {'arg'}`, not an attribute-level rejection at all — outside what
        // this per-attribute table can express, since `children` is only
        // enforced for YAML). This spec exists solely so `actions/
        // executable.rs`'s arg-children validation (which reads only
        // `value`) doesn't fall through to the top-level `"arg"` spec above
        // and wrongly accept `default`/`description`/`name` here too. Not a
        // real XML element name.
        supported: &["value"],
        known_unsupported: &[],
        children: &[],
    },
    AttrSpec {
        element: "arg",
        // Top-level declaration form only: name/default/description(/if/
        // unless). Measured against real ROS 2: it REJECTS `value` here
        // (`Unexpected attribute(s) found in 'arg': {'value'}`) — `value`
        // only exists on the disjoint `<include>` child form, which is a
        // genuinely different ROS 2 entity validated separately (see the
        // `include-arg` spec below and its use in `actions/include.rs`).
        // A single unioned spec previously let this parser accept `value`
        // here, which real ROS 2 rejects (differential test finding).
        supported: &["if", "unless", "name", "default", "description"],
        known_unsupported: &[],
        // `<choice value="…"/>` — a CHILD entity, not an attribute.
        // `DeclareLaunchArgument.parse` reads it as
        // `entity.get_attr('choice', data_type=List[Entity], optional=True)`
        // (`launch/actions/declare_launch_argument.py:176`), so real ROS 2
        // accepts it on both frontends. The YAML frontend spells a child as a
        // key of the same mapping, which is why an empty `children` here made
        // `choice:` look like an unknown attribute and rejected every launch
        // file that constrains an argument — golfcart.launch.yaml among them.
        children: &["choice"],
    },
    AttrSpec {
        element: "choice",
        // The value list itself is not enforced yet: this parser accepts a
        // value outside the declared set where ROS 2 would reject it. That is
        // laxness, not divergence in what parses — see docs/issues.
        supported: &["value"],
        known_unsupported: &[],
        children: &[],
    },
    AttrSpec {
        element: "include-arg",
        // The `<arg>` child of `<include>` — NOT the same ROS 2 entity as
        // top-level `<arg>` above, despite sharing an XML tag name. Measured:
        // accepts only name/value; REJECTS default/description (`Unexpected
        // attribute(s) found in 'arg': {'description', 'zzz', 'default'}`)
        // and REJECTS if/unless too (`Unexpected attribute(s) found in
        // 'arg': {'if'}` / `{'unless'}`). Not a real XML element name —
        // `actions/include.rs` routes its arg-children validation here
        // explicitly instead of through the generic `"arg"` spec above.
        supported: &["name", "value"],
        known_unsupported: &[],
        children: &[],
    },
    AttrSpec {
        element: "let",
        supported: &["if", "unless", "name", "value"],
        known_unsupported: &[],
        children: &[],
    },
    AttrSpec {
        element: "group",
        supported: &["if", "unless", "scoped", "forwarding"],
        // ROS 2 rejects both (measured); this parser reads them. Warn rather
        // than break launch files that already depend on the behavior.
        known_unsupported: &["namespace", "ns"],
        // A group contains arbitrary actions; YAML nests them under a
        // `children:` key (measured: real ROS 2's `launch_yaml` `Entity`
        // reserves `children` as the generic key for nested sub-entities —
        // `launch_yaml/entity.py`'s `Entity.children` property reads
        // `self.__element['children']`) rather than as sibling keys, so
        // `children` itself is the one legal key to list here.
        children: &["children"],
    },
    AttrSpec {
        element: "include",
        supported: &["if", "unless", "file"],
        known_unsupported: &[],
        // `let` is Jazzy-only and is a CHILD (`data_type=List[Entity]` in
        // `include_launch_description.py`), carrying launch arguments the
        // same way `arg` does — not an attribute.
        children: &["arg", "let"],
    },
    AttrSpec {
        element: "set_env",
        supported: &["if", "unless", "name", "value"],
        known_unsupported: &[],
        children: &[],
    },
    // Measured: ROS 2 accepts `name` (required) plus the conditions, and
    // rejects anything else — `<unset_env name="A" zzz="x"/>` gives
    // `Unexpected attribute(s) found in `unset_env`: {'zzz'}`.
    AttrSpec {
        element: "unset_env",
        supported: &["if", "unless", "name"],
        known_unsupported: &[],
        children: &[],
    },
    // PARSER EXTENSION, not ROS 2. `<pop-ros-namespace/>` is rejected by
    // Humble's frontend outright — `Unrecognized entity of the type:
    // pop-ros-namespace` — so a launch file using it will not run under
    // stock `ros2 launch`. This parser dispatches it (popping the namespace
    // pushed by `<push-ros-namespace>`) and reads no attributes of its own.
    // The spec exists so unknown attributes on it are still rejected;
    // element-level conformance is a separate question (issue 0011).
    AttrSpec {
        element: "pop-ros-namespace",
        supported: &["if", "unless"],
        known_unsupported: &[],
        children: &[],
    },
    // PARSER EXTENSION, not ROS 2 — Humble gives `Unrecognized entity of the
    // type: declare_argument`. It mirrors `<arg>`'s declaration form and
    // additionally reads `choices`, which ROS 2 spells as a `<choice>` CHILD
    // of `<arg>` rather than an attribute. Same caveat as
    // `pop-ros-namespace`: a file using it is not portable to `ros2 launch`.
    AttrSpec {
        element: "declare_argument",
        supported: &["if", "unless", "name", "default", "description", "choices"],
        known_unsupported: &[],
        children: &[],
    },
    AttrSpec {
        element: "set_parameter",
        supported: &["if", "unless", "name", "value"],
        known_unsupported: &[],
        children: &[],
    },
    AttrSpec {
        element: "set_remap",
        supported: &["if", "unless", "from", "to"],
        known_unsupported: &[],
        children: &[],
    },
    AttrSpec {
        element: "push-ros-namespace",
        supported: &["if", "unless", "namespace"],
        // ROS 2 rejects `ns` (measured: `Unexpected attribute(s) found in
        // `push_ros_namespace`: {'ns'}`, even alongside a valid namespace=);
        // this parser reads it as a backwards-compat alias for `namespace`
        // (see the `push-ros-namespace` arm in traverser/entity.rs). Same
        // precedent as `<group namespace=/ns=>` above: warn rather than
        // break launch files that already depend on the alias.
        known_unsupported: &["ns"],
        children: &[],
    },
    // Child elements: NOT actions, so no if/unless (measured — ROS 2
    // rejects `if` on `<param>`, `<remap>`, and `<env>`).
    AttrSpec {
        element: "param",
        supported: &["name", "value", "from", "type"],
        // ROS 2 accepts `allow_substs` alongside `from=` (`launch_ros`
        // `Node._parse_nested_parameter_tuples()` reads it via
        // `param.get_attr('allow_substs', ...)` and threads it into
        // `ParameterFile.allow_substs`, gating whether `$(...)` patterns
        // inside the param file are substituted before load). Measured live
        // in Autoware (`autoware_diffusion_planner`, `autoware_lidar_transfusion`,
        // and others ship `<param from="..." allow_substs="true"/>`). This
        // parser always resolves substitutions in loaded YAML param files
        // (`params.rs::load_and_resolve_param_file()`) regardless of this
        // flag, so the attribute is accepted but has no effect — warn, don't
        // reject.
        known_unsupported: &["allow_substs"],
        children: &["param"], // nested param groups
    },
    AttrSpec {
        element: "remap",
        supported: &["from", "to"],
        known_unsupported: &[],
        children: &[],
    },
    AttrSpec {
        element: "env",
        supported: &["name", "value"],
        known_unsupported: &[],
        children: &[],
    },
];

/// Every spec, for tests that need to enumerate the full table (e.g. the
/// differential test's `CANDIDATES`-completeness assertion) rather than
/// look one up by name.
pub fn all_specs() -> &'static [AttrSpec] {
    SPECS
}

/// Map an element name (including hyphenated aliases the traverser accepts)
/// to its canonical spec. `None` means "not validated" — `<launch>` is not
/// strict in ROS 2, and any element this parser does not dispatch has no
/// spec to check against.
pub fn spec_for(element: &str) -> Option<&'static AttrSpec> {
    let canonical = match element {
        "set-env" => "set_env",
        "unset-env" => "unset_env",
        "set-remap" => "set_remap",
        "push_ros_namespace" => "push-ros-namespace",
        "pop_ros_namespace" => "pop-ros-namespace",
        "node-container" => "node_container",
        "composable-node" => "composable_node",
        "load-composable-node" => "load_composable_node",
        other => other,
    };
    SPECS.iter().find(|s| s.element == canonical)
}

/// Validate an XML entity's attributes. Child ELEMENTS are validated
/// separately, on their own entities, so `spec.children` is not consulted.
///
/// Generic rather than `&dyn Entity`: the traverser holds a concrete
/// `&XmlEntity<'_, '_>`.
pub fn validate_attrs<E: Entity + ?Sized>(entity: &E) -> Result<()> {
    let element = entity.type_name();
    let names: Vec<&str> = entity.attributes().into_iter().map(|(k, _)| k).collect();
    validate_named(element, &names)
}

/// Validate a bare `(element, attribute names)` pair.
pub fn validate_named(element: &str, attrs: &[&str]) -> Result<()> {
    check(element, attrs, false)
}

/// Validate an `<arg>` child entity against a context-specific spec key
/// (`"include-arg"`, `"executable-arg"`, …) instead of the generic spec
/// `validate_attrs` would derive from `child.type_name()` (which is always
/// the literal tag name `"arg"` and would resolve to the disjoint top-level
/// declaration spec). One `<arg>`-accepting call site == one call here;
/// centralizing the attribute-extraction + lookup means a future call site
/// can't accidentally fall back to `validate_attrs` and silently inherit the
/// wrong (top-level `"arg"`) attribute set.
///
/// Errors still name the real tag (`arg`), not the internal spec key — see
/// `display_name`.
pub fn validate_arg_child<E: Entity + ?Sized>(child: &E, spec_name: &str) -> Result<()> {
    let names: Vec<&str> = child.attributes().into_iter().map(|(k, _)| k).collect();
    check(spec_name, &names, false)
}

/// Validate YAML mapping keys. YAML nests child elements as keys of the same
/// mapping (`node: { pkg: …, param: [...] }`) where XML makes them separate
/// elements, so legal child names are accepted here and only here.
pub fn validate_yaml_keys(element: &str, keys: &[&str]) -> Result<()> {
    check(element, keys, true)
}

/// Map an internal, non-XML spec key back to the real tag name for error
/// messages. `include-arg`/`executable-arg` are lookup keys only — no launch
/// file ever spells them, so a user-facing "Unexpected attribute(s) found in
/// `include-arg`" would be confusing. Everything else displays as itself.
fn display_name(element: &str) -> &str {
    match element {
        "include-arg" | "executable-arg" => "arg",
        other => other,
    }
}

/// `(element, attribute)` pairs from `known_unsupported` whose value this
/// parser actually reads and acts on, despite ROS 2 rejecting the attribute
/// outright (or, for `namespace` on `<group>`, despite the attribute being
/// genuinely valid ROS 2 that this parser happens to also read under its
/// `ns` alias name). Legacy backwards-compat aliases, not unimplemented
/// features — see the doc comments on the `group` and `push-ros-namespace`
/// specs above. The generic `known_unsupported` warning text ("the value is
/// ignored") is false for these three: `actions/group.rs` reads `namespace`
/// and `ns`, `traverser/entity.rs`'s `push-ros-namespace` arm and
/// `traverser/yaml.rs:262` read `ns`. Listed here so `check()` can say so.
const HONORED_ALIASES: &[(&str, &str)] = &[
    ("group", "namespace"),
    ("group", "ns"),
    ("push-ros-namespace", "ns"),
];

fn check(element: &str, names: &[&str], allow_children: bool) -> Result<()> {
    let Some(spec) = spec_for(element) else {
        return Ok(());
    };

    let mut unexpected: Vec<&str> = Vec::new();
    for name in names {
        if spec.supported.contains(name) {
            continue;
        }
        if allow_children && spec.children.contains(name) {
            continue;
        }
        if spec.known_unsupported.contains(name) {
            // Keyed on `spec.element` (the canonical name), not the raw
            // `element` parameter — callers pass whatever spelling the
            // launch file used (e.g. `push_ros_namespace` underscore form),
            // and `HONORED_ALIASES` is written in canonical form.
            if HONORED_ALIASES.contains(&(spec.element, *name)) {
                log::warn!(
                    "<{} {name}=…> is rejected by real ROS 2 but this \
                     parser reads it as a backwards-compat alias; the \
                     value IS honored. Use --parser python if you need \
                     exact ROS 2 rejection behavior instead.",
                    display_name(element),
                );
            } else {
                log::warn!(
                    "<{} {name}=…> is valid ROS 2 but not supported by the \
                     Rust parser; the value is ignored. Use --parser python \
                     if you need it.",
                    display_name(element),
                );
            }
            continue;
        }
        unexpected.push(name);
    }

    if unexpected.is_empty() {
        return Ok(());
    }

    unexpected.sort_unstable();
    let rendered = unexpected
        .iter()
        .map(|a| format!("'{a}'"))
        .collect::<Vec<_>>()
        .join(", ");
    Err(ParseError::UnexpectedAttribute {
        element: display_name(element).to_string(),
        attributes: rendered,
    })
}
