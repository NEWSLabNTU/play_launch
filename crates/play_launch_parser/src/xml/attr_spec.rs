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
        known_unsupported: &["launch-prefix", "cwd", "emulate_tty", "shell"],
        children: &["env"],
    },
    AttrSpec {
        element: "arg",
        // The `<include>` child form uses name/value; the top-level
        // declaration form uses name/default/description. One element name,
        // so the spec is the union — the actions themselves reject a
        // nonsensical combination.
        supported: &["if", "unless", "name", "default", "description", "value"],
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
        // `children:` key rather than as sibling keys, so nothing to list.
        children: &[],
    },
    AttrSpec {
        element: "include",
        supported: &["if", "unless", "file"],
        known_unsupported: &[],
        children: &["arg"],
    },
    AttrSpec {
        element: "set_env",
        supported: &["if", "unless", "name", "value"],
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
        known_unsupported: &[],
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

/// Map an element name (including hyphenated aliases the traverser accepts)
/// to its canonical spec. `None` means "not validated" — `<launch>` is not
/// strict in ROS 2, and any element this parser does not dispatch has no
/// spec to check against.
pub fn spec_for(element: &str) -> Option<&'static AttrSpec> {
    let canonical = match element {
        "set-env" => "set_env",
        "set-remap" => "set_remap",
        "push_ros_namespace" => "push-ros-namespace",
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

/// Validate YAML mapping keys. YAML nests child elements as keys of the same
/// mapping (`node: { pkg: …, param: [...] }`) where XML makes them separate
/// elements, so legal child names are accepted here and only here.
pub fn validate_yaml_keys(element: &str, keys: &[&str]) -> Result<()> {
    check(element, keys, true)
}

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
            log::warn!(
                "<{element} {name}=…> is valid ROS 2 but not supported by the \
                 Rust parser; the value is ignored. Use --parser python if \
                 you need it."
            );
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
        element: element.to_string(),
        attributes: rendered,
    })
}
