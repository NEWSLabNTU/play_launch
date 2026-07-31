# Removing `<node machine=>` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the non-standard `<node machine="…">` attribute and its
`execution.deploy[fqn].host` model field from play_launch and its three
submodules, and make the Rust launch parser reject unknown attributes the
way ROS 2's `launch_xml` does.

**Architecture:** `machine=` is ROS 1 roslaunch syntax that the Rust parser
accepts and the Python parser (real ROS 2) rejects. Multi-host launching
moves to a standard `<arg>` + `if=` condition, which already works in both
parsers today, so partitioning happens at `resolve` time and no host field
is needed anywhere. A per-element attribute allowlist (`AttrSpec`) closes
the general class of bug: unknown attributes become errors, and six
attributes ROS 2 accepts but this parser drops become warnings.

**Tech Stack:** Rust 2024 edition, `roxmltree` (XML), `serde_yaml_ng`
(YAML launch files), `thiserror`, `log`, PyO3 (Python parser bridge),
`cargo nextest` (integration tests), `just` (task runner).

## Global Constraints

- Four git repositories are involved, nested as submodules. Bottom-up
  order is mandatory: `play_launch_parser` → `ros-launch-manifest` →
  `ros-launch-resolve` → `play_launch`. `nano-ros` gets documentation only.
- Repository paths on this machine:
  - `play_launch`: `/home/aeon/repos/play_launch`
  - `ros-launch-resolve`: `/home/aeon/repos/play_launch/src/ros-launch-resolve`
  - `play_launch_parser`: `/home/aeon/repos/play_launch/src/ros-launch-resolve/parser`
  - `ros-launch-manifest`: `/home/aeon/repos/play_launch/src/ros-launch-resolve/third-party/ros-launch-manifest`
  - `nano-ros`: `/home/aeon/repos/nano-ros`
- Work on a branch in every repo: `remove-machine-attr`. Push branches.
  **Never merge to `main` in any repo** without explicit approval.
- **ALWAYS** use `just build` in `play_launch`, never `colcon build`.
- **ALWAYS** use the Bash tool's `timeout` parameter, never a `timeout`
  command prefix.
- Temp files go in the repo's own `tmp/` (gitignored), never `/tmp`.
- Create files with the Write tool, never `cat` + heredoc.
- In `play_launch_parser`, run `just quality` (clippy `-D warnings`,
  `cargo fmt --check`, `cargo test --all`) before every commit. Baseline is
  371 tests without `--features ir`, 413 with it.
- Python tests in `play_launch_parser` must acquire `python_test_guard()`
  as their first line.
- `info!` is for end users only. Never promote `debug!` to `info!` to make
  a test pass; set `RUST_LOG` in the test instead.
- ROS 2 environment: `source /opt/ros/humble/setup.bash` (and
  `install/setup.bash` in `play_launch`) before running anything that
  touches the Python parser or resolves a launch file.

---

## File Structure

### `play_launch_parser`

| File | Responsibility | Change |
|---|---|---|
| `crates/play_launch_parser/src/xml/attr_spec.rs` | **New.** `AttrSpec` type, the per-element table, and `validate()`. Single source of truth for what each element accepts. | Create |
| `crates/play_launch_parser/src/xml/mod.rs` | Module wiring | Modify |
| `crates/play_launch_parser/src/error.rs` | `ParseError::UnexpectedAttribute` | Modify |
| `crates/play_launch_parser/src/traverser/entity.rs` | XML element dispatch — the one place every element passes through, so validation hooks in here rather than in 13 action modules | Modify |
| `crates/play_launch_parser/src/traverser/yaml.rs` | YAML mapping-key validation, reusing the same table; `machine: None` removal | Modify |
| `crates/play_launch_parser/src/actions/node.rs` | `machine` field, read, and resolve | Modify |
| `crates/play_launch_parser/src/actions/{container,executable}.rs` | `machine: None` initializers | Modify |
| `crates/play_launch_parser/src/{captures,ir}.rs` | `machine` field | Modify |
| `crates/play_launch_parser/src/record/{types,generator}.rs` | `machine` field and its resolution | Modify |
| `crates/play_launch_parser/src/traverser/ir_evaluator.rs`, `src/actions/mod.rs`, `src/python/bridge.rs` | `machine` plumbing | Modify |
| `crates/play_launch_parser/tests/attr_strictness.rs` | **New.** Rust-side unit tests for accept/warn/reject | Create |
| `crates/play_launch_parser/tests/attr_differential.rs` | **New.** Differential test against real ROS 2 | Create |

### `ros-launch-manifest`

| File | Responsibility | Change |
|---|---|---|
| `model/src/lib.rs` | `Deploy` struct — drop `host` | Modify |
| `model/src/system_config.rs` | Placement resolution — drop the `by_machine` fallback and `existing_host` | Modify |

### `ros-launch-resolve`

| File | Responsibility | Change |
|---|---|---|
| `resolve/src/ros/launch_dump.rs` | `NodeRecord.machine` | Modify |
| `resolve/src/ros/model_builder.rs` | `machine` → `deploy.host` mapping | Modify |
| `resolve/src/ros/manifest_loader.rs` | Three `machine: None` in test fixtures | Modify |

### `play_launch`

| File | Responsibility | Change |
|---|---|---|
| `src/play_launch/src/execution/node_cmdline.rs` | `machine` in the record adapter | Modify |
| `src/play_launch/src/commands/run.rs` | `machine: None` | Modify |
| `tests/fixtures/multihost/launch/multihost.launch.xml` | The fixture, rewritten to arg + condition | Modify |
| `tests/tests/resolve_multihost.rs` | Test, rewritten to node-set assertions | Modify |
| `docs/guide/multi-host.md` | **New.** User-facing pattern documentation | Create |
| `docs/design/unified-system-model.md`, `docs/roadmap/phase-46-unified_system_model.md`, `docs/roadmap/README.md`, `CLAUDE.md` | Corrections | Modify |

### `nano-ros`

| File | Responsibility | Change |
|---|---|---|
| `docs/issues/0356-machine-attr-is-ros-1-not-ros-2.md` | **New.** The finding + the breaking change | Create |
| `docs/roadmap/phase-325-multihost-via-launch-args.md` | **New.** Their migration work breakdown | Create |

---

## Task 1: `AttrSpec` type and the element table

**Repo:** `play_launch_parser`

**Files:**
- Create: `crates/play_launch_parser/src/xml/attr_spec.rs`
- Modify: `crates/play_launch_parser/src/xml/mod.rs`
- Modify: `crates/play_launch_parser/src/error.rs`
- Test: `crates/play_launch_parser/tests/attr_strictness.rs`

**Interfaces:**
- Consumes: `Entity` trait from `crate::xml::entity` (has
  `type_name() -> &str` and `attributes() -> Vec<(&str, &str)>`).
- Produces:
  - `pub struct AttrSpec { pub element: &'static str, pub supported: &'static [&'static str], pub known_unsupported: &'static [&'static str], pub children: &'static [&'static str] }`
  - `pub fn spec_for(element: &str) -> Option<&'static AttrSpec>`
  - `pub fn validate_attrs<E: Entity + ?Sized>(entity: &E) -> Result<()>` —
    XML attributes only.
  - `pub fn validate_named(element: &str, attrs: &[&str]) -> Result<()>` —
    the bare form, used by tests and by XML child elements.
  - `pub fn validate_yaml_keys(element: &str, keys: &[&str]) -> Result<()>` —
    YAML nests child elements as mapping keys, so this accepts
    `supported ∪ children`. Used in Task 8.
  - `ParseError::UnexpectedAttribute { element: String, attributes: String }`

**Why `children` exists:** in XML, `<param>` is a separate element validated
on its own, so `<node>`'s attribute list must NOT contain `param`. In YAML,
`param` is a *key of the node mapping*. One table serves both only if legal
child names are tracked separately from legal attributes.

`play_launch_parser`'s `lib.rs` declares `pub mod xml;` and `pub mod error;`
(but `mod traverser;` is private), so the integration tests below can reach
`play_launch_parser::xml::attr_spec` and `play_launch_parser::error::Result`.

- [ ] **Step 1: Create the branch**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
git checkout -b remove-machine-attr
```

- [ ] **Step 2: Add the error variant**

In `crates/play_launch_parser/src/error.rs`, add this variant to
`ParseError` immediately after the existing `UnexpectedElement` variant
(which is at line 29-30):

```rust
    /// An attribute no ROS 2 action consumes. Mirrors `launch_xml`'s
    /// `assert_entity_completely_parsed()`, whose message reads
    /// `Unexpected attribute(s) found in `node`: {'machine'}`.
    #[error("Unexpected attribute(s) found in `{element}`: {{{attributes}}}")]
    UnexpectedAttribute {
        element: String,
        /// Comma-separated, sorted, each quoted: `'machine', 'bogus'`.
        attributes: String,
    },
```

- [ ] **Step 3: Write the failing test**

Create `crates/play_launch_parser/tests/attr_strictness.rs`:

```rust
//! Per-element attribute strictness — the Rust parser must accept exactly
//! what ROS 2's `launch_xml` accepts, warn on ROS 2 attributes it does not
//! implement, and reject everything else.

use play_launch_parser::xml::attr_spec::{spec_for, validate_named};

#[test]
fn node_spec_lists_the_ros2_supported_attributes() {
    let spec = spec_for("node").expect("node has a spec");
    for attr in [
        "pkg", "exec", "name", "namespace", "args", "output", "respawn",
        "respawn_delay", "if", "unless",
    ] {
        assert!(
            spec.supported.contains(&attr),
            "`{attr}` must be supported on <node>; supported = {:?}",
            spec.supported
        );
    }
}

#[test]
fn node_spec_marks_unimplemented_ros2_attributes_as_known() {
    let spec = spec_for("node").expect("node has a spec");
    for attr in [
        "exec_name", "ros_args", "launch-prefix", "cwd", "emulate_tty", "shell",
    ] {
        assert!(
            spec.known_unsupported.contains(&attr),
            "`{attr}` is valid ROS 2 and must warn, not error; \
             known_unsupported = {:?}",
            spec.known_unsupported
        );
    }
}

#[test]
fn machine_is_rejected_on_node() {
    let err = validate_named("node", &["pkg", "exec", "machine"])
        .expect_err("machine= is ROS 1 syntax and must be rejected");
    let msg = err.to_string();
    assert!(
        msg.contains("Unexpected attribute(s) found in `node`"),
        "message must mirror launch_xml: {msg}"
    );
    assert!(msg.contains("'machine'"), "message must name the attribute: {msg}");
}

#[test]
fn unknown_attributes_are_rejected_and_all_named_at_once() {
    let err = validate_named("node", &["pkg", "exec", "zzz", "aaa"])
        .expect_err("unknown attributes must be rejected");
    let msg = err.to_string();
    assert!(msg.contains("'aaa'"), "{msg}");
    assert!(msg.contains("'zzz'"), "{msg}");
    // Sorted, so the message is stable across attribute ordering.
    assert!(
        msg.find("'aaa'").unwrap() < msg.find("'zzz'").unwrap(),
        "attributes must be sorted: {msg}"
    );
}

#[test]
fn known_unsupported_attributes_are_accepted() {
    validate_named("node", &["pkg", "exec", "launch-prefix"])
        .expect("launch-prefix is valid ROS 2 — warn, do not error");
}

#[test]
fn group_namespace_is_known_unsupported_not_supported() {
    // ROS 2 rejects `<group namespace=>` outright (measured); this parser
    // reads it. Downgrading to a warning tells authors their launch file is
    // not portable without breaking them today.
    let spec = spec_for("group").expect("group has a spec");
    assert!(spec.known_unsupported.contains(&"namespace"), "{spec:?}");
    assert!(spec.known_unsupported.contains(&"ns"), "{spec:?}");
    assert!(!spec.supported.contains(&"namespace"), "{spec:?}");
}

#[test]
fn child_elements_do_not_accept_conditions() {
    // `<param>`, `<remap>`, `<env>` are not actions — ROS 2 rejects if/unless
    // on them (measured).
    for element in ["param", "remap", "env"] {
        let spec = spec_for(element).unwrap_or_else(|| panic!("{element} has a spec"));
        assert!(
            !spec.supported.contains(&"if"),
            "<{element}> must not accept if=: {spec:?}"
        );
    }
}

#[test]
fn hyphenated_element_aliases_resolve_to_the_same_spec() {
    // The traverser dispatches `set-env` and `set_env` to the same action.
    assert_eq!(
        spec_for("set-env").map(|s| s.supported),
        spec_for("set_env").map(|s| s.supported),
    );
    assert_eq!(
        spec_for("node-container").map(|s| s.supported),
        spec_for("node_container").map(|s| s.supported),
    );
}

#[test]
fn the_launch_root_is_not_strict() {
    // Measured: `<launch zzz="1">` parses fine in ROS 2.
    assert!(
        spec_for("launch").is_none(),
        "<launch> must have no spec so validation skips it"
    );
}
```

- [ ] **Step 4: Run the test to verify it fails**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
cargo test --test attr_strictness 2>&1 | tail -20
```

Expected: FAIL to compile — `unresolved import play_launch_parser::xml::attr_spec`.

- [ ] **Step 5: Write the implementation**

Create `crates/play_launch_parser/src/xml/attr_spec.rs`:

```rust
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

use crate::error::{ParseError, Result};
use crate::xml::entity::Entity;

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
    "if", "unless", "pkg", "exec", "name", "namespace", "args", "output",
    "respawn", "respawn_delay",
];
const NODE_KNOWN_UNSUPPORTED: &[&str] = &[
    "exec_name", "ros_args", "launch-prefix", "cwd", "emulate_tty", "shell",
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
        children: &["param", "remap", "env", "composable_node", "composable-node"],
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
        supported: &["if", "unless", "name", "output", "respawn", "respawn_delay", "cmd"],
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
        known_unsupported: &[],
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
```

In `crates/play_launch_parser/src/xml/mod.rs`, add:

```rust
pub mod attr_spec;
```

- [ ] **Step 6: Run the test to verify it passes**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
cargo test --test attr_strictness 2>&1 | tail -20
```

Expected: PASS, 9 tests.

- [ ] **Step 7: Quality check**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
just quality 2>&1 | tail -30
```

Expected: clippy clean, fmt clean, 371 tests + the 9 new ones pass.

- [ ] **Step 8: Commit**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
git add crates/play_launch_parser/src/xml/attr_spec.rs \
        crates/play_launch_parser/src/xml/mod.rs \
        crates/play_launch_parser/src/error.rs \
        crates/play_launch_parser/tests/attr_strictness.rs
git commit -m "feat(xml): add per-element attribute allowlists

Mirrors launch_xml's assert_entity_completely_parsed(). Tables measured
against Humble by injecting one candidate attribute at a time per element.
Not yet wired into the traverser.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Wire validation into the XML traverser

**Repo:** `play_launch_parser`

**Files:**
- Modify: `crates/play_launch_parser/src/traverser/entity.rs`
- Test: `crates/play_launch_parser/tests/attr_strictness.rs` (extend)

**Interfaces:**
- Consumes: `validate_attrs` from Task 1.
- Produces: every XML element is validated — dispatched actions in the
  traverser, child elements in the action modules that parse them. No new
  public API.

**Two hook sites are required, not one.** `LaunchTraverser::traverse_entity`
(`traverser/entity.rs:17`) dispatches on `entity.type_name()` with arms for
`launch`, `arg`, `node`, `executable`, `include`, `group`, `let`,
`set_env`/`set-env`, `set_parameter`, `set_remap`/`set-remap`,
`push-ros-namespace`/`push_ros_namespace`, `node_container`/`node-container`,
`composable_node`/`composable-node`, and
`load_composable_node`/`load-composable-node`. But **child elements never
reach it** — `<param>`, `<remap>`, and `<env>` are consumed by
`NodeAction::from_entity`'s own `for child in entity.children()` loop
(`actions/node.rs`, the `match child.type_name()` around line 95), and the
same pattern appears in `container.rs`, `executable.rs`, and `include.rs`.
Each of those loops needs its own `validate_attrs(&child)?`.

**Validate before the condition check.** `traverse_entity` calls
`should_process_entity(entity, …)` first and returns early when a condition
excludes the element. ROS 2 validates attributes regardless of `if`/`unless`
(conditions are evaluated at launch time, after parsing), so validation must
come first — otherwise `<node machine="x" unless="true"/>` would slip
through.

- [ ] **Step 1: Write the failing test**

Append to `crates/play_launch_parser/tests/attr_strictness.rs`:

```rust
use std::io::Write;

fn parse_source(xml: &str) -> play_launch_parser::error::Result<()> {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tmp");
    std::fs::create_dir_all(&dir).expect("create tmp dir");
    let path = dir.join(format!("attr_strict_{}.launch.xml", std::process::id()));
    let mut fh = std::fs::File::create(&path).expect("write fixture");
    fh.write_all(xml.as_bytes()).expect("write fixture");
    drop(fh);
    let result = play_launch_parser::parse_launch_file(&path, Default::default());
    let _ = std::fs::remove_file(&path);
    result.map(|_| ())
}

#[test]
fn parsing_rejects_machine_on_node() {
    let err = parse_source(
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t" machine="robot1"/>
</launch>
"#,
    )
    .expect_err("machine= must fail the parse");
    let msg = err.to_string();
    assert!(
        msg.contains("Unexpected attribute(s) found in `node`") && msg.contains("'machine'"),
        "{msg}"
    );
}

#[test]
fn parsing_rejects_an_unknown_attribute() {
    let err = parse_source(
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t" zzz_bogus="x"/>
</launch>
"#,
    )
    .expect_err("unknown attributes must fail the parse");
    assert!(err.to_string().contains("'zzz_bogus'"), "{err}");
}

#[test]
fn parsing_accepts_a_known_unsupported_attribute() {
    parse_source(
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t" launch-prefix="nice -n 5"/>
</launch>
"#,
    )
    .expect("launch-prefix is valid ROS 2 — must parse with a warning");
}

#[test]
fn parsing_accepts_an_unknown_attribute_on_the_launch_root() {
    parse_source(
        r#"<launch zzz="1">
  <node pkg="demo_nodes_cpp" exec="talker" name="t"/>
</launch>
"#,
    )
    .expect("<launch> is not strict in ROS 2");
}

#[test]
fn parsing_rejects_an_unknown_attribute_on_a_child_element() {
    // `<param>`, `<remap>`, `<env>` never reach the traverser dispatch —
    // the action's own children loop consumes them, so they need their own
    // validation hook.
    let err = parse_source(
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t">
    <remap from="a" to="b" zzz_bogus="x"/>
  </node>
</launch>
"#,
    )
    .expect_err("unknown attributes on <remap> must fail the parse");
    let msg = err.to_string();
    assert!(
        msg.contains("Unexpected attribute(s) found in `remap`") && msg.contains("'zzz_bogus'"),
        "{msg}"
    );
}

#[test]
fn parsing_validates_even_when_a_condition_excludes_the_element() {
    // ROS 2 parses (and validates) regardless of if/unless — conditions are
    // evaluated at launch time. This parser evaluates them at parse time, so
    // validation must run before the condition check.
    let err = parse_source(
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t" machine="robot1" unless="true"/>
</launch>
"#,
    )
    .expect_err("a skipped element must still be validated");
    assert!(err.to_string().contains("'machine'"), "{err}");
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
cargo test --test attr_strictness parsing_ 2>&1 | tail -20
```

Expected: `parsing_rejects_machine_on_node`,
`parsing_rejects_an_unknown_attribute`,
`parsing_rejects_an_unknown_attribute_on_a_child_element`, and
`parsing_validates_even_when_a_condition_excludes_the_element` all FAIL —
the parse currently succeeds in every case.

- [ ] **Step 3: Hook the traverser dispatch**

In `crates/play_launch_parser/src/traverser/entity.rs`, `traverse_entity`
begins at line 17. Insert validation as its **first** statement, before the
`should_process_entity` early return:

```rust
    pub(crate) fn traverse_entity(&mut self, entity: &XmlEntity) -> Result<()> {
        // Reject attributes ROS 2 would reject; warn on ROS 2 attributes we
        // do not implement. Runs BEFORE the condition check because ROS 2
        // validates regardless of if/unless — it evaluates conditions at
        // launch time, after parsing. `<launch>` and undispatched elements
        // have no spec and are skipped (see `xml::attr_spec::spec_for`).
        crate::xml::attr_spec::validate_attrs(entity)?;

        // Check if entity should be processed based on if/unless conditions
        if !should_process_entity(entity, &self.context)? {
```

- [ ] **Step 4: Hook the child-element loops**

Four action modules consume child elements directly, so those children never
pass through `traverse_entity`. In each, add validation as the first
statement inside the `for child in entity.children()` loop, before the
`match child.type_name()`:

```rust
        for child in entity.children() {
            // Child elements never reach `traverse_entity` — validate here.
            crate::xml::attr_spec::validate_attrs(&child)?;
            match child.type_name() {
```

Apply to:
- `src/actions/node.rs` — the loop around line 95 (`param`, `remap`, `env`,
  `composable_node`)
- `src/actions/container.rs` — its `composable_node` / `param` / `remap`
  loop
- `src/actions/executable.rs` — its `env` loop
- `src/actions/include.rs` — its `arg` loop

Locate each with:

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
grep -n "for child in entity.children()" crates/play_launch_parser/src/actions/*.rs
```

Add the hook to every hit that grep reports, not only the four named above.

- [ ] **Step 5: Run the test to verify it passes**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
cargo test --test attr_strictness 2>&1 | tail -20
```

Expected: PASS, 15 tests.

- [ ] **Step 6: Run the full suite to catch fixtures that now fail**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
cargo test --all 2>&1 | tail -40
```

Expected: 371 + 15 pass. **If existing fixtures now fail**, read the error
— it names the element and attribute. For each one, decide:
- the attribute is valid ROS 2 and this parser does not implement it → add
  it to that element's `known_unsupported`
- the attribute is valid ROS 2 and this parser *does* read it → it belongs
  in `supported`; the table missed it
- the attribute is not ROS 2 → the fixture is wrong; fix the fixture

Do not blanket-add attributes to `supported` to make tests pass. Verify
each against the probe method in Task 3.

- [ ] **Step 7: Quality check and commit**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
just quality 2>&1 | tail -30
git add -A
git commit -m "feat(xml): reject unknown attributes

Validated at two sites: the traverser dispatch for actions, and each action
module's own children loop for <param>/<remap>/<env>/<arg>, which never
reach the traverser. Runs before the if/unless check because ROS 2 validates
regardless of conditions. <launch> and undispatched elements are skipped,
matching ROS 2.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Differential test against real ROS 2

**Repo:** `play_launch_parser`

**Files:**
- Create: `crates/play_launch_parser/tests/attr_differential.rs`

**Interfaces:**
- Consumes: `spec_for`, `validate_named` from Task 1;
  `parse_launch_file` from the crate root.
- Produces: no API. A test that fails when the tables drift from ROS 2.

This is the safety net. The tables in Task 1 were measured once by hand; this
test re-measures them on every run, so a curation mistake surfaces by name
instead of as silent divergence.

**Critical:** each element's fixture must parse cleanly with **no** injected
attribute before any candidate is judged. During the design probe, a
`load_composable_node` fixture whose `<composable_node>` lacked a `name`
failed for an unrelated reason, which made every candidate look accepted.

- [ ] **Step 1: Write the test**

Create `crates/play_launch_parser/tests/attr_differential.rs`:

```rust
//! Differential attribute test: this parser must agree with real ROS 2 on
//! which attributes each element accepts.
//!
//! ROS 2 is the oracle (CLAUDE.md: "When Rust and Python parser behaviors
//! differ, Python's behavior is always correct"). Skips when no ROS 2
//! environment is available.

use play_launch_parser::xml::attr_spec::{spec_for, validate_named};
use std::io::Write;
use std::process::Command;

/// Minimal well-formed body per element, `{ATTR}` marking the injection
/// point. Every one of these must parse in ROS 2 with `{ATTR}` empty.
const FIXTURES: &[(&str, &str)] = &[
    ("node", r#"<node pkg="demo_nodes_cpp" exec="talker" name="t"{ATTR}/>"#),
    ("executable", r#"<executable cmd="echo hi"{ATTR}/>"#),
    ("arg", r#"<arg name="a" default="d"{ATTR}/>"#),
    ("let", r#"<let name="a" value="v"{ATTR}/>"#),
    ("group", r#"<group{ATTR}><let name="x" value="1"/></group>"#),
    ("set_env", r#"<set_env name="A" value="1"{ATTR}/>"#),
    ("set_parameter", r#"<set_parameter name="p" value="1"{ATTR}/>"#),
    ("set_remap", r#"<set_remap from="a" to="b"{ATTR}/>"#),
    ("push-ros-namespace", r#"<push-ros-namespace namespace="/n"{ATTR}/>"#),
    (
        "node_container",
        r#"<node_container pkg="rclcpp_components" exec="component_container" name="c" namespace=""{ATTR}/>"#,
    ),
    (
        "load_composable_node",
        r#"<load_composable_node target="/c"{ATTR}><composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Talker" name="t"/></load_composable_node>"#,
    ),
    (
        "composable_node",
        r#"<load_composable_node target="/c"><composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Talker" name="t"{ATTR}/></load_composable_node>"#,
    ),
    (
        "param",
        r#"<node pkg="demo_nodes_cpp" exec="talker" name="t"><param name="p" value="1"{ATTR}/></node>"#,
    ),
    (
        "remap",
        r#"<node pkg="demo_nodes_cpp" exec="talker" name="t"><remap from="a" to="b"{ATTR}/></node>"#,
    ),
    (
        "env",
        r#"<node pkg="demo_nodes_cpp" exec="talker" name="t"><env name="A" value="1"{ATTR}/></node>"#,
    ),
];

/// Every attribute worth probing, across all elements.
const CANDIDATES: &[&str] = &[
    "if", "unless", "pkg", "exec", "plugin", "name", "namespace", "ns",
    "args", "ros_args", "exec_name", "output", "respawn", "respawn_delay",
    "launch-prefix", "cwd", "emulate_tty", "shell", "cmd", "scoped",
    "forwarding", "default", "description", "value", "file", "target",
    "from", "to", "type", "machine", "zzz_bogus",
];

fn tmp_dir() -> std::path::PathBuf {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tmp/attr_diff");
    std::fs::create_dir_all(&dir).expect("create tmp dir");
    dir
}

fn write_fixture(body: &str, attr: Option<&str>, tag: &str) -> std::path::PathBuf {
    let injected = match attr {
        Some(a) => format!(" {a}=\"x\""),
        None => String::new(),
    };
    let xml = format!("<launch>\n  {}\n</launch>\n", body.replace("{ATTR}", &injected));
    let path = tmp_dir().join(format!("{tag}.launch.xml"));
    let mut fh = std::fs::File::create(&path).expect("write fixture");
    fh.write_all(xml.as_bytes()).expect("write fixture");
    path
}

/// Does real ROS 2 accept this file? `None` means no ROS 2 available.
fn ros2_accepts(path: &std::path::Path) -> Option<bool> {
    let script = format!(
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1 || exit 42; \
         python3 -c \"
from launch.frontend import Parser
import sys
try:
    root, parser = Parser.load(sys.argv[1])
    parser.parse_description(root)
    print('OK')
except Exception as exc:
    print('REJECT' if 'Unexpected attribute' in str(exc) else 'OK')
\" {}",
        path.display()
    );
    let out = Command::new("bash").arg("-c").arg(&script).output().ok()?;
    if out.status.code() == Some(42) {
        return None;
    }
    let stdout = String::from_utf8_lossy(&out.stdout);
    if stdout.contains("REJECT") {
        Some(false)
    } else if stdout.contains("OK") {
        Some(true)
    } else {
        None
    }
}

/// Does this parser accept it? `known_unsupported` counts as accepted —
/// it warns and continues, which is what ROS 2 does modulo behavior.
fn rust_accepts(element: &str, attr: Option<&str>) -> bool {
    let mut attrs: Vec<&str> = Vec::new();
    if let Some(a) = attr {
        attrs.push(a);
    }
    validate_named(element, &attrs).is_ok()
}

#[test]
fn every_fixture_has_a_passing_baseline() {
    let Some(_) = ros2_accepts(&write_fixture(FIXTURES[0].1, None, "baseline_probe")) else {
        eprintln!("skip: no ROS 2 environment");
        return;
    };
    for (element, body) in FIXTURES {
        let path = write_fixture(body, None, &format!("base_{element}"));
        assert_eq!(
            ros2_accepts(&path),
            Some(true),
            "baseline fixture for <{element}> must parse in ROS 2 with no \
             injected attribute; otherwise every candidate looks accepted \
             and this whole test is vacuous"
        );
    }
}

#[test]
fn rust_and_ros2_agree_on_every_candidate_attribute() {
    let Some(_) = ros2_accepts(&write_fixture(FIXTURES[0].1, None, "agree_probe")) else {
        eprintln!("skip: no ROS 2 environment");
        return;
    };

    let mut disagreements: Vec<String> = Vec::new();

    for (element, body) in FIXTURES {
        assert!(
            spec_for(element).is_some(),
            "<{element}> is in the differential matrix but has no AttrSpec"
        );
        for attr in CANDIDATES {
            let path = write_fixture(body, Some(attr), &format!("{element}_{attr}"));
            let Some(ros2) = ros2_accepts(&path) else {
                continue;
            };
            let rust = rust_accepts(element, Some(attr));
            if ros2 != rust {
                disagreements.push(format!(
                    "<{element} {attr}=>: ROS 2 {}, Rust {}",
                    if ros2 { "accepts" } else { "rejects" },
                    if rust { "accepts" } else { "rejects" },
                ));
            }
        }
    }

    assert!(
        disagreements.is_empty(),
        "attribute allowlists have drifted from ROS 2:\n  {}",
        disagreements.join("\n  ")
    );
}
```

- [ ] **Step 2: Run it**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
bash -c 'source /opt/ros/humble/setup.bash; cargo test --test attr_differential -- --nocapture' 2>&1 | tail -40
```

Expected: PASS. If `rust_and_ros2_agree_on_every_candidate_attribute` lists
disagreements, fix the **tables in `attr_spec.rs`**, not the test — ROS 2 is
the oracle. The one expected asymmetry is `<group namespace=>`, which ROS 2
rejects while this parser warns; if the test flags it, adjust
`rust_accepts` to treat `known_unsupported` as a documented divergence:

```rust
    // `known_unsupported` entries that ROS 2 REJECTS are deliberate: we warn
    // instead of breaking launch files that already use them.
    if let Some(spec) = spec_for(element) {
        if attr.is_some_and(|a| spec.known_unsupported.contains(&a)) {
            return true;
        }
    }
```

and add an explicit allow-list of known divergences at the top of the test:

```rust
/// Deliberate divergences: this parser warns where ROS 2 rejects, because
/// rejecting would break launch files that already rely on the behavior.
const ALLOWED_DIVERGENCES: &[(&str, &str)] = &[("group", "namespace"), ("group", "ns")];
```

filtering them out of `disagreements`.

- [ ] **Step 3: Quality check and commit**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
just quality 2>&1 | tail -30
git add crates/play_launch_parser/tests/attr_differential.rs
git commit -m "test(xml): differential attribute test against real ROS 2

Re-measures the allowlists on every run so curation mistakes surface by
name. Asserts a passing baseline per fixture first -- a fixture that fails
for an unrelated reason makes every candidate look accepted.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Remove `machine` from the parser

**Repo:** `play_launch_parser`

**Files:**
- Modify: `crates/play_launch_parser/src/actions/node.rs` (lines 46-47,
  80-82, 152, 179-181, 261)
- Modify: `crates/play_launch_parser/src/actions/container.rs` (line 281)
- Modify: `crates/play_launch_parser/src/actions/executable.rs` (line 126)
- Modify: `crates/play_launch_parser/src/actions/mod.rs` (line 70)
- Modify: `crates/play_launch_parser/src/captures.rs` (lines 17-19)
- Modify: `crates/play_launch_parser/src/ir.rs` (lines 150-151)
- Modify: `crates/play_launch_parser/src/traverser/ir_evaluator.rs` (lines
  309, 330)
- Modify: `crates/play_launch_parser/src/traverser/yaml.rs` (line 357)
- Modify: `crates/play_launch_parser/src/record/types.rs` (lines 289-294,
  422, 450)
- Modify: `crates/play_launch_parser/src/record/generator.rs` (lines
  332-334, 359, 532)
- Modify: `crates/play_launch_parser/src/python/bridge.rs` (line 49)

**Interfaces:**
- Consumes: nothing new.
- Produces: `NodeRecord`, `NodeCapture`, and the IR `Node` variant no longer
  have a `machine` field. `ros-launch-resolve` (Task 6) depends on this.

Task 2 already made `machine=` a hard error, so this task removes the now
dead capture path.

- [ ] **Step 1: Confirm the test from Task 2 still guards the behavior**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
cargo test --test attr_strictness parsing_rejects_machine_on_node 2>&1 | tail -10
```

Expected: PASS. This test is what keeps `machine=` rejected after the field
is gone.

- [ ] **Step 2: Remove the field from the XML action**

In `crates/play_launch_parser/src/actions/node.rs`, delete:
- the struct field and its doc comment (lines 46-47):
  ```rust
      /// `<node machine="…">` — target host for ROS 2 multi-host launch.
      pub machine: Option<Vec<Substitution>>,
  ```
- the attribute read (lines 80-82) including its comment:
  ```rust
          // `<node machine="…">` — ROS 2 multi-host launch target host.
          let machine = entity
              .optional_attr_str("machine")?
              .map(|s| parse_substitutions(&s))
              .transpose()?;
  ```
- the `machine,` entry in the `Ok(Self { … })` initializer (line 152)
- the resolve block in `to_capture()` (lines 179-181, starting
  `// Resolve `machine` (multi-host target host).`)
- the `machine,` entry in the `NodeCapture` construction (line 261)

- [ ] **Step 3: Remove the field everywhere else**

Delete `machine` from each remaining site listed under **Files**. All are
either a struct field with a doc comment, a `machine,` in a struct literal,
a `machine: None,` initializer, or a `machine: machine.as_ref().map(…)`
mapping. None have logic attached.

For `src/traverser/yaml.rs:357`, delete the whole line:
```rust
            machine: None, // YAML-launch node machine= deferred (XML is the primary path)
```

For `src/actions/container.rs:281` and `src/actions/executable.rs:126`,
delete the `machine: None` lines and their trailing comments
(`// container machine= routing deferred`, `// raw executables aren't
ROS-machine-routed`).

- [ ] **Step 4: Build to find every remaining reference**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
cargo build --all-targets 2>&1 | grep -E "^error|machine" | head -30
```

Expected: initially errors naming any site missed above. Fix until clean.

- [ ] **Step 5: Verify no references remain**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
grep -rn "\bmachine\b" --include="*.rs" crates/ src/ | grep -viE "state.?machine|same machine|real machine"
```

Expected: no output.

- [ ] **Step 6: Run everything, including the IR feature**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
cargo test --all 2>&1 | tail -20
cargo test --features ir 2>&1 | tail -20
```

Expected: both pass. The IR removal in `ir.rs` and `ir_evaluator.rs` is only
exercised under `--features ir`, so this second run is not optional.

- [ ] **Step 7: Quality check and commit**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
just quality 2>&1 | tail -30
git add -A
git commit -m "feat!: remove the ROS 1 \`<node machine=>\` attribute

machine= is roslaunch (ROS 1) syntax. launch_ros's Node.parse() has no such
attribute and launch_xml rejects it, so accepting it diverged from ROS 2 and
from our own Python parser. Multi-host launching moves to a standard <arg> +
if= condition, partitioning at resolve time.

BREAKING CHANGE: NodeRecord, NodeCapture, and the IR Node variant no longer
carry a \`machine\` field. Launch files using machine= now fail to parse.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
git push -u origin remove-machine-attr
```

---

## Task 5: Remove `Deploy.host` from the model

**Repo:** `ros-launch-manifest`

**Files:**
- Modify: `model/src/lib.rs` (lines 961-973)
- Modify: `model/src/system_config.rs` (lines ~300-330, ~369-377, ~573-618)

**Interfaces:**
- Consumes: nothing.
- Produces: `Deploy` no longer has a `host: Option<String>` field.
  `ros-launch-resolve` (Task 6) depends on this.

**Behavior change worth understanding before you start:** with multiple
in-scope `[deploy.*]` blocks, a node could be placed by its `machine=`-derived
host alone. After this change every node needs an explicit `nodes = [..]`
entry or placement hard-errors with "is not placed". That is the pre-0291
behavior, restored deliberately — `machine=` no longer exists to derive a
host from.

- [ ] **Step 1: Create the branch**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/third-party/ros-launch-manifest
git checkout -b remove-machine-attr
```

- [ ] **Step 2: Delete the obsolete test**

In `model/src/system_config.rs`, delete the whole
`node_machine_attribute_places_without_a_nodes_list` test (lines ~573-618)
together with its doc comment beginning `/// nano-ros issue 0291 —`.

- [ ] **Step 3: Run the suite to see what the deletion exposes**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/third-party/ros-launch-manifest
cargo test 2>&1 | tail -20
```

Expected: PASS. The deleted test was the only one exercising `by_machine`.

- [ ] **Step 4: Delete the `by_machine` placement fallback**

In `model/src/system_config.rs`, inside the `else` branch of the
`let (dname, block) = if let Some(k) = single { … } else { … }` expression,
delete the `by_machine` binding and its comment block:

```rust
                // Placement sources, in order:
                //   1. an explicit `nodes = [..]` entry;
                //   2. the node's own `<node machine="…">`, which `model_builder`
                //      has already recorded as `execution.deploy[fqn].host`.
                //
                // (2) matters because `machine=` IS a placement — the multi-host
                // example says `machine="robot1"` and expects `[deploy.robot1]`.
                // Demanding a duplicate `nodes = [..]` for something the launch
                // file already states is redundant, and made that example
                // unresolvable (nano-ros issue 0291).
                let by_machine = execution
                    .deploy
                    .get(*fqn)
                    .and_then(|d| d.host.as_deref())
                    .and_then(|h| in_scope.iter().find(|(k, _)| k.as_str() == h))
                    .map(|(k, b)| (k.as_str(), *b));
```

and drop the `.or(by_machine)` from the `match` scrutinee below it, so it
reads:

```rust
                match in_scope
                    .iter()
                    .find(|(_, b)| b.nodes.iter().any(|n| n == fqn))
                    .map(|(k, b)| (k.as_str(), *b))
                {
```

- [ ] **Step 5: Delete the `existing_host` preservation**

Still in `model/src/system_config.rs`, delete:

```rust
            // Preserve a launch-derived host (`<node machine="…">`) — this
            // insert replaces the whole entry, and blanking it here dropped
            // the very placement that selected this block (issue 0291).
            let existing_host = execution.deploy.get(*fqn).and_then(|d| d.host.clone());
```

and remove the `host: existing_host,` line from the `Deploy { … }` literal
that follows.

- [ ] **Step 6: Delete the field**

In `model/src/lib.rs`, delete from the `Deploy` struct (lines 971-973):

```rust
    /// Host name for multi-host Linux deployments.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub host: Option<String>,
```

Then rewrite the `Deploy.target` doc comment (lines 962-968), whose
justification for `None` referenced the field just deleted. Replace it with:

```rust
    /// `linux` or `mcu:<board>` (see [`Target`]). `None` = **unplaced** —
    /// the deploy entry exists for its other facts (domain, locator, rmw,
    /// extras) but names no board placement, so the consuming entry's own
    /// board decides. A placement from the system config is always `Some`.
    ///
    /// Launch files no longer produce deploy entries at all: `<node
    /// machine=>` was ROS 1 syntax and was removed (2026-07-31). Multi-host
    /// launches partition at resolve time via a standard `<arg>` + `if=`
    /// condition, so each resolved model already holds exactly one host's
    /// nodes.
```

- [ ] **Step 7: Build and fix fallout**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/third-party/ros-launch-manifest
cargo build --all-targets 2>&1 | grep -E "^error" -A5 | head -40
```

Fix every `no field `host`` error. Expect hits in `system_config.rs` tests
that construct or assert on `Deploy`.

- [ ] **Step 8: Verify no references remain**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/third-party/ros-launch-manifest
grep -rn "\.host\b\|host:" --include="*.rs" model/src/ | grep -viE "localhost"
grep -rn "\bmachine\b" --include="*.rs" model/src/ | grep -viE "same machine|real machine|state.?machine"
```

Expected: no output from either.

- [ ] **Step 9: Test and commit**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/third-party/ros-launch-manifest
cargo test 2>&1 | tail -20
cargo clippy --all-targets --all-features -- -D warnings 2>&1 | tail -20
cargo fmt -- --check
git add -A
git commit -m "feat!: remove Deploy.host

host was only ever populated from <node machine=>, which is ROS 1 roslaunch
syntax and has been removed from the parser. Multi-host launches now
partition at resolve time via a standard <arg> + if= condition, so each
model already holds one host's nodes and needs no host field.

Drops the by_machine placement fallback with it: with multiple in-scope
[deploy.*] blocks, every node again needs an explicit \`nodes = [..]\` entry.

BREAKING CHANGE: model::Deploy no longer has a \`host\` field. Consumers
vendoring this crate (nano-ros) should stay on their pinned revision until
they migrate.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
git push -u origin remove-machine-attr
```

---

## Task 6: Remove the `machine` → `deploy.host` mapping

**Repo:** `ros-launch-resolve`

**Files:**
- Modify: `resolve/src/ros/launch_dump.rs` (lines 186-195)
- Modify: `resolve/src/ros/model_builder.rs` (lines ~455-463, ~483-485,
  ~789-800)
- Modify: `resolve/src/ros/manifest_loader.rs` (lines 2152, 2198, 2308)
- Modify: `.gitmodules` pointers for `parser` and
  `third-party/ros-launch-manifest`

**Interfaces:**
- Consumes: `NodeRecord` without `machine` (Task 4), `Deploy` without `host`
  (Task 5).
- Produces: `launch_dump::NodeRecord` without `machine`;
  `build_system_model` emits no `execution.deploy` entries from a launch
  file alone. `play_launch` (Task 7) depends on this.

- [ ] **Step 1: Create the branch and point the submodules at the new work**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve
git checkout -b remove-machine-attr
cd parser && git checkout remove-machine-attr && cd ..
cd third-party/ros-launch-manifest && git checkout remove-machine-attr && cd ../..
git add parser third-party/ros-launch-manifest
```

- [ ] **Step 2: Delete the `machine` field from the dump record**

In `resolve/src/ros/launch_dump.rs`, delete lines 186-195 — the whole doc
comment beginning `/// `<node machine="…">` — the target host` plus:

```rust
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub machine: Option<String>,
```

- [ ] **Step 3: Delete the collection and the emission in `model_builder`**

In `resolve/src/ros/model_builder.rs`:

Delete the comment block and the map declaration (lines ~455-463):

```rust
    // `<node machine="…">` → `execution.deploy[fqn].host` (nano-ros #236 /
    // Phase 46.1). Collected alongside `structure.nodes` so the key is the
    // SAME reconciled launch-dump FQN (`fqn(ns, name)`) other consumers
    // (bindings, sched) already use — populated into `execution` below,
    // once that layer exists. `target` stays `None` (UNPLACED — the model
    // names no board; the consuming entry's board decides, nano-ros #236)
    // so a later `--system` config pass can set a real placement without
    // this step guessing.
    let mut deploy_hosts: BTreeMap<String, String> = BTreeMap::new();
```

Delete the insert inside the `for n in &dump.node` loop (lines ~483-485):

```rust
        if let Some(machine) = &n.machine {
            deploy_hosts.insert(node_fqn.clone(), machine.clone());
        }
```

Delete the emission block (lines ~789-800) — the comment beginning
`// deploy: `<node machine="…">` → per-node host` plus:

```rust
    for (node_fqn, host) in deploy_hosts {
        execution.deploy.entry(node_fqn).or_default().host = Some(host);
    }
```

- [ ] **Step 4: Delete the three test-fixture initializers**

In `resolve/src/ros/manifest_loader.rs`, delete the `machine: None,` line at
each of lines 2152, 2198, and 2308.

- [ ] **Step 5: Build and fix fallout**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve
cargo build --all-targets 2>&1 | grep -E "^error" -A5 | head -40
```

If `BTreeMap` is now unused in `model_builder.rs`, remove the import.

- [ ] **Step 6: Verify no references remain**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve
grep -rn "\bmachine\b" --include="*.rs" resolve/src cli 2>/dev/null | grep -viE "same machine|real machine|state.?machine|resolving machine"
```

Expected: only `resolve/src/model.rs:158` ("an absolute path from the
resolving machine"), which is unrelated prose.

- [ ] **Step 7: Test and commit**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve
cargo test 2>&1 | tail -20
cargo clippy --all-targets --all-features -- -D warnings 2>&1 | tail -20
cargo fmt -- --check
git add -A
git commit -m "feat!: drop the machine= to deploy.host mapping

A launch file alone no longer produces execution.deploy entries. Multi-host
launches partition at resolve time via a standard <arg> + if= condition.

Bumps the parser and ros-launch-manifest submodules onto their matching
branches.

BREAKING CHANGE: launch_dump::NodeRecord no longer has a \`machine\` field.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
git push -u origin remove-machine-attr
```

---

## Task 7: Update play_launch and rewrite the multi-host fixture

**Repo:** `play_launch`

**Files:**
- Modify: `src/play_launch/src/execution/node_cmdline.rs` (lines 301-303, 368)
- Modify: `src/play_launch/src/commands/run.rs` (line 54)
- Modify: `tests/fixtures/multihost/launch/multihost.launch.xml`
- Modify: `tests/tests/resolve_multihost.rs`
- Modify: `src/ros-launch-resolve` submodule pointer

**Interfaces:**
- Consumes: `launch_dump::NodeRecord` without `machine` (Task 6).
- Produces: nothing downstream.

- [ ] **Step 1: Create the branch and bump the submodule**

```bash
cd /home/aeon/repos/play_launch
git checkout -b remove-machine-attr
cd src/ros-launch-resolve && git checkout remove-machine-attr && cd ../..
git add src/ros-launch-resolve
```

- [ ] **Step 2: Rewrite the fixture**

Replace `tests/fixtures/multihost/launch/multihost.launch.xml` entirely:

```xml
<?xml version="1.0"?>
<!--
  Multi-host launch, ROS 2 style. There is no `machine=` attribute in ROS 2
  (that is ROS 1 roslaunch); hosts are selected with an ordinary launch
  argument and `if=` conditions, so the partition happens at resolve time:

      play_launch resolve multihost.launch.xml host:=robot1 -o robot1.yaml

  `host` is not special to play_launch -- it is a launch argument like any
  other, and the name is this file's choice. A node with no `if=` runs on
  every host.
-->
<launch>
  <arg name="host" default="all"/>

  <node pkg="demo_nodes_cpp" exec="talker" name="talker1" namespace="/robot1"
        if="$(eval '&quot;$(var host)&quot; in (&quot;robot1&quot;, &quot;all&quot;)')"/>

  <node pkg="demo_nodes_cpp" exec="talker" name="talker2" namespace="/robot2"
        if="$(eval '&quot;$(var host)&quot; in (&quot;robot2&quot;, &quot;all&quot;)')"/>

  <!-- Unconditioned: runs on every host. -->
  <node pkg="demo_nodes_cpp" exec="listener" name="hub" namespace="/hub"/>
</launch>
```

- [ ] **Step 3: Rewrite the test**

Replace `tests/tests/resolve_multihost.rs` entirely:

```rust
//! Multi-host launches partition at resolve time via a standard `<arg>` +
//! `if=` condition.
//!
//! ROS 2 has no multi-machine launch — `<node machine="…">` is ROS 1
//! roslaunch syntax and `launch_xml` rejects it (`ros2/design` #255, the
//! multi-machine proposal, was closed unmerged). Selecting the host with a
//! launch argument means each resolved SystemModel already holds exactly
//! one host's nodes, so no `host` field is needed anywhere in the model.

use play_launch_tests::fixtures;
use std::{collections::BTreeSet, path::PathBuf, process::Command};

fn play_launch_bin() -> PathBuf {
    let release = fixtures::repo_root().join("target/release/play_launch");
    if release.is_file() {
        return release;
    }
    let debug = fixtures::repo_root().join("target/debug/play_launch");
    if debug.is_file() {
        return debug;
    }
    fixtures::play_launch_bin()
}

/// Resolve the fixture for one `host:=` value with one parser.
fn resolve(parser: &str, host: &str, out: &std::path::Path) -> serde_json::Value {
    let env = fixtures::install_env();
    let launch =
        fixtures::repo_root().join("tests/fixtures/multihost/launch/multihost.launch.xml");
    let mut cmd = Command::new(play_launch_bin());
    cmd.env_clear();
    cmd.envs(&env);
    cmd.args([
        "resolve",
        "--parser",
        parser,
        launch.to_str().unwrap(),
        &format!("host:={host}"),
        "-o",
        out.to_str().unwrap(),
    ]);
    let output = cmd.output().expect("run play_launch resolve");
    assert!(
        output.status.success(),
        "resolve --parser {parser} host:={host} failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let yaml = std::fs::read_to_string(out).expect("read model");
    serde_yaml_ng::from_str(&yaml).expect("parse model yaml")
}

fn node_fqns(model: &serde_json::Value) -> BTreeSet<String> {
    model["structure"]["nodes"]
        .as_object()
        .expect("structure.nodes")
        .keys()
        .cloned()
        .collect()
}

fn expected(host: &str) -> BTreeSet<String> {
    let mut set = BTreeSet::new();
    // Unconditioned — present for every host.
    set.insert("/hub/hub".to_string());
    if host == "robot1" || host == "all" {
        set.insert("/robot1/talker1".to_string());
    }
    if host == "robot2" || host == "all" {
        set.insert("/robot2/talker2".to_string());
    }
    set
}

#[test]
fn host_arg_partitions_the_launch_in_both_parsers() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");

    for parser in ["rust", "python"] {
        for host in ["robot1", "robot2", "all"] {
            let out = tmp.path().join(format!("{parser}_{host}.yaml"));
            let model = resolve(parser, host, &out);
            assert_eq!(
                node_fqns(&model),
                expected(host),
                "--parser {parser} host:={host} selected the wrong node set"
            );
        }
    }
}

#[test]
fn a_launch_only_resolve_produces_no_deploy_entries() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let out = tmp.path().join("all.yaml");
    let model = resolve("rust", "all", &out);

    // Regression guard on the `machine=` removal: nothing in a launch file
    // creates a deploy entry any more. Placement comes from a `--system`
    // config pass, never from launch syntax.
    let deploy = &model["execution"]["deploy"];
    assert!(
        deploy.is_null() || deploy.as_object().is_some_and(|d| d.is_empty()),
        "a launch-only resolve must produce no deploy entries: {deploy:?}"
    );
}

#[test]
fn the_ros1_machine_attribute_is_rejected() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let launch = tmp.path().join("ros1_machine.launch.xml");
    std::fs::write(
        &launch,
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t" machine="robot1"/>
</launch>
"#,
    )
    .expect("write fixture");

    let env = fixtures::install_env();
    let mut cmd = Command::new(play_launch_bin());
    cmd.env_clear();
    cmd.envs(&env);
    cmd.args([
        "resolve",
        launch.to_str().unwrap(),
        "-o",
        tmp.path().join("out.yaml").to_str().unwrap(),
    ]);
    let output = cmd.output().expect("run play_launch resolve");
    assert!(
        !output.status.success(),
        "machine= is ROS 1 syntax and must fail the resolve"
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("machine"),
        "the error must name the offending attribute:\n{stderr}"
    );
}
```

- [ ] **Step 4: Remove `machine` from the executor**

In `src/play_launch/src/execution/node_cmdline.rs`, delete lines 301-303:

```rust
        // Not a spawn input either way — `from_node_record` ignores it
        // (`machine: _`), it only affects placement (Phase 46.1).
        machine: None,
```

and remove `machine: _,` from the `NodeRecord { … }` destructure at line 368.

In `src/play_launch/src/commands/run.rs`, delete `machine: None,` at line 54.

- [ ] **Step 5: Build**

```bash
cd /home/aeon/repos/play_launch
just build 2>&1 | tail -30
```

Expected: clean build. Fix any remaining `machine` reference the compiler
names.

- [ ] **Step 6: Run the rewritten test**

```bash
cd /home/aeon/repos/play_launch/tests
cargo nextest run -E 'binary(resolve_multihost)' 2>&1 | tail -30
```

Expected: 3 tests pass.

- [ ] **Step 7: Full regression**

```bash
cd /home/aeon/repos/play_launch
just test-all 2>&1 | tail -40
```

Expected: all pass. This run is what validates the Task 1 allowlists against
real-world launch XML — Autoware's includes reach far more files than the
fixtures do. A failure naming an element and attribute means a table needs a
`known_unsupported` entry; go back to Task 1's table and re-verify against
ROS 2 with the probe method from Task 3 before changing it.

- [ ] **Step 8: Commit**

```bash
cd /home/aeon/repos/play_launch
git add -A
git commit -m "feat!: remove machine=; multi-host via a standard launch arg

Bumps ros-launch-resolve onto the branch that drops the machine= capture and
the deploy.host mapping, and rewrites the multihost fixture to the ROS 2
pattern: an <arg name=\"host\"> plus if= conditions, so the partition happens
at resolve time.

The fixture is now loadable by \`ros2 launch\` and by --parser python, which
it never was before.

BREAKING CHANGE: launch files using <node machine=> now fail to parse, and a
launch-only resolve no longer emits execution.deploy entries.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

## Task 8: YAML frontend strictness

**Repo:** `play_launch_parser`

**Files:**
- Modify: `crates/play_launch_parser/src/traverser/yaml.rs`
- Test: `crates/play_launch_parser/tests/attr_strictness.rs` (extend)

**Interfaces:**
- Consumes: `validate_yaml_keys(element: &str, keys: &[&str]) -> Result<()>`
  from Task 1 — the variant that accepts `spec.children`, because YAML nests
  child elements as mapping keys.
- Produces: no API. YAML launch files reject unknown mapping keys.

`traverser/yaml.rs` walks `serde_yaml_ng::Value` mappings directly and never
touches `Entity`, so neither of Task 2's hooks covers it. The dispatch lives
in `process_yaml_actions` (line 45). Each item is a single-key mapping
`{ action_type: { …attrs } }`; the loop binds `action_type: &str` (line 55)
and `action_map: Option<&Mapping>` (line 60), then checks conditions and
enters `match action_type` (line 69).

- [ ] **Step 1: Write the failing test**

Append to `crates/play_launch_parser/tests/attr_strictness.rs`:

```rust
fn parse_yaml_source(yaml: &str) -> play_launch_parser::error::Result<()> {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tmp");
    std::fs::create_dir_all(&dir).expect("create tmp dir");
    let path = dir.join(format!("attr_strict_{}.launch.yaml", std::process::id()));
    let mut fh = std::fs::File::create(&path).expect("write fixture");
    fh.write_all(yaml.as_bytes()).expect("write fixture");
    drop(fh);
    let result = play_launch_parser::parse_launch_file(&path, Default::default());
    let _ = std::fs::remove_file(&path);
    result.map(|_| ())
}

#[test]
fn yaml_parsing_rejects_an_unknown_key() {
    let err = parse_yaml_source(
        "launch:\n  - node:\n      pkg: demo_nodes_cpp\n      exec: talker\n      zzz_bogus: x\n",
    )
    .expect_err("unknown YAML node keys must fail the parse");
    let msg = err.to_string();
    assert!(
        msg.contains("Unexpected attribute(s) found in `node`") && msg.contains("'zzz_bogus'"),
        "{msg}"
    );
}

#[test]
fn yaml_parsing_rejects_machine() {
    let err = parse_yaml_source(
        "launch:\n  - node:\n      pkg: demo_nodes_cpp\n      exec: talker\n      machine: robot1\n",
    )
    .expect_err("machine= is ROS 1 syntax and must be rejected in YAML too");
    assert!(err.to_string().contains("'machine'"), "{err}");
}

#[test]
fn yaml_parsing_accepts_supported_keys() {
    parse_yaml_source(
        "launch:\n  - node:\n      pkg: demo_nodes_cpp\n      exec: talker\n      name: t\n      namespace: /n\n",
    )
    .expect("supported keys must parse");
}
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
cargo test --test attr_strictness yaml_ 2>&1 | tail -20
```

Expected: the two rejection tests FAIL — YAML currently ignores unknown keys.

- [ ] **Step 3: Implement**

In `crates/play_launch_parser/src/traverser/yaml.rs`, inside
`process_yaml_actions`, insert validation immediately after `action_map` is
bound (line 60) and **before** the `yaml_check_condition` block — same
reasoning as Task 2, ROS 2 validates regardless of conditions:

```rust
                let action_map = value.as_mapping();

                // Same allowlists the XML frontend uses. YAML nests child
                // elements as keys of this mapping (`node: { pkg: …, param:
                // [...] }`) where XML makes them separate elements, so this
                // uses the `_yaml_keys` variant, which also accepts
                // `spec.children`.
                if let Some(map) = action_map {
                    let keys: Vec<&str> = map.keys().filter_map(|k| k.as_str()).collect();
                    crate::xml::attr_spec::validate_yaml_keys(action_type, &keys)?;
                }

                // Check if/unless conditions on the action
```

- [ ] **Step 4: Run the YAML suite**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
cargo test --test attr_strictness 2>&1 | tail -20
cargo test --test yaml_tests 2>&1 | tail -30
```

Expected: both pass. The 50 existing YAML tests are the real check here — if
any now fail, the failure names the key. The fix is one of:

- the key is a legal ROS 2 **child element** (`param`, `remap`, `env`,
  `composable_node`, `arg`) → add it to that element's `children` list
- the key is a legal ROS 2 **attribute** the table missed → add it to
  `supported`, after verifying against ROS 2 with the probe in Task 3
- the key is neither → the YAML fixture is wrong

Never weaken the check to make a test green. Note also that YAML uses
underscore forms (`set_env`, `node_container`) where XML also allows hyphen
forms; `spec_for` already canonicalizes both.

- [ ] **Step 5: Quality check and commit**

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve/parser
just quality 2>&1 | tail -30
git add -A
git commit -m "feat(yaml): apply the attribute allowlists to YAML launch files

The YAML frontend walks serde_yaml mappings directly and never touches
Entity, so it needed its own hook into the same tables. ROS 2's YAML
frontend is equally strict.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
git push
```

After this commit, re-bump the submodule pointers so `ros-launch-resolve` and
`play_launch` pick up the YAML work:

```bash
cd /home/aeon/repos/play_launch/src/ros-launch-resolve
git add parser && git commit -m "chore: bump parser for YAML attribute strictness

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>" && git push
cd /home/aeon/repos/play_launch
git add src/ros-launch-resolve && git commit -m "chore: bump ros-launch-resolve for YAML attribute strictness

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
```

---

## Task 9: play_launch documentation

**Repo:** `play_launch`

**Files:**
- Create: `docs/guide/multi-host.md`
- Modify: `docs/design/unified-system-model.md` (the "Cross-track
  (nano-ros)" section, lines ~90-135)
- Modify: `docs/roadmap/phase-46-unified_system_model.md` (line 29)
- Modify: `docs/roadmap/README.md` (line 161)
- Modify: `CLAUDE.md`

**Interfaces:** none.

- [ ] **Step 1: Write the user guide**

Create `docs/guide/multi-host.md`:

````markdown
# Multi-host launches

ROS 2 has no multi-machine launch. The ROS 1 `<machine>` tag and the
`<node machine="…">` attribute do not exist in ROS 2 — `launch_ros`'s
`Node.parse()` has no such attribute, and the XML frontend rejects it:

```
ValueError: Unexpected attribute(s) found in `node`: {'machine'}
```

A multi-machine design was proposed (`ros2/design` PR #255) and closed
without merging. Under ROS 2's peer-to-peer DDS discovery there is no
central launcher to distribute processes from, so the launch file describes
one host's processes and each host runs its own.

## The pattern

Select the host with an ordinary launch argument and gate nodes on it:

```xml
<launch>
  <arg name="host" default="all"/>

  <node pkg="talker_pkg" exec="talker" name="talker"
        if="$(eval '&quot;$(var host)&quot; in (&quot;robot1&quot;, &quot;all&quot;)')"/>

  <node pkg="listener_pkg" exec="listener" name="listener"
        if="$(eval '&quot;$(var host)&quot; in (&quot;robot2&quot;, &quot;all&quot;)')"/>

  <!-- No if= — runs on every host. -->
  <node pkg="hub_pkg" exec="hub" name="hub"/>
</launch>
```

Then on each machine:

```sh
# On robot1
play_launch launch my_bringup multihost.launch.xml host:=robot1

# On robot2
play_launch launch my_bringup multihost.launch.xml host:=robot2
```

Or resolve a model per host ahead of time:

```sh
play_launch resolve multihost.launch.xml host:=robot1 -o robot1.yaml
play_launch resolve multihost.launch.xml host:=robot2 -o robot2.yaml
```

Each model holds exactly that host's nodes plus the unconditioned ones. The
partition happens at resolve time, so anything consuming the model takes it
as-is.

`host` is not special to play_launch. It is a launch argument like any
other, and the name is yours to choose — `robot`, `role`, and `machine_id`
work identically.

## Notes

- The nodes must still reach each other. That is a DDS concern, not a
  launch one: put them on the same `ROS_DOMAIN_ID` and make sure discovery
  traffic can cross the network.
- `$(eval …)` is a Python expression. The `&quot;` entities are XML
  escaping for the double quotes inside it.
- For two or three hosts the `in (…)` form above is clearest. For many
  hosts, prefer one `<group>` per host with a single condition on the group.
- This works identically under `--parser rust` and `--parser python`, and
  the launch file is loadable by stock `ros2 launch`.

## Migrating from `machine=`

`<node machine="robot1">` used to be accepted by play_launch's Rust parser
and mapped to `execution.deploy[fqn].host` in the SystemModel. It was
removed on 2026-07-31: it was never ROS 2, the Python parser always
rejected it, and the resulting launch files could not be run by `ros2
launch`. Rewrite

```xml
<node pkg="talker_pkg" exec="talker" name="talker" machine="robot1"/>
```

as

```xml
<node pkg="talker_pkg" exec="talker" name="talker"
      if="$(eval '&quot;$(var host)&quot; == &quot;robot1&quot;')"/>
```

and add `<arg name="host"/>` at the top of the file.
````

- [ ] **Step 2: Correct the design doc**

In `docs/design/unified-system-model.md`, replace the paragraph beginning
`**The one field nano-ros genuinely needs and can't get: `<node machine=>`**
(around line 107) with:

```markdown
**`<node machine=>` — REVERSED 2026-07-31.** This section previously said
`machine=` was "standard ROS 2". It is not: it is ROS 1 roslaunch syntax.
`launch_ros`'s `Node.parse()` has no `machine` attribute, `launch_xml`
rejects it outright, and ROS 2's multi-machine launch proposal
(`ros2/design` #255) was closed unmerged. Phase 46.1 wired
`machine=` → `execution.deploy[fqn].host` on that false premise; both the
attribute and the `Deploy.host` field were removed on 2026-07-31.

Multi-host launches now use a standard `<arg>` + `if=` condition and
partition at resolve time, so each resolved model holds one host's nodes and
needs no host field. See `docs/guide/multi-host.md` and
`docs/superpowers/specs/2026-07-31-machine-attr-removal-design.md`.
```

Leave the surrounding cross-track narrative intact — it records why the
field existed.

- [ ] **Step 3: Mark the roadmap entries reverted**

In `docs/roadmap/phase-46-unified_system_model.md`, change the 46.1 bullet
at line 29 to:

```markdown
- **46.1** ⛔ REVERTED 2026-07-31 — `<node machine=>` →
  `execution.deploy[fqn].host` (nano-ros #236). `machine=` is ROS 1
  roslaunch syntax, not ROS 2; both it and `Deploy.host` were removed. See
  `docs/superpowers/specs/2026-07-31-machine-attr-removal-design.md`.
```

In `docs/roadmap/README.md` line 161, replace the sentence
`Also fixed nano-ros issue #236: `<node machine=>` now flows into
`execution.deploy[fqn].host`.` with:

```markdown
(The `<node machine=>` → `execution.deploy[fqn].host` mapping added here for
nano-ros #236 was REVERTED on 2026-07-31 — `machine=` is ROS 1 syntax, not
ROS 2.)
```

- [ ] **Step 4: Update CLAUDE.md**

Add to the top of the "Key Recent Changes" list:

```markdown
- **2026-07-31**: Removed `<node machine=>` — it is ROS 1 roslaunch syntax,
  not ROS 2 (`launch_ros`'s `Node.parse()` has no such attribute;
  `launch_xml` rejects it; `ros2/design` #255 closed unmerged). The Rust
  parser accepted it while the Python parser rejected it, so the multihost
  fixture could not be loaded by `ros2 launch`. `Deploy.host` is gone from
  the model with it, and a launch-only resolve now produces no
  `execution.deploy` entries. Multi-host launches use a standard `<arg>` +
  `if=` condition and partition at resolve time
  (`play_launch resolve <launch> host:=robot1`) — see
  `docs/guide/multi-host.md`. The audit also found the Rust parser silently
  ignored **any** unknown `<node>` attribute, dropping six that ROS 2
  accepts (`exec_name`, `ros_args`, `launch-prefix`, `cwd`, `emulate_tty`,
  `shell`); it now carries per-element allowlists
  (`play_launch_parser` `src/xml/attr_spec.rs`) that error on unknown
  attributes and warn on known-unsupported ones, on both the XML and YAML
  frontends, guarded by a differential test against real ROS 2. Spec:
  `docs/superpowers/specs/2026-07-31-machine-attr-removal-design.md`.
```

Also add `multi-host.md` to the guide list in the "Documentation" section.

- [ ] **Step 5: Commit and push**

```bash
cd /home/aeon/repos/play_launch
git add -A
git commit -m "docs: multi-host guide; correct the machine= record

Adds docs/guide/multi-host.md and reverses the 'machine= is standard ROS 2'
claim in unified-system-model.md, which was the premise Phase 46.1 was built
on.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
git push -u origin remove-machine-attr
```

---

## Task 10: nano-ros issue and phase doc

**Repo:** `nano-ros` (documentation only — no code changes)

**Files:**
- Create: `docs/issues/0356-machine-attr-is-ros-1-not-ros-2.md`
- Create: `docs/roadmap/phase-325-multihost-via-launch-args.md`

**Interfaces:** none.

Their conventions: `docs/design/` holds numbered RFCs (rationale),
`docs/roadmap/` holds `phase-NNN-slug.md` work breakdowns naming the RFC
they implement, `docs/issues/` holds `NNNN-slug.md`. Highest existing issue
is 0355; highest phase is 324.

- [ ] **Step 1: Confirm the numbering is still free**

```bash
cd /home/aeon/repos/nano-ros
ls docs/issues | tail -3
ls docs/roadmap | grep -E "phase-32[0-9]" | sort
```

If 0356 or phase-325 is taken, use the next free number and adjust both
filenames and their cross-references.

- [ ] **Step 2: Create the branch**

```bash
cd /home/aeon/repos/nano-ros
git checkout -b machine-attr-is-ros1
```

- [ ] **Step 3: Write the issue**

Create `docs/issues/0356-machine-attr-is-ros-1-not-ros-2.md`:

```markdown
# 0356 — `<node machine=>` is ROS 1 syntax, not ROS 2

**Status:** Open
**Filed:** 2026-07-31
**Affects:** `nros codegen entry --host`, `nros::main!(host=…)`,
`Plan::for_host`, the four `multihost.launch.xml` example workspaces
**Upstream:** play_launch / ros-launch-resolve / ros-launch-manifest

## Summary

The multi-host partitioning added in phase-211.F is built on
`<node machine="robot1">`, which is ROS 1 roslaunch syntax. ROS 2 has no
such attribute and no multi-machine launch at all.

## Evidence

- `launch_ros`'s `Node.parse()` reads `args`, `ros_args`, `name`,
  `exec_name`, `pkg`, `exec`, `namespace`, and the `remap`/`param` children.
  There is no `machine`. The string does not appear anywhere in
  `ros2/launch_ros`.
- ROS 2's XML frontend is strict:
  `launch_xml/entity.py::assert_entity_completely_parsed()` raises on any
  attribute no action consumed. Running
  `examples/workspaces/rust/src/demo_bringup/launch/multihost.launch.xml`
  through Humble's frontend gives:

  ```
  ValueError: Unexpected attribute(s) found in `node`: {'machine'}
  ```

  All four example workspaces (`c`, `cpp`, `mixed`, `rust`) fail the same
  way. They cannot be run by `ros2 launch`.
- The ROS 2 multi-machine launch proposal (`ros2/design` PR #255, opened
  2019-09-16) was **closed without merging**. Under peer-to-peer DDS
  discovery there is no central launcher to distribute processes from.
- play_launch's Rust parser accepted `machine=` while its Python parser
  (real `launch`) rejected it — a parser-parity break that went unnoticed
  because nothing exercised the Python path on these fixtures.

## Upstream change

Removed on 2026-07-31:

- `play_launch_parser`: the `machine` capture, end to end
- `ros-launch-resolve`: `launch_dump::NodeRecord.machine` and the
  `machine=` → `execution.deploy[fqn].host` mapping in `model_builder`
- `ros-launch-manifest`: **`model::Deploy.host` is gone**, along with the
  `by_machine` placement fallback added for issue 0291

nano-ros vendors `ros-launch-manifest` and `ros-launch-resolve` under
`packages/cli/third-party/` at pinned revisions, so **the build is not
broken today**. Bumping either vendored copy before migrating will fail to
compile: `PlanNode.host` derives from `Deploy.host`.

Two consequences to plan for:

1. `Deploy.host` no longer exists. Anything reading it needs a different
   source — or, per the migration, no source at all.
2. With multiple in-scope `[deploy.*]` blocks, a node can no longer be
   placed by its `machine=`-derived host. Every node needs an explicit
   `nodes = [..]` entry, or placement errors with "is not placed". This is
   the pre-0291 behavior, restored because the fact 0291's fallback read no
   longer exists.

## Resolution

phase-325 — multi-host via launch arguments. The partition moves from a
bake-time filter (`Plan::for_host`) to a resolve-time launch argument, which
deletes a code path rather than replacing it.
```

- [ ] **Step 4: Write the phase doc**

Create `docs/roadmap/phase-325-multihost-via-launch-args.md`:

````markdown
# phase-325 — multi-host via launch arguments

**Resolves:** [issue 0356](../issues/0356-machine-attr-is-ros-1-not-ros-2.md)
**Status:** Not started

## Why

`<node machine="robot1">` is ROS 1 roslaunch syntax. ROS 2 has no
multi-machine launch and its XML frontend rejects the attribute outright, so
the four `multihost.launch.xml` fixtures cannot be run by `ros2 launch`.
Upstream removed `machine=` and `model::Deploy.host` on 2026-07-31; see
issue 0356 for the evidence.

## What replaces it

A standard `<arg>` plus `if=` conditions. The host is selected when the
launch file is *resolved*, so the resulting SystemModel already contains
exactly that host's nodes:

```xml
<launch>
  <arg name="host" default="all"/>
  <node pkg="talker_pkg" exec="talker" name="talker"
        if="$(eval '&quot;$(var host)&quot; in (&quot;robot1&quot;, &quot;all&quot;)')"/>
  <node pkg="listener_pkg" exec="listener" name="listener"
        if="$(eval '&quot;$(var host)&quot; in (&quot;robot2&quot;, &quot;all&quot;)')"/>
</launch>
```

A node with no `if=` is shared across hosts, matching the old "no `machine=`
means every host" rule.

The partition moves from bake time to resolve time, so `Plan::for_host` and
`PlanNode.host` are deleted rather than reimplemented.

## Work

1. **Fixtures.** Rewrite `multihost.launch.xml` in all four example
   workspaces (`examples/workspaces/{c,cpp,mixed,rust}/src/demo_bringup/launch/`)
   to the arg + condition form. Verify each one loads under stock
   `ros2 launch`, which none of them do today.

2. **Plan.** Delete `PlanNode.host` and `Plan::for_host` from
   `packages/cli/nros-cli-core/src/codegen/entry/mod.rs` (`for_host` at
   line 118, `host` field at line 198), plus the
   `plan_for_host_partitions_by_machine` unit test at line 693.

3. **CLI and macro.** `nros codegen entry --host <id>`
   (`packages/cli/nros-cli-core/src/cmd/codegen.rs:131-134`) and
   `nros::main!(launch=…, host="…")`
   (`packages/core/nros-macros/src/main_macro.rs:81-86`) become generic
   launch-argument passing. `host:=<id>` is then an ordinary launch
   argument with no special handling, which also unblocks any other
   conditional a workspace wants to drive from the bake.

   Decide during implementation whether to keep `--host` as sugar for
   `--launch-arg host:=<id>` or to remove it. Upstream's position is that no
   argument name should get special treatment; sugar that expands to a
   launch argument does not violate that, a code path keyed on the name
   does.

4. **system.toml.** `[deploy.robot1]` / `[deploy.robot2]` blocks in the
   example workspaces stop doubling as placement selectors. Each needs an
   explicit `nodes = [..]` list, or the blocks are removed if the per-host
   models make them redundant. See issue 0356 for why the `by_machine`
   fallback went away.

5. **Tests.** `packages/testing/nros-tests/tests/multihost_partition_bake.rs`
   asserts "the deploy-target id == the launch `machine=` id"
   (`multihost_deploy_targets_match_baked_hosts`, line 99) — that premise
   dissolves. Rewrite it to assert that resolving with `host:=robot1`
   produces a model containing only robot1's nodes.
   `multihost_e2e.rs` and the `Workload::Multihost` matrix entry
   (`packages/testing/nros-tests/src/matrix.rs:264`) need the new invocation
   form.

6. **Vendored crates.** Bump `packages/cli/third-party/ros-launch-manifest`
   and `packages/cli/third-party/ros-launch-resolve` (both copies) only
   after steps 2-5 land. Bumping earlier fails to compile.

## Net effect

One code path disappears. Multi-host becomes an ordinary launch-argument
conditional that stock ROS 2 tooling understands, and the launch files
become runnable by `ros2 launch` for the first time.
````

- [ ] **Step 5: Verify the cross-references resolve**

```bash
cd /home/aeon/repos/nano-ros
test -f docs/issues/0356-machine-attr-is-ros-1-not-ros-2.md && echo "issue OK"
test -f docs/roadmap/phase-325-multihost-via-launch-args.md && echo "phase OK"
grep -n "0356" docs/roadmap/phase-325-multihost-via-launch-args.md
grep -n "phase-325" docs/issues/0356-machine-attr-is-ros-1-not-ros-2.md
```

Expected: both files exist and reference each other.

- [ ] **Step 6: Verify the line references in the phase doc are accurate**

```bash
cd /home/aeon/repos/nano-ros
sed -n '116,120p;196,200p;690,695p' packages/cli/nros-cli-core/src/codegen/entry/mod.rs
sed -n '129,136p' packages/cli/nros-cli-core/src/cmd/codegen.rs
sed -n '79,88p' packages/core/nros-macros/src/main_macro.rs
sed -n '95,115p' packages/testing/nros-tests/tests/multihost_partition_bake.rs
```

Expected: each range shows the construct the phase doc names. Correct any
that have drifted.

- [ ] **Step 7: Commit and push**

```bash
cd /home/aeon/repos/nano-ros
git add docs/issues/0356-machine-attr-is-ros-1-not-ros-2.md \
        docs/roadmap/phase-325-multihost-via-launch-args.md
git commit -m "docs: file 0356 and phase-325 -- machine= is ROS 1, not ROS 2

<node machine=> is roslaunch syntax. launch_ros has no such attribute,
launch_xml rejects it, and ros2/design#255 (multi-machine launch) closed
unmerged, so all four multihost.launch.xml fixtures fail under ros2 launch.

Upstream removed machine= and model::Deploy.host on 2026-07-31. Our vendored
copies are pinned, so nothing breaks until we bump. phase-325 migrates the
partition from a bake-time Plan::for_host filter to a resolve-time launch
argument.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"
git push -u origin machine-attr-is-ros1
```

---

## Task 11: Final verification across all repos

**Files:** none — verification only.

- [ ] **Step 1: Confirm no `machine` references survive**

```bash
cd /home/aeon/repos/play_launch
grep -rn "\bmachine\b" --include="*.rs" --include="*.xml" --include="*.yaml" \
     src/play_launch tests/ src/ros-launch-resolve/resolve \
     src/ros-launch-resolve/parser/crates \
     src/ros-launch-resolve/third-party/ros-launch-manifest/model 2>/dev/null \
  | grep -viE "same machine|real machine|state.?machine|machine-readable|resolving machine|/target/"
```

Expected: no output. Any hit outside those exclusions is a missed site.

- [ ] **Step 2: Confirm `Deploy.host` is gone**

```bash
cd /home/aeon/repos/play_launch
grep -rn "\.host\b" --include="*.rs" \
     src/ros-launch-resolve/third-party/ros-launch-manifest/model/src \
     src/ros-launch-resolve/resolve/src 2>/dev/null | grep -v localhost
```

Expected: no output.

- [ ] **Step 3: Confirm the fixture is now ROS 2-loadable**

```bash
cd /home/aeon/repos/play_launch
bash -c 'source /opt/ros/humble/setup.bash; python3 -c "
from launch.frontend import Parser
root, parser = Parser.load(\"tests/fixtures/multihost/launch/multihost.launch.xml\")
parser.parse_description(root)
print(\"fixture loads under stock ros2 launch\")
"'
```

Expected: `fixture loads under stock ros2 launch`. Before this work the same
command raised
`ValueError: Unexpected attribute(s) found in `node`: {'machine'}`.

- [ ] **Step 4: Full test run**

```bash
cd /home/aeon/repos/play_launch
just test-all 2>&1 | tail -40
```

Expected: all pass.

- [ ] **Step 5: Autoware parity**

```bash
cd /home/aeon/repos/play_launch/tests/fixtures/autoware
just compare-dumps 2>&1 | tail -20
```

Expected: models match between parsers. This is the strongest signal that
the strictness tables did not break real-world launch files.

- [ ] **Step 6: Report the branches**

Collect and report:

```bash
for repo in \
  /home/aeon/repos/play_launch \
  /home/aeon/repos/play_launch/src/ros-launch-resolve \
  /home/aeon/repos/play_launch/src/ros-launch-resolve/parser \
  /home/aeon/repos/play_launch/src/ros-launch-resolve/third-party/ros-launch-manifest \
  /home/aeon/repos/nano-ros; do
  echo "=== $repo"
  git -C "$repo" branch --show-current
  git -C "$repo" log --oneline origin/main..HEAD 2>/dev/null | head -10
done
```

Report the branch name and commit list per repo. **Do not merge anything.**

---

## Notes for the implementer

**If Task 2 or Task 7 surfaces failing fixtures**, the failure names the
element and attribute. Resist the urge to add the attribute to `supported`
to make the test green. Verify against ROS 2 first, with the same method
that produced the tables:

```bash
bash -c 'source /opt/ros/humble/setup.bash; python3 -c "
from launch.frontend import Parser
import tempfile, os
src = \"\"\"<launch>
  <node pkg=\"demo_nodes_cpp\" exec=\"talker\" name=\"t\" THE_ATTR=\"x\"/>
</launch>
\"\"\"
with tempfile.NamedTemporaryFile(\"w\", suffix=\".launch.xml\", delete=False) as fh:
    fh.write(src); p = fh.name
try:
    root, parser = Parser.load(p); parser.parse_description(root); print(\"ROS 2 ACCEPTS\")
except Exception as e: print(\"ROS 2:\", type(e).__name__, e)
finally: os.unlink(p)
"'
```

If ROS 2 accepts it and this parser reads it → `supported`. If ROS 2 accepts
it and this parser ignores it → `known_unsupported`. If ROS 2 rejects it →
the fixture is wrong.

**The one deliberate divergence** is `<group namespace=>` / `<group ns=>`:
ROS 2 rejects them, this parser reads them, and they are in
`known_unsupported` so existing launch files keep working with a portability
warning. Task 3's differential test needs an explicit allow-list entry for
this pair.
