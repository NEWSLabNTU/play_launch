# Phase 22: Launch Tree IR (Intermediate Representation)

**Status**: In Progress
**Priority**: High (foundation for static analysis, QoS annotation, multi-path reasoning)
**Dependencies**: Phase 13 (Rust parser), Phase 14 (Python execution)

## Overview

Convert the Rust parser from an interpreter (eagerly resolves substitutions, prunes conditional branches, flattens includes) into a compiler with an intermediate representation that preserves the full program structure.

### Motivation

The current parser (`parse_launch_file()`) produces a flat `record.json` for one specific set of inputs. This loses:

- **Conditional branches not taken** — if `use_sim=false`, the simulation nodes are silently dropped with no trace
- **Substitution expressions** — `$(var robot_name)_driver` becomes `"turtlebot_driver"`, losing the expression
- **Include hierarchy** — all nodes are flattened into a single list
- **Variable dependencies** — no way to ask "what changes if I set `a=5`?"
- **Source provenance** — no record of which file/line defined each node

The IR preserves all of this, enabling:

1. **Evaluation** with concrete inputs → `record.json` (backward compatible, replaces current parser)
2. **Static analysis** → all possible nodes, variable dependency graphs, reachability analysis
3. **Annotation** → topic QoS, node relationships, parameter schemas

### Design Informed By

The official ROS 2 `launch` package (`external/launch/`) uses a similar structure: `Action` base class with optional `Condition`, `LaunchDescription` as a container, `Substitution` types as lazy expressions. Our IR mirrors this but in Rust with stronger typing.

## IR Type Design

### Core Principle: Separate Parsing From Evaluation

```
                           ┌──────────────────┐
  XML/Python/YAML ────────►│  Parser (Front)   │──────► LaunchProgram (IR)
                           └──────────────────┘              │
                                                             │
                    ┌────────────────────────────────────────┤
                    │                                        │
                    ▼                                        ▼
           ┌───────────────┐                      ┌──────────────────┐
           │  Evaluator    │                      │  Static Analyzer │
           │  (inputs →    │                      │  (all-paths      │
           │   record.json)│                      │   analysis)      │
           └───────────────┘                      └──────────────────┘
```

### Key Design Decisions

1. **Reuse existing `Substitution` enum** — our `Vec<Substitution>` already models the expression AST. Wrap in a newtype `Expr` to distinguish unevaluated expressions from resolved strings.

2. **Condition on every action** — matches the official launch `Action.condition: Optional[Condition]`. XML `if=`/`unless=` attributes are stored, not evaluated during IR construction.

3. **Include = sub-program** — `ActionKind::Include` contains `body: Option<Box<LaunchProgram>>`, preserving the tree structure. Flattening happens only during evaluation.

4. **Include arg values as `Expr`** — parsed into substitution AST during IR construction (not raw strings). The evaluator resolves them in order (later args can reference earlier ones). Static analysis can inspect the substitution structure without re-parsing.

5. **Align `ContainerAction` with `NodeAction`** — change `ContainerAction` and `ComposableNodeAction` fields from eagerly-resolved `String` to deferred `Vec<Substitution>`, matching `NodeAction`'s pattern.

6. **Python files deferred to Phase 22.3** — Python launch files use mock execution via PyO3. Phase 22.0-22.2 handles XML/YAML. Python will use annotated execution (capture conditions encountered) in a later sub-phase.

### Type Definitions

New module: `src/play_launch_parser/src/play_launch_parser/src/ir.rs`

```rust
/// A lazy string expression (unevaluated substitution chain).
/// Wraps Vec<Substitution>. Evaluate with the IR evaluator.
pub struct Expr(pub Vec<Substitution>);

/// Source location for an IR node.
pub struct Span { pub file: PathBuf, pub line: usize }

/// Condition gating an action's execution.
pub enum Condition {
    If(Expr),       // execute when truthy
    Unless(Expr),   // execute when falsy
}

/// A single action with optional condition and provenance.
pub struct Action {
    pub kind: ActionKind,
    pub condition: Option<Condition>,
    pub span: Option<Span>,
}

/// All launch action types.
pub enum ActionKind {
    // Arguments & Variables
    DeclareArgument { name, default: Option<Expr>, description, choices },
    SetVariable { name, value: Expr },

    // Environment
    SetEnv { name: Expr, value: Expr },
    UnsetEnv { name: Expr },

    // Namespace
    PushNamespace { namespace: Expr },

    // Global Parameters & Remappings
    SetParameter { name: Expr, value: Expr },
    SetRemap { from: Expr, to: Expr },

    // Grouping
    Group { namespace: Option<Expr>, body: Vec<Action> },

    // Include (with resolved sub-program)
    Include { file: Expr, args: Vec<IncludeArg>, body: Option<Box<LaunchProgram>> },

    // Nodes
    SpawnNode { package: Expr, executable: Expr, name, namespace, params, ... },
    SpawnExecutable { cmd: Expr, name, args, env },
    SpawnContainer { package: Expr, executable: Expr, name: Expr, ..., nodes: Vec<ComposableNodeDecl> },
    LoadComposableNode { target: Expr, nodes: Vec<ComposableNodeDecl> },
}

/// A complete launch program (one file's worth of actions).
pub struct LaunchProgram { pub source: PathBuf, pub body: Vec<Action> }

/// Supporting types
pub struct IncludeArg { pub name: String, pub value: Expr }
pub struct ParamDecl { pub name: String, pub value: Expr }
pub struct RemapDecl { pub from: Expr, pub to: Expr }
pub struct EnvDecl { pub name: Expr, pub value: Expr }
pub struct ComposableNodeDecl { package, plugin, name, namespace, params, ..., condition, span }
```

### Relationship to Existing Types

| Existing type                    | IR equivalent                | Change                                                                                 |
|----------------------------------|------------------------------|----------------------------------------------------------------------------------------|
| `Vec<Substitution>`              | `Expr`                       | Newtype wrapper                                                                        |
| `NodeAction`                     | `ActionKind::SpawnNode`      | Fields already `Vec<Substitution>`                                                     |
| `ContainerAction`                | `ActionKind::SpawnContainer` | **Fields changed** from `String` to `Vec<Substitution>`                                |
| `ComposableNodeAction`           | `ComposableNodeDecl`         | **Fields changed** from `String` to `Vec<Substitution>`                                |
| `IncludeAction`                  | `ActionKind::Include`        | **Args changed** from `(String, String)` to `(String, Vec<Substitution>)`; adds `body` |
| `should_process_entity()` → bool | `Action.condition`           | Stored, not evaluated                                                                  |
| (none)                           | `Span`                       | New: source provenance                                                                 |

### New Public API

```rust
/// Parse a launch file into its IR without evaluating.
pub fn analyze_launch_file(path: &Path) -> Result<LaunchProgram>;

/// Parse and evaluate (existing, unchanged).
pub fn parse_launch_file(path: &Path, cli_args: HashMap<String, String>) -> Result<RecordJson>;

impl LaunchProgram {
    pub fn evaluate(&self, inputs: HashMap<String, String>) -> Result<RecordJson>;
    pub fn arguments(&self) -> Vec<&DeclareArgument>;
    pub fn all_nodes(&self) -> Vec<&ActionKind>;
}
```

## Implementation Order

```
22.0 IR type definitions + module                     complete
22.1 Align action parse helpers (Container, Include)  complete
22.2 XML IR builder (analyze_launch_file)             complete
22.3 IR evaluator (evaluate_launch_file)              complete
22.4 Python launch file IR support                    planned
22.5 Static analysis passes                           future
```

---

## Phase 22.0: IR Type Definitions

**Status**: Complete

Define all IR types in a new `ir` module. No functional changes — compilation-only validation.

### Work Items

- [x] Create `src/play_launch_parser/src/play_launch_parser/src/ir.rs`
- [x] Define `Expr`, `Span`, `Condition`, `Action`, `ActionKind` (14 variants), `LaunchProgram`
- [x] Define supporting types: `IncludeArg`, `ParamDecl`, `RemapDecl`, `EnvDecl`, `ComposableNodeDecl`
- [x] Add `pub mod ir` to `lib.rs`
- [x] Implement `Expr` helper methods (`literal()`, `is_literal()`, `as_literal()`)
- [x] Add `From<Vec<Substitution>> for Expr` impl

### Files

| File | Change |
|---|---|
| `src/.../ir.rs` | **New** — 209 lines |
| `src/.../lib.rs` | Add `pub mod ir` |

### Verification

- [x] `cargo build` compiles
- [x] `just test` — all existing tests pass (no behavior change)

---

## Phase 22.1: Align Action Parse Helpers

**Status**: Complete

Change `ContainerAction`, `ComposableNodeAction`, and `IncludeAction` to store fields as `Vec<Substitution>` (deferred resolution), matching `NodeAction`'s existing pattern.

### Work Items

- [x] `ContainerAction`: change `name`, `package`, `executable`, `namespace` from `String` to `Vec<Substitution>`
- [x] Update `ContainerAction::from_entity()` to call `parse_substitutions()` but NOT `resolve_substitutions()`
- [x] Update `ContainerAction::to_container_record()` and `to_node_record()` to resolve at use site
- [x] `ComposableNodeAction`: change `package`, `plugin`, `name`, `namespace` to `Vec<Substitution>`
- [x] Update `ComposableNodeAction::from_entity()` accordingly
- [x] Update `to_load_node_record()` to resolve at use site
- [x] `IncludeAction`: change `args` values from `String` to `Vec<Substitution>`
- [x] Update `process_include()` to resolve arg values at use site
- [x] Add `From` impls: 11 impls converting all action types to IR `ActionKind`

### Files

| File                                      | Change                                              |
|-------------------------------------------|-----------------------------------------------------|
| `src/.../actions/container.rs`            | Fields → `Vec<Substitution>`, resolve at use site   |
| `src/.../actions/load_composable_node.rs` | `ComposableNodeAction` fields → `Vec<Substitution>` |
| `src/.../actions/include.rs`              | Arg values → `Vec<Substitution>`                    |
| `src/.../actions/mod.rs`                  | Add 11 `From` impls for `ActionKind`                |
| `src/.../traverser/entity.rs`             | Update callers to resolve before use                |
| `src/.../traverser/include.rs`            | Update include arg resolution                       |

### Verification

- [x] `just test` — all 326 parser tests pass
- [x] `just test-all` — all 356 tests pass (326 parser + 30 integration)
- [x] Zero clippy warnings

---

## Phase 22.2: XML IR Builder

**Status**: Complete

Add `build_ir_entity()` to the traverser that constructs `LaunchProgram` from XML, preserving all conditional branches and unevaluated expressions. Add `analyze_launch_file()` public API.

### Work Items

- [x] Add `build_ir_entity()` method on `LaunchTraverser` (parallel to `traverse_entity()`)
  - Records `if=`/`unless=` as `Condition` instead of evaluating
  - Groups collect children into `body: Vec<Action>`
  - Includes recursively call `build_ir_file()` and set `body`
  - Extracts `Span` from XML entity position via `XmlEntity::line_number()`
- [x] Add `build_ir_file()` dispatching by extension (`.xml`, `.yaml`)
- [x] Python/YAML files: emit `ActionKind::OpaqueFunction` placeholder (to be filled in 22.4)
- [x] Add `analyze_launch_file(path)` and `analyze_launch_file_with_args(path, args)` to `lib.rs`
- [ ] Add CLI subcommand: `play_launch_parser analyze <file>` (deferred — not critical for library use)

### Implementation Notes

- IR builder in `traverser/ir_builder.rs` (separate from `entity.rs` for clarity)
- `build_ir_entity()` still applies `<arg>`, `<let>`, `<declare_argument>` to context so include file paths can resolve
- `build_ir_include()` returns `Option<Box<LaunchProgram>>` — gracefully degrades to `None` if file path cannot be resolved
- `extract_condition()` and `make_span()` helper functions for condition/provenance extraction

### Tests (15 tests in `tests/ir_tests.rs`)

- [x] `test_ir_simple_node`: one node → `SpawnNode` with correct fields and span
- [x] `test_ir_conditional_branches`: `if=`/`unless=` → both branches preserved with `Condition`
- [x] `test_ir_include_tree`: nested include → `Include.body` contains parsed sub-program
- [x] `test_ir_include_with_args`: include with `<arg>` children → `IncludeArg` values preserved
- [x] `test_ir_group_scoping`: `<group ns="...">` → `Group` with children in `body`
- [x] `test_ir_variable_expressions`: `$(var name)_suffix` → `Expr` contains `LaunchConfiguration`
- [x] `test_ir_let_and_arg`: `<arg>` → `DeclareArgument`, `<let>` → `SetVariable`
- [x] `test_ir_container_with_composable_nodes`: container → `SpawnContainer` with `nodes` list
- [x] `test_ir_set_env_and_unset_env`, `test_ir_set_parameter`, `test_ir_set_remap`
- [x] `test_ir_push_namespace`, `test_ir_executable`, `test_ir_load_composable_node`
- [x] `test_ir_span_line_numbers`: second node has greater line number than first

### Files

| File                              | Change                                       |
|-----------------------------------|----------------------------------------------|
| `src/.../traverser/ir_builder.rs` | **New** — `build_ir_entity/file/include()`   |
| `src/.../traverser/mod.rs`        | Add `mod ir_builder`                         |
| `src/.../xml/entity.rs`           | Add `XmlEntity::line_number()`               |
| `src/.../lib.rs`                  | Add `analyze_launch_file[_with_args]()`      |
| `src/.../tests/ir_tests.rs`       | **New** — 15 IR construction tests           |

### Verification

- [x] `just test` — all 326 parser tests + 15 new IR tests pass
- [x] `just test-all` — all 356 tests pass (326 parser + 30 integration)
- [x] Zero clippy warnings
- [x] Backward compat: `parse_launch_file()` unchanged, all existing tests pass

---

## Phase 22.3: IR Evaluator

**Status**: Complete

Walks the IR tree, resolves expressions against a `LaunchContext`, follows conditional branches, and produces `RecordJson` — the same output as `parse_launch_file()`. Validates round-trip correctness for pure-XML launch files.

### Architecture

```
evaluate_launch_file(path, args) -> RecordJson
  ├── analyze_launch_file_with_args(path, args) → LaunchProgram
  └── LaunchTraverser::new(args)
      ├── evaluate_ir(&program)
      │   └── evaluate_action(action)
      └── into_record_json()
```

The evaluator is a set of methods on `LaunchTraverser`, reusing existing infrastructure:
- `CommandGenerator::generate_node_record()` / `generate_executable_record()` for node records
- `ContainerAction::to_container_record()` / `to_load_node_records()` for container records
- `into_record_json()` for final assembly with global param backfill
- IR→Action conversion helpers unwrap `Expr` back to `Vec<Substitution>` for record-producing actions

### Work Items

- [x] Implement `Expr::resolve(&self, context: &LaunchContext) -> Result<String>`
- [x] Make `is_truthy()` `pub(crate)` in `condition.rs`
- [x] Add IR→Action conversion helpers (`ir_to_node_action`, `ir_to_executable_action`, etc.)
- [x] Implement `evaluate_ir()` and `evaluate_action()` on `LaunchTraverser`
  - Evaluate `Condition` → skip action if false
  - Resolve `Expr` → `String` using `Expr::resolve()` with context
  - Handle scope: `Group` pushes/pops context, `Include` creates child traverser
  - Accumulate `NodeRecord`, `ComposableNodeContainerRecord`, `LoadNodeRecord`
- [x] Add `evaluate_launch_file()` public API in `lib.rs`
- [x] Add query methods on `LaunchProgram`: `arguments()`, `all_nodes()`
- [x] Python/YAML includes: skipped with debug log (body is `None` from IR builder)

### Tests (18 tests in `tests/ir_eval_tests.rs`)

- [x] `test_evaluate_simple_node`: single node → correct RecordJson
- [x] `test_evaluate_conditional`: `if=true` node present, `unless=true` node skipped
- [x] `test_evaluate_conditional_override`: CLI args override default → opposite branch taken
- [x] `test_evaluate_group_namespace`: group namespace applied to child nodes
- [x] `test_evaluate_include`: include with args → included nodes with overridden arg value
- [x] `test_evaluate_container`: container → 1 container record + 2 load_node records
- [x] `test_evaluate_set_env`: env vars propagated to node records
- [x] `test_evaluate_set_parameter`: global params in node records (normalized to `True`)
- [x] `test_evaluate_unset_env`: set then unset → variable absent from output
- [x] `test_evaluate_push_namespace`: `push-ros-namespace` in group → correct namespace
- [x] `test_evaluate_nested_groups`: nested groups → combined namespace
- [x] `test_evaluate_executable`: executable with args → correct cmd vector
- [x] `test_evaluate_round_trip`: multi-feature XML → field-by-field equality with `parse_launch_file()`
- [x] `test_evaluate_round_trip_with_args`: CLI arg override → same output both paths
- [x] `test_evaluate_round_trip_with_group_namespace`: group namespaces → same output
- [x] `test_evaluate_round_trip_include`: include with args → same output
- [x] `test_evaluate_launch_program_arguments`: `arguments()` returns declared arg names
- [x] `test_evaluate_launch_program_all_nodes`: `all_nodes()` returns spawn actions recursively

### Files

| File | Change |
|---|---|
| `src/.../ir.rs` | Add `Expr::resolve()`, `LaunchProgram::arguments()`, `all_nodes()` |
| `src/.../condition.rs` | Make `is_truthy()` `pub(crate)` |
| `src/.../traverser/ir_evaluator.rs` | **New** — evaluator + IR→Action conversion helpers |
| `src/.../traverser/mod.rs` | Add `mod ir_evaluator` |
| `src/.../lib.rs` | Add `evaluate_launch_file()` |
| `tests/ir_eval_tests.rs` | **New** — 18 evaluator tests |

### Verification

- [x] `cargo test --all` — 344 tests pass (233 unit + 18 edge + 3 perf + 18 IR eval + 15 IR build + 36 python + 21 xml)
- [x] `just test` — all 30 integration tests pass
- [x] Zero clippy warnings
- [x] Code formatted
- [x] No changes to existing `traverse_entity()`, `into_record_json()`, `CommandGenerator`, or action types

---

## Phase 22.4: Python Launch File IR Support

**Status**: Future

Use annotated execution to capture IR from Python launch files. Run the Python file via PyO3, but record condition expressions and action structure rather than eagerly resolving.

### Approach

- Mock `IfCondition`/`UnlessCondition` record their predicate expression before evaluating
- Mock `Node`, `GroupAction`, etc. emit `Action` IR nodes alongside existing captures
- Python actions that cannot be statically analyzed (e.g., `OpaqueFunction` with arbitrary Python logic) are represented as `ActionKind::OpaqueFunction { description: String }`
- Converts Python captures to IR actions with conditions preserved

---

## Phase 22.5: Static Analysis Passes

**Status**: Future

Build analysis passes over the IR.

### Planned Passes

- **Variable dependency graph**: track which variables reference which others
- **Reachability analysis**: given input constraints (e.g., `use_sim ∈ {true, false}`), enumerate which nodes can possibly be spawned
- **Topic QoS annotation**: query ROS 2 node descriptions for publisher/subscriber QoS settings
- **Parameter schema extraction**: collect all parameters per node with types and defaults
- **Diff analysis**: compare two sets of inputs → show which nodes/params change

---

## Backward Compatibility

The existing `parse_launch_file()` function signature and behavior are **unchanged** throughout all sub-phases. The existing integration tests (`tests/tests/autoware.rs`) serve as the backward compatibility gate:

| Test                               | What it validates                                                     |
|------------------------------------|-----------------------------------------------------------------------|
| `test_autoware_dump_rust`          | Entity counts match expected (46 nodes, 15 containers, 54 load_nodes) |
| `test_autoware_dump_python`        | Python parser still works                                             |
| `test_autoware_dump_counts_match`  | Rust and Python produce same counts                                   |
| `test_autoware_parser_parity`      | Rust and Python `record.json` diff passes                             |
| `test_autoware_process_count_rust` | Launched processes match dump count                                   |
| `test_autoware_smoke_test`         | Full launch + 15s settle + health analysis passes                     |

Every sub-phase must pass all of these before proceeding.
