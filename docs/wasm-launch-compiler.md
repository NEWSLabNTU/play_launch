# WASM Launch Compiler

**Status**: Design
**Author**: Phase 23 design exploration
**Date**: 2026-02-22

## Vision

Compile all ROS 2 launch file formats (XML, YAML, Python) into portable WebAssembly modules. The WASM modules execute in a sandboxed runtime that enforces launch-only semantics, producing execution plans (`record.json`) without requiring a ROS 2 installation, Python interpreter, or the original launch file source.

```
XML  ──┐                                    ┌─── record.json
YAML ──┼──► IR (LaunchProgram) ──► .wasm ──►│
Python ┘                                    └─── (portable, sandboxed)
```

## Motivation

| Problem | WASM solution |
|---|---|
| Parsing requires Python + ROS 2 packages installed | WASM module is self-contained |
| `OpaqueFunction` can run arbitrary code | Sandbox restricts to launch API only |
| Launch files are not portable across platforms | WASM runs on any architecture |
| No way to analyze Python launch files statically | AST compiler extracts structure |
| Re-parsing is slow for large launch trees | Compiled WASM is fast to re-execute |
| Security: launch files can access filesystem/network | WASM has no ambient authority |

## Architecture

### Compilation Pipeline

```
                    ┌──────────────┐
  XML launch file ──► XML Parser   ├──┐
                    └──────────────┘  │
                    ┌──────────────┐  │    ┌─────────┐    ┌──────────┐
 YAML launch file ──► YAML Parser  ├──┼───►│   IR    ├───►│  WASM    │
                    └──────────────┘  │    │(Launch  │    │ Module   │
                    ┌──────────────┐  │    │Program) │    │ (.wasm)  │
 Python launch    ──► AST Compiler ├──┘    └─────────┘    └────┬─────┘
                    └──────────────┘                           │
                                                               ▼
                                                    ┌──────────────────┐
                                                    │   WASM Runtime   │
                                                    │   (wasmtime)     │
                                                    │                  │
                                                    │  Host imports:   │
                                                    │   spawn_node()   │
                                                    │   resolve_var()  │
                                                    │   find_pkg()     │
                                                    │   push_ns()      │
                                                    │   ...            │
                                                    └────────┬─────────┘
                                                             │
                                                             ▼
                                                    ┌──────────────────┐
                                                    │  Execution Plan  │
                                                    │  (record.json)   │
                                                    └──────────────────┘
```

### Include Resolution

Launch files include other launch files. In the compiled model:

```
outer.launch.xml
  <include file="$(find-pkg-share my_pkg)/launch/inner.launch.xml" />
```

Two strategies:

**A. Pre-linked (single module)**: The compiler resolves all includes at compile time and emits a single flat WASM module. Simpler runtime, but requires all included files available at compile time.

**B. Multi-module (lazy linking)**: Each launch file compiles to its own WASM module. Includes become host calls: `include_module("my_pkg", "launch/inner.wasm")`. The runtime loads and instantiates the included module on demand.

**Recommendation**: Start with **A** (pre-linked). The IR builder already resolves includes and attaches the body. The WASM codegen walks the full IR tree and emits a single module. Move to B later if incremental recompilation becomes important.

## IR Layer (Existing)

Phase 22.0-22.3 built the IR types and evaluator. The WASM codegen consumes the same `LaunchProgram`:

```rust
pub struct LaunchProgram {
    pub source: PathBuf,
    pub body: Vec<Action>,
}

pub struct Action {
    pub kind: ActionKind,
    pub condition: Option<Condition>,
    pub span: Option<Span>,
}

pub enum ActionKind {
    DeclareArgument { name, default, description, choices },
    SetVariable { name, value: Expr },
    SetEnv { name, value: Expr },
    UnsetEnv { name },
    PushNamespace { namespace: Expr },
    SetParameter { name, value: Expr },
    SetRemap { from: Expr, to: Expr },
    Group { namespace, body: Vec<Action> },
    Include { file: Expr, args, body: Option<Box<LaunchProgram>> },
    SpawnNode { package, executable, name, namespace, params, ... },
    SpawnExecutable { cmd, name, args, env },
    SpawnContainer { package, executable, name, ..., nodes },
    LoadComposableNode { target, nodes },
    OpaqueFunction { description },
}
```

`Expr` wraps `Vec<Substitution>` — the unevaluated expression AST. The WASM codegen emits code that calls host functions to resolve substitutions at runtime.

## WASM Module Design

### Host Import Surface

The WASM module imports a set of host functions that define what a launch program can do. This is the **sandbox boundary** — anything not imported is inaccessible.

```
Module: "launch"

  ;; --- Context ---
  declare_arg(name, default, description)    → void
  set_var(name, value)                       → void
  resolve_var(name)                          → string
  set_env(name, value)                       → void
  unset_env(name)                            → void
  push_namespace(ns)                         → void
  pop_namespace()                            → void
  set_global_param(name, value)              → void
  set_remap(from, to)                        → void
  save_scope()                               → scope_id
  restore_scope(scope_id)                    → void

  ;; --- Package Resolution ---
  find_package_share(pkg)                    → string
  resolve_exec_path(pkg, exec)               → string

  ;; --- Substitutions ---
  eval_env_var(name, default)                → string
  eval_command(cmd)                          → string
  eval_python_expr(expr)                     → string
  eval_condition(expr)                       → bool

  ;; --- Record Production ---
  spawn_node(pkg, exec, name, ns, ...)       → void
  spawn_executable(cmd, name, ...)           → void
  begin_container(pkg, exec, name, ns, ...)  → container_id
  load_composable_node(container_id, ...)    → void
  end_container(container_id)                → void

  ;; --- String Operations ---
  concat(a, b)                               → string
  str_equals(a, b)                           → bool
  is_truthy(value)                           → bool
```

Strings are passed via WASM linear memory. The host and guest share a simple string protocol: write to memory, pass (offset, length) pairs.

### String Passing Convention

WASM core only has numeric types. Strings use a bump allocator in linear memory:

```wasm
(memory (export "memory") 1)  ;; 1 page = 64KB, grows as needed
(global $bump (mut i32) (i32.const 0))

;; write_str: copy string literal into memory, return (offset, len)
;; Host functions accept (i32, i32) pairs for strings
```

The host reads strings from guest memory. The guest writes string literals during initialization and receives host string results via a return-string protocol (host writes to guest memory, returns offset+len).

### Module Structure

```wasm
(module
  ;; Type declarations
  (type $void_ss (func (param i32 i32 i32 i32)))          ;; (str, str) → void
  (type $str_s (func (param i32 i32) (result i32 i32)))   ;; str → str
  (type $bool_s (func (param i32 i32) (result i32)))       ;; str → bool
  ;; ...

  ;; Host imports
  (import "launch" "declare_arg" (func $declare_arg ...))
  (import "launch" "resolve_var" (func $resolve_var ...))
  (import "launch" "spawn_node" (func $spawn_node ...))
  ;; ...

  ;; Linear memory for strings
  (memory (export "memory") 1)

  ;; Data segment: all string literals from the launch file
  (data (i32.const 0) "demo_nodes_cpp\00talker\00my_talker\00use_sim\00true\00...")

  ;; The compiled launch program
  (func (export "plan") (param $args_ptr i32) (param $args_len i32)
    ;; declare_arg("use_sim", "true")
    call $declare_arg  ;; with string offsets for "use_sim", "true"

    ;; if is_truthy(resolve_var("use_sim"))
    call $resolve_var  ;; resolve "use_sim"
    call $is_truthy
    if
      ;; spawn_node("sim_pkg", "sim_node", ...)
      call $spawn_node
    end

    ;; spawn_node("demo_nodes_cpp", "talker", ...)
    call $spawn_node
  )
)
```

### Expr Compilation

Each `Expr` (substitution chain) compiles to a sequence of WASM instructions that build a string:

| Substitution | WASM codegen |
|---|---|
| `Text("hello")` | Load string literal offset+len from data segment |
| `LaunchConfiguration("name")` | `call $resolve_var` with "name" |
| `EnvironmentVariable { name, default }` | `call $eval_env_var` with name, default |
| `FindPackageShare("pkg")` | `call $find_package_share` with "pkg" |
| `Command("echo test")` | `call $eval_command` with "echo test" |
| `PythonExpression("1+1")` | `call $eval_python_expr` with "1+1" |
| `Text("a") + LaunchConfig("x") + Text("_b")` | Resolve each part, `call $concat` to join |

### Condition Compilation

```rust
Condition::If(expr)     →  compile_expr(expr); call $is_truthy; if ... end
Condition::Unless(expr) →  compile_expr(expr); call $is_truthy; i32.eqz; if ... end
```

### Group/Scope Compilation

```rust
Group { namespace, body } →
    call $save_scope       ;; returns scope_id
    local.set $scope
    ;; if namespace: compile_expr(ns); call $push_namespace
    ;; compile each action in body
    local.get $scope
    call $restore_scope
```

## WASM Runtime Design

The host side is a Rust program using `wasmtime`:

```rust
struct LaunchHost {
    context: LaunchContext,
    records: Vec<NodeRecord>,
    containers: Vec<ComposableNodeContainerRecord>,
    load_nodes: Vec<LoadNodeRecord>,
}

fn execute_launch_wasm(wasm_bytes: &[u8], args: HashMap<String, String>) -> Result<RecordJson> {
    let engine = Engine::default();
    let module = Module::new(&engine, wasm_bytes)?;

    let host = LaunchHost::new(args);
    let mut store = Store::new(&engine, host);
    let mut linker = Linker::new(&engine);

    // Register all host functions
    linker.func_wrap("launch", "declare_arg",
        |mut caller: Caller<LaunchHost>, name_ptr, name_len, default_ptr, default_len| {
            let name = read_guest_string(&caller, name_ptr, name_len);
            let default = read_guest_string(&caller, default_ptr, default_len);
            caller.data_mut().context.declare_arg(name, default);
        })?;

    linker.func_wrap("launch", "spawn_node",
        |mut caller: Caller<LaunchHost>, /* params */| {
            // Build NodeRecord from params, push to caller.data_mut().records
        })?;

    // ... register all imports ...

    let instance = linker.instantiate(&mut store, &module)?;
    let plan = instance.get_typed_func::<(i32, i32), ()>(&mut store, "plan")?;
    plan.call(&mut store, (args_ptr, args_len))?;

    Ok(store.into_data().into_record_json())
}
```

### Sandbox Guarantees

| Property | Mechanism |
|---|---|
| No filesystem access | No WASI imports, no `fd_*` functions |
| No network access | No socket imports |
| No process spawning | `eval_command` is the only subprocess escape, host controls |
| No ambient authority | All capabilities are explicit host imports |
| Deterministic (mostly) | `eval_command` and `eval_env_var` are the only non-deterministic imports |
| Memory bounded | WASM linear memory has configurable limits |
| CPU bounded | `wasmtime` fuel metering prevents infinite loops |

### `eval_command` Policy

The `$(command ...)` substitution in ROS 2 runs arbitrary shell commands. The host can enforce a policy:

- **Strict mode**: Reject `eval_command` entirely. Launch files using `$(command ...)` fail at compile time.
- **Allowlist mode**: Only permit specific commands (e.g., `hostname`, `rospack find`).
- **Passthrough mode**: Execute commands on the host (breaks sandboxing but matches current behavior).

Default: **allowlist mode** with common ROS 2 commands permitted.

## Python AST Compiler

### Strategy

Parse the Python source with `ast.parse()`, pattern-match on the ROS 2 launch API, and compile to IR. This runs at compile time only — no Python interpreter needed at plan execution time.

```
Python source  →  ast.parse()  →  AST pattern matching  →  IR (LaunchProgram)
```

The compiler runs in Python (not Rust) because it needs `ast.parse()`. It outputs a JSON-serialized IR that the Rust codegen consumes. Alternatively, it outputs WASM directly.

### Supported Patterns

**Tier 1: Direct declarative** (covers ~70% of launch files)

```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arg', default_value='val'),
        Node(package='pkg', executable='exec',
             condition=IfCondition(LaunchConfiguration('arg'))),
        GroupAction([...]),
        IncludeLaunchDescription(...),
    ])
```

The compiler recognizes:
- `LaunchDescription([...])` → `LaunchProgram { body: [...] }`
- `DeclareLaunchArgument(name, default_value=v)` → `ActionKind::DeclareArgument`
- `Node(package=p, executable=e, ...)` → `ActionKind::SpawnNode`
- `ComposableNodeContainer(...)` → `ActionKind::SpawnContainer`
- `GroupAction([...])` → `ActionKind::Group`
- `IncludeLaunchDescription(source, launch_arguments=[...])` → `ActionKind::Include`
- `SetEnvironmentVariable(name, value)` → `ActionKind::SetEnv`
- `SetParameter(name, value)` → `ActionKind::SetParameter`
- `PushRosNamespace(ns)` → `ActionKind::PushNamespace`
- `IfCondition(expr)` → `Condition::If(compile_substitution(expr))`
- `UnlessCondition(expr)` → `Condition::Unless(compile_substitution(expr))`

Substitution compilation:
- `LaunchConfiguration('name')` → `Substitution::LaunchConfiguration("name")`
- `FindPackageShare('pkg')` → `Substitution::FindPackageShare("pkg")`
- `EnvironmentVariable('VAR')` → `Substitution::EnvironmentVariable("VAR")`
- `PathJoinSubstitution([a, b])` → concatenation of compiled parts with `/`
- `PythonExpression('expr')` → `Substitution::PythonExpression("expr")`
- String literal `'text'` → `Substitution::Text("text")`
- List of substitutions `[a, b, c]` → concatenation

**Tier 2: Simple OpaqueFunction** (covers ~25% more)

```python
def configure(context):
    if context.launch_configurations['key'] == 'value':
        return [Node(...)]
    else:
        return [Node(...)]
```

The compiler analyzes the function body AST:
- `context.launch_configurations[key]` or `.get(key, default)` → `Substitution::LaunchConfiguration`
- `if expr == value:` → `Condition::If(Equals(expr, value))`
- `if expr:` → `Condition::If(expr)`
- Each branch body is compiled as a `Group` with the appropriate condition
- Variable assignment (`x = LaunchConfiguration('name')`) is tracked through simple SSA

**Tier 3: Loops and comprehensions** (covers ~4% more)

```python
nodes = [Node(package=p, executable='node') for p in ['pkg_a', 'pkg_b']]
```

The compiler handles:
- List comprehensions with literal iterables → unroll to flat list
- Simple `for` loops with literal `range()` or literal list → unroll
- String formatting with known substitutions

**Rejected patterns** (the remaining ~1%)

```python
# External I/O
result = subprocess.check_output(...)

# Dynamic imports
module = importlib.import_module(name)

# Generators, async, class definitions
async def setup(): ...

# Complex control flow
while condition: ...
```

These produce a clear compile error:
```
error: unsupported Python pattern at line 15
  --> my_launch.py:15:5
   |
15 |     result = subprocess.check_output(['rospack', 'find', pkg])
   |     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   = help: WASM compilation requires deterministic launch files
   = help: replace with FindPackageShare('pkg') substitution
```

### AST Compiler Implementation

The compiler is a Python module (`play_launch_compiler/`) that:

1. Parses the source file with `ast.parse(source, filename)`
2. Finds the `generate_launch_description` function definition
3. Walks the AST, building an IR tree:
   - `ast.Call` nodes are pattern-matched by function name
   - `ast.If` nodes become conditional groups
   - `ast.Return` nodes with list values become action sequences
   - Variable bindings (`ast.Assign`) are tracked in a symbol table
4. Outputs the IR as JSON (consumed by the Rust WASM codegen)

```python
# Pseudocode for the core compiler loop
def compile_call(node: ast.Call) -> Action:
    func_name = get_call_name(node)
    match func_name:
        case "Node":
            return compile_node(node)
        case "DeclareLaunchArgument":
            return compile_declare_arg(node)
        case "GroupAction":
            return compile_group(node)
        case "IncludeLaunchDescription":
            return compile_include(node)
        case "OpaqueFunction":
            return compile_opaque_function(node)
        case _:
            raise CompileError(f"unsupported action: {func_name}", node)
```

### Fallback: Instrumented Execution

For the ~1% of patterns that can't be statically compiled, the AST compiler can optionally fall back to instrumented execution:

1. Detect that the pattern is uncompilable
2. Execute the Python file with enhanced mocks that record the visit tree (not just captures)
3. The mocked execution produces IR with conditions preserved as `Expr` values
4. Merge the execution-derived IR into the compiled IR

This fallback requires a Python runtime at **compile time** but not at **plan execution time**.

## Roadmap

### Phase 23.0: XML/YAML to IR (existing)

**Status**: Complete (Phase 22.0-22.3)

The IR builder and evaluator already handle XML launch files. YAML launch files currently produce `OpaqueFunction` placeholders — extend the YAML parser to produce proper IR actions.

### Phase 23.1: IR to WASM Codegen

Implement `compile_to_wasm(program: &LaunchProgram) -> Vec<u8>`.

- Define the host import surface (function signatures)
- Implement string literal collection and data segment emission
- Implement `Expr` compilation (substitution chains → host calls + concat)
- Implement `Condition` compilation (if/else blocks)
- Implement action compilation (each `ActionKind` → host call sequence)
- Implement scope management (save/restore for groups and includes)
- Emit valid WASM module bytes via `wasm-encoder`

**Crate**: `wasm-encoder` (Bytecode Alliance, `wasm-tools` monorepo)

### Phase 23.2: WASM Runtime

Implement the host side that executes compiled WASM modules.

- Define `LaunchHost` struct holding `LaunchContext` + record accumulators
- Implement all host import functions (`spawn_node`, `resolve_var`, etc.)
- Implement string passing (read/write guest linear memory)
- Implement `execute_launch_wasm(bytes, args) -> RecordJson`
- Add fuel metering for CPU bounding

**Crate**: `wasmtime` (Bytecode Alliance)

### Phase 23.3: Record Dump via WASM

Wire the compilation and execution into the existing `play_launch` CLI.

```
play_launch dump --wasm <pkg> <launch_file>    # compile + execute, dump record.json
play_launch compile <pkg> <launch_file> -o f.wasm  # compile only
play_launch exec f.wasm --args key:=val        # execute pre-compiled
```

### Phase 23.4: XML/YAML PoC Testing

End-to-end validation:

- `parse_launch_file(path, args)` == `compile_then_execute(path, args)` for all XML test fixtures
- Round-trip tests: simple nodes, conditions, groups, includes, containers, env, params
- Performance comparison: direct evaluation vs WASM execution
- Autoware XML subset (the XML-only portion of the launch tree)

### Phase 23.5: Python AST Compiler

Implement the Python AST → IR compiler.

- Tier 1: Declarative patterns (LaunchDescription, Node, GroupAction, etc.)
- Tier 2: OpaqueFunction body analysis (if/else on LaunchConfiguration)
- Tier 3: Loop unrolling (list comprehensions, simple for loops)
- Error reporting for unsupported patterns
- Output: JSON-serialized IR (or direct WASM emission)

### Phase 23.6: Python Integration

Integrate the Python compiler into the full pipeline.

- Python launch files compile to WASM alongside XML/YAML
- Include resolution across file types (XML includes Python, Python includes XML)
- Handle the fallback path for uncompilable patterns

### Phase 23.7: Autoware Smoke Tests

Full Autoware validation:

- Compile the entire Autoware planning_simulator launch tree to WASM
- Execute the WASM module with Autoware's default arguments
- Verify: 46 nodes, 15 containers, 54 composable nodes (matches existing test)
- Parser parity: WASM output == `parse_launch_file()` output
- Performance: measure compilation time + execution time vs direct parsing

## Tooling

| Crate | Purpose | Version (2026-02) |
|---|---|---|
| `wasm-encoder` | Emit WASM binary from Rust | 0.245.x |
| `wasmtime` | Execute WASM with host functions | 41.x |
| `wasmprinter` | WASM binary → WAT text (debugging) | 0.245.x |
| `wat` | WAT text → WASM binary (testing) | 0.245.x |

All from the Bytecode Alliance `wasm-tools` monorepo. Core WASM modules (not Component Model) — simpler, more stable, sufficient for our use case.

## Constraints and Non-Goals

**Constraints on launch files**:
- No arbitrary filesystem access from within the launch program
- No network access
- No dynamic code loading
- `$(command ...)` substitutions are subject to host policy (allowlist)
- Python launch files must use the supported ROS 2 launch API subset
- Loops must have statically determinable iteration counts (for unrolling)

**Non-goals for initial implementation**:
- Browser execution (server-side only for now)
- Component Model / WIT interfaces (core modules suffice)
- Hot reloading of WASM modules
- WASM-to-WASM module linking for includes (pre-link instead)
- Support for event handlers (`OnProcessStart`, `OnProcessExit`, etc.)
- Support for `TimerAction` (deferred actions)

## Open Questions

1. **String passing**: Bump allocator in linear memory vs multi-value returns vs shared memory. Bump allocator is simplest but wastes memory. Evaluate after prototyping.

2. **Include granularity**: Pre-linked (single module) is simpler but means recompiling everything when one file changes. Acceptable for v1?

3. **Python compiler location**: Python module (needs `ast.parse()`) vs Rust with `rustpython-parser`. The former is simpler; the latter avoids Python dependency at compile time.

4. **`PythonExpression` substitution**: `$(eval 1+2*3)` requires a Python expression evaluator. Options: embed a minimal expression evaluator in the WASM runtime, or reject `PythonExpression` in strict mode.

5. **Versioning**: Should compiled WASM modules embed a version number for the host import surface? Yes — breaking changes to imports would silently produce wrong results.
