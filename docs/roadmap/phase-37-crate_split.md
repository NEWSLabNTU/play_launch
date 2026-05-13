# Phase 37 — Crate Split + Boundary Cleanup

Refactor `play_launch` from a 19k-LOC monolith into 6 focused crates
with sharp interfaces between executor, checker, enforcement, and
parser. Driven by the boundary review in
[`docs/design/architecture-review.md`](../design/architecture-review.md).

No behavioural change. Every step keeps the binary working and the
test suite green; no flag day.

## Goals

1. Single source of truth for each cross-crate type:
   - `InterceptionEvent` / `EventKind` (today: mirrored in producer + consumer)
   - `record.json` schema (today: mirrored in parser + executor)
2. Sharpen the parser / executor / checker / enforcement boundaries:
   each becomes a standalone crate with explicit deps.
3. Per-subcommand CLI argument groups; remove the 30-field
   `LaunchArgs` god-struct.
4. Faster incremental builds — touching the web UI shouldn't recompile
   the rule engine.

## Non-goals

- Not changing the public CLI surface.
- Not changing `record.json` format on disk.
- Not changing the SPSC event byte layout.
- Not rewriting the manifest static checker (already well-factored
  in `ros-launch-manifest`).
- Not breaking the parser submodule's standalone publishability.

## Target crate layout

```
play_launch_record_format         ← record.json types only
play_launch_interception_types    ← EventKind, InterceptionEvent, chunk codec
play_launch_manifest_index        ← ManifestIndex + graph (replaces play_launch::ros::manifest_*)
play_launch_enforcement           ← RuleEngine (replaces play_launch::runtime_enforcement)
play_launch_executor              ← execution + member_actor + monitoring + process
play_launch_web                   ← web + diagnostics
play_launch                       ← cli/, commands/, main.rs (thin glue)
```

Dep graph (read top-to-bottom):

```
                play_launch_record_format
                          │
       ┌──────────────────┼──────────────────┐
       ▼                  ▼                  ▼
  play_launch_      play_launch_     play_launch_
  parser            executor         manifest_index
  (submodule)             │                  │
                          │                  ▼
                          │          play_launch_
                          │          enforcement
                          │                  │
       play_launch_       │                  │
       interception_      └──────┬───────────┘
       types                     ▼
            │             play_launch_web
            ▼                    │
       (play_launch_             ▼
        interception              play_launch
        cdylib uses               (CLI glue)
        same crate)
```

## Sub-phases

### 37.1 `play_launch_interception_types` crate

**Scope.** Extract `EventKind`, `InterceptionEvent`,
`TopicNameDeclared` / `TypeNameDeclared` chunk codec, and the
24-byte payload constants into a new tiny crate. Both
`play_launch_interception` (.so producer) and
`play_launch::interception` (consumer) depend on it. The
hand-mirrored copy in `src/play_launch/src/interception/mod.rs:25-58`
deletes; events come from the shared crate.

**Files touched.**
- New: `src/play_launch_interception_types/{Cargo.toml,src/lib.rs}`.
- Edit: `src/play_launch_interception/src/event.rs` becomes a re-export.
- Edit: `src/play_launch/src/interception/mod.rs` drops the mirror.
- Edit: both Cargo.toml files add the new dep.

**Risk.** Low. ~200 LOC mechanical move. Byte layout fixed by
`#[repr(C)]` + `const _: () = assert!(size_of::<…>() == 40)`.

**Acceptance.** Existing interception unit tests + 5 runtime
enforcement smoke tests pass. Autoware end-to-end produces zero
violations as before.

### 37.2 `play_launch_record_format` crate

**Scope.** Extract `record.json` types (ScopeEntry, ScopeTable,
NodeRecord, ContainerRecord, LoadNodeRecord, etc.) from the parser
submodule's `record/types.rs` and the executor-side
`play_launch::ros::launch_dump`. Single crate, both sides depend on
it. Parser writes; executor reads.

**Files touched.**
- New: `src/play_launch_record_format/{Cargo.toml,src/lib.rs}`.
- Edit (submodule): `src/play_launch_parser/.../record/types.rs`
  becomes a thin re-export. Parser Cargo.toml adds the dep.
- Edit: `src/play_launch/src/ros/launch_dump.rs` deletes its
  duplicate; uses the new crate.
- Coordinated commit + push across both repos.

**Risk.** Medium. Touches the parser submodule, so coordinated push
needed. Schema serialization must be byte-identical (serde derive
should preserve this; verify with a round-trip test against an
existing record.json from the autoware fixture).

**Acceptance.** `record.json` files written before the refactor
deserialize identically after. Parser unit tests (~370) pass.
Executor `launch` and `replay` work on existing record.json
files.

### 37.3 Trait-ify `ManifestIndex` in enforcement

**Scope.** Introduce `trait ManifestIndexLike` in
`runtime_enforcement` that names just the methods the rule engine
calls (`topic_lookup`, `external_lookup`, `manifests_for_node`,
etc.). Rule engine accepts `&dyn ManifestIndexLike`. Implements the
trait for `ros_launch_manifest_types::ManifestIndex`.

**Files touched.**
- New small file: `src/play_launch/src/runtime_enforcement/index_trait.rs`.
- Edit: `RuleEngine` constructor signature.

**Risk.** Very low. ~50 LOC, additive.

**Acceptance.** Existing tests pass. Add at least one new unit test
that exercises `RuleEngine` with a hand-rolled mock impl (no
manifest loader needed).

### 37.4 Per-subcommand CLI arg structs

**Scope.** Split `cli::options::LaunchArgs` into composable groups
via clap's `#[clap(flatten)]`:

- `InterceptionArgs` (— `--enforce-rules`, `--block-unauthorized-…`)
- `MonitoringArgs` (`--disable-monitoring`, `--enable-cpu-profiling`)
- `WebArgs` (`--web-addr`, `--disable-web-ui`)
- `ContainerArgs` (`--container-mode`)
- `ManifestArgs` (`--manifest-dir`)
- `ConfigArgs` (`--config`)
- `LaunchSpecificArgs` (the launch-only knobs)

Subcommands compose what they need. `replay` doesn't carry
launch-time-only fields.

**Risk.** Low. Mechanical clap refactor.

**Acceptance.** Argument parsing produces the same effective state.
Help text grouping improves; no regressions in CLI test fixtures.

### 37.5 `play_launch_manifest_index` + `play_launch_enforcement`

**Scope.** Two new crates.

- `play_launch_manifest_index`:
  - moves `play_launch::ros::manifest_loader`
  - moves `play_launch::ros::manifest_graph`
  - depends on `play_launch_record_format`, `ros-launch-manifest-types`
- `play_launch_enforcement`:
  - moves `play_launch::runtime_enforcement::*`
  - depends on `play_launch_manifest_index`,
    `play_launch_interception_types`, optionally `rclrs`
    (feature-gated for lifecycle subscriber)
  - tokio integration kept behind a thin wrapper module so the core
    `RuleEngine::observe()` stays sync + deterministic

**Files touched.**
- New: 2 crates, total ~3k LOC moved.
- Edit: `play_launch` re-exports paths for back-compat (just for the
  transition); old `play_launch::runtime_enforcement::RuleEngine`
  becomes a `pub use play_launch_enforcement::RuleEngine`.

**Risk.** Medium. Needs careful disentangling of imports. Smoke
tests must keep passing.

**Acceptance.** `play_launch_enforcement` tests pass standalone
(`cargo test -p play_launch_enforcement`). End-to-end smoke
unchanged.

### 37.6 `play_launch_executor` crate

**Scope.** Largest move. Extracts `execution/`, `member_actor/`,
`monitoring/`, `process/` from `play_launch`. The executor's
public API is a single `Executor::run(record, config) -> Result<()>`.

`play_launch::commands::launch` collapses to a glue function:

```rust
let record = play_launch_parser::parse_launch_file(...)?;
let config = build_config(args);
play_launch_executor::Executor::new(record, config).run().await
```

**Risk.** Medium-high. ~8k LOC move. Lots of internal crossings
between member_actor and monitoring (process stats, csv writer,
io_helper IPC). Plan to keep `ipc/` co-located with executor since
it's executor-internal.

**Acceptance.** All integration tests pass (parser unit + scope +
integration + runtime_enforcement smoke). Autoware end-to-end
produces zero violations.

### 37.7 `play_launch_web` crate

**Scope.** Moves `web/` + `diagnostics/`. Depends on
`play_launch_executor` (member handles, snapshot APIs) and
`play_launch_enforcement` (live violation stream).

**Risk.** Low after 37.5 + 37.6 land. Web layer already talks to
executor through narrow handle interfaces (SSE + REST).

**Acceptance.** Web UI works as before. Browser smoke (launch + open
http://127.0.0.1:8080) shows nodes, topics, violations.

## Order

```
37.1 (interception_types)  ─┐
37.2 (record_format)        ├─→ 37.3 (trait) ─→ 37.4 (CLI) ─→ 37.5 (manifest+enforce) ─→ 37.6 (executor) ─→ 37.7 (web)
                            │
                            └─ both pre-reqs for 37.6
```

37.1 and 37.2 are independent and can be done in either order.
37.3 and 37.4 are independent of each other after 37.1; can be
parallel. 37.5 needs 37.1, 37.2, 37.3. 37.6 needs 37.2, 37.5.
37.7 needs 37.6.

## Status

| Sub-phase | Status | Notes |
|----------:|--------|-------|
| 37.1 | not started | lowest-risk; recommended starting point |
| 37.2 | not started | coordinated parser-submodule push |
| 37.3 | not started | enables enforcement unit tests |
| 37.4 | not started | mechanical clap refactor |
| 37.5 | not started | requires 37.1–37.3 |
| 37.6 | not started | largest move, requires 37.2 + 37.5 |
| 37.7 | not started | requires 37.6 |

## Success criteria (Phase 37 overall)

After all 7 sub-phases:

- [ ] `wc -l src/play_launch/src/**/*.rs` drops to ~3k (from ~19k).
- [ ] `cargo build -p play_launch_enforcement` succeeds standalone.
- [ ] `cargo build -p play_launch_executor` succeeds standalone.
- [ ] No type duplication grep hits:
  - [ ] `EventKind` defined in exactly one crate.
  - [ ] `ScopeEntry` defined in exactly one crate.
- [ ] All existing tests pass: 120 unit + 28 interception + 5 smoke +
  371 parser + 42 integration + 42 IR.
- [ ] Autoware end-to-end under `--enforce-rules=warn` produces zero
  violations.

## Out of scope

- Splitting `python/` bridge into its own crate (low value; small).
- Splitting `ipc/` from executor (it's executor-internal).
- Rewriting `commands/*` to be pure thin glue (will happen
  naturally as side-effect of 37.6).

## Risks

- **Parser submodule coordination (37.2).** Push order matters: push
  parser changes first, then play_launch submodule-pointer update.
  Otherwise main repo CI fetches a parser SHA not yet on the parser
  repo's main.
- **API drift during 37.5 + 37.6.** While the play_launch crate's
  internal modules move into new crates, types they expose need
  stable names. Use a `play_launch::compat` re-export module during
  the transition; remove after one release cycle.
- **Rclrs feature flags.** `rclrs` brings async lifecycle subscriber
  into `runtime_enforcement`. When extracted, the new
  `play_launch_enforcement` crate should feature-gate this so the
  core engine compiles in environments without ROS deps (useful for
  fuzz + property tests).

## Comparison with prior factoring

- Phase 31 (manifest crate split into types + check) — same idea,
  different layer. Successful prior art for "split a feature into
  its own crate".
- Phase 22 (Launch IR feature-gated) — uses `--features ir` for
  optional types in the parser. Pattern transferable to
  `rclrs`-dependent enforcement code.
