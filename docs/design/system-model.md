# SystemModel — the resolved system artifact

Status: Draft (2026-07). Producer-side design; the consumer-side sibling is
nano-ros RFC-0050 (`docs/design/0050-system-model.md` in the nano-ros repo).

## Summary

Factor play_launch's front-end into a `resolve` step that emits a
**SystemModel**: one fully-resolved, checked, YAML artifact describing one
concrete system variant — node graph + contracts + execution/deployment.
play_launch's own runtime consumes the model instead of re-parsing launch at
spawn time; the nano-ros build system consumes the same model to bake each
MCU node's configuration into its image.

```
launch tree (XML/py) ──┐
contract manifests ────┼→ play_launch resolve: parse → bind args → filter
system config ─────────┘   conditions → merge scopes → check (14 rules)
                                     │
                          SystemModel  (system_model.yaml)
                        ┌────────────┴─────────────┐
              play_launch runtime          nano-ros build system
              (spawn/supervise Linux       (bake per-image slices for
               nodes; interception          MCU nodes: tiers, QoS,
               monitors read contracts)     budgets, wiring)
```

## Why

1. **One canonical input.** Today the launch parser, the manifest checker,
   record/replay, and the web UI each hold their own view of "the system".
   A single derived artifact gives them one input and makes record/replay
   and UI state trivially consistent.
2. **Cross-machine contracts.** nano-ros (embedded ROS 2 client) adopts the
   same manifest contract format; an E2E budget spanning an MCU sensor node
   and a Linux pipeline only means something if both sides read the same
   numbers from the same artifact.
3. **Reviewability.** The model is emitted only when the checker passes
   (Error severity refuses; warnings embed). What was reviewed is
   byte-identical to what runs.

## Decisions (agreed with the nano-ros side)

| Decision | Choice |
|---|---|
| Name | `SystemModel`; file `system_model.yaml` |
| Types home | `ros-launch-manifest` repo, new `model` crate beside `types`/`check` — both projects vendor that repo already |
| Binding | **early** — one model per concrete arg-set; no variables or conditions inside the artifact |
| Serialization | YAML, serde; schema versioned independently of the manifest format version |
| Checker gate | refuse on Error severity; embed warnings under `diagnostics:` |
| Deployment | integrator-owned — placement (`linux` vs `mcu:<board>`) comes from the integrator's system config, never from component manifests |

### Early binding

`resolve(args) -> SystemModel`. On Linux the resolver is present at run time
(this binary), so launch-time arg overrides are just resolve-executed-late;
the artifact needs no internal condition evaluator, and no consumer
re-implements substitution. The embedded consumer is single-variant by
physics anyway. Z3 satisfiability over the whole arg space stays a
manifest-level (pre-resolution) check.

Consequence: resolve must be fast and **cache-keyed** — key = hash(arg
binding + all input file contents). Repeated launches with unchanged inputs
reuse the cached model.

## Model layout (three layers)

```yaml
meta:
  version: 1                    # SystemModel schema version
  args: { mode: velodyne }      # the binding this model was resolved from
  inputs:                       # content hashes of every input file
    - { path: perception.yaml, sha256: ... }
  resolver: { tool: play_launch, version: ... }
  diagnostics:                  # embedded checker warnings
    - "warning[wiring]: ..."

structure:                      # layer 1 — from the launch tree
  scopes: ...                   # tree kept for diagnostics + budget attribution
  nodes:                        # resolved instances: FQN, pkg, exec/plugin
    /perception/detector: { pkg: ..., exec: ..., scope: ... }
  topics:                       # merged, FQN, typed
    /sensing/pointcloud: { type: sensor_msgs/msg/PointCloud2, pub: [...], sub: [...] }
  services: ...
  actions: ...

contracts:                      # layer 2 — from the manifests, post-merge
  endpoints: ...                # min/max_rate_hz, max_age_ms, jitter_ms, state, required
  node_paths: ...               # max_latency_ms, correlation, tolerance_ms
  scope_paths: ...              # E2E max_latency_ms, max_drop_rate, max_consecutive
  topics: ...                   # rate_hz, max_transport_ms, drops, qos

execution:                      # layer 3 — from the integrator's system config
  deploy:
    /perception/detector: { target: linux, host: main }
    /sensing/imu_node:    { target: "mcu:stm32f4" }
  tiers: ...                    # per-tier spin period + platform priorities
  bindings: ...                 # node/callback-group -> tier
```

Producers own layers: launch tree → structure, manifests → contracts,
system config → execution. Consumers slice: play_launch runtime takes the
`linux` deploy subset; nano-ros build takes `mcu:*`.

## Work items (proposal)

**Scheduling SSoT (2026-07-18 decision):** the execution layer will carry
the *resolved* sched plan — mapper identity, resolved chains, per-path
ranks — not just flat `tiers`/`bindings`, so every consumer (runtime apply,
`--explain`, analysis, monitoring, off-host nano-ros) reads scheduling from
the model instead of re-deriving it. Design of record:
[system-model-sched-ssot.md](system-model-sched-ssot.md); work items:
[phase-45](../roadmap/phase-45-sched_ssot_unification.md). That document
supersedes the vocabulary-v2 coordination note below, which stays as the
detailed type/translation reference (exact `types`/`sched`-crate structs to
share): [system-model-vocab-v2-embedding.md](system-model-vocab-v2-embedding.md).

1. `ros-launch-manifest`: add `model` crate — `SystemModel` types, serde,
   schema doc, golden-file round-trip tests. No behavior, pure schema.
2. play_launch: extract the existing parse→bind→merge→check pipeline into a
   library `resolve()` returning `SystemModel`; add the
   `play_launch resolve --args ... --out system_model.yaml` verb (cache
   layer keyed on input hashes).
3. play_launch runtime: consume a `SystemModel` (in-memory from resolve, or
   `--model` file) for spawn + supervision; interception monitors read
   layer-2 contracts from the model instead of the manifest loader.
4. Record format / web UI: point both at the model as their system view
   (follow-up; record format already overlaps heavily with layer 1).

## nano-ros R1 asks (canonical-path decision, 2026-07-17)

nano-ros adopted the SystemModel as its CANONICAL config path (its own
launch/system.toml bake retires at parity — nano-ros RFC-0052 §Canonical
path, phase-296 §Retirement). The parity gate needs, on this side:

Full inventory: nano-ros RFC-0052 §Parity gap analysis (status table).
Dependency-ordered asks on this side:

**Model schema (ros-launch-manifest `model` crate):**
1. `Deploy { domain, locator, rmw }` + a `Deploy.extra` open map
   (consumer-defined keys — nano-ros build tuning rides it so the end
   state never parses system.toml directly).
2. `execution.transports` — the largest gap: per-deploy network/session
   identity (ip/mac/gateway/interfaces, wifi ssid/psk, serial/can
   device+baud, per-transport rmw/locator/domain). Folds multi-domain
   routing; the embedded boot bake cannot exist without it.
3. `execution.bridges` (in-binary topic relays) + `execution.features`
   (capability axes).
4. `structure.nodes[].params` — RESOLVED parameter values. ROS params
   are system semantics, not spawn info (the two-artifact split excluded
   cmd/env/params-FILES); the embedded target has no record.json to read
   params from.
5. Endpoint contracts (`PubContract`/`SubContract`) gain optional `qos`
   (per-endpoint QoS already exists manifest-side).
6. Per-node `lifecycle_autostart` (`none|configure|active`).

**resolve:**
7. Read the integrator's `system.toml` as the system-config input
   (deploy/tiers/transports/bridges/features → execution layer).
8. Merge manifest `actions:` into the index — `structure.actions` is
   always empty today (the loader never merges actions).
9. Per-target resolve ergonomics (one-model-all-targets vs per-target
   `--target` resolves; TierDef already carries all platforms — decide
   + document).

## Parser fidelity (cross-runtime consistency)

The model is only "canonical" if the resolver's launch parser agrees with
every consumer's parser on the RESOLVED node graph — same FQNs, same
params. A divergence there silently changes node identity or topic names
between the Linux runtime and an embedded image built from the same model.

- **`resolve` from a launch file** uses the Rust `play_launch_parser`
  (`parse_launch_file`), not the Python ROS 2 dumper — the dumper (with
  `expanded_node_namespace`) is only on the `--record` path. So
  `play_launch_parser` must reproduce ROS 2 launch semantics faithfully.
- **`<group ns="…">` fix (2026-07):** the parser had dropped the `ns`
  attribute on `<group>` (treating only `<push-ros-namespace>` as
  namespace-setting), so a node under `<group ns="alpha">` resolved as
  `/talker` instead of `/alpha/talker`. This diverged from nano-ros's
  `nros-launch-parser` (which honors `<group ns=>` per its RFC-0024
  subset) and from ROS 2 launch_xml (which translates the attribute into a
  leading push-ros-namespace). Fixed: `GroupAction` parses `ns`/
  `namespace` and the entity traverser pushes it for the group body.
  General rule: when a consumer parser and `play_launch_parser` disagree
  on a launch construct, the resolved model is wrong for that construct —
  fix the parser, add a regression test, and note it here.

## Non-goals

- Variables or conditions inside the model (early binding).
- Automatic placement decisions (integrator-owned).
- Replacing manifests as the authoring surface — the model is derived,
  never hand-edited.
- Dynamic mode switching within one model — resolve another model and
  restart the affected subtree.
