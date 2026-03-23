# Launch Manifest: Prior Art and Related Tools

**Status**: Research
**Date**: 2026-03-23

---

## Overview

This document surveys tools and projects that share conceptual ground with our
launch manifest design: declarative specifications of expected communication
graphs, interface contracts, or system topology that can be audited against
runtime. Each section covers core concepts, relevance to our design, and
patterns worth adopting or avoiding.

---

## 1. Jsonnet

**What it is**: A data templating language from Google that extends JSON with
variables, conditionals, arithmetic, functions, imports, and object
composition. It outputs pure JSON (or YAML). Widely used for Kubernetes
manifests (via Tanka, ksonnet), Prometheus/Grafana dashboards (via mixins),
and Terraform configurations.

### Core Concepts

- **Pure functional evaluation**: Jsonnet programs are side-effect-free
  expressions. Every `.jsonnet` file evaluates to a single JSON value.
  Shared libraries live in `.libsonnet` files.

- **Import system**: `import "path.libsonnet"` loads and evaluates another
  file. Imports are lazy and cached. The imported file is parsed in
  isolation -- its behavior is not affected by the importing context. This
  gives strong modularity guarantees.

- **Object composition (`+`)**: The `+` operator merges two objects,
  right-hand side wins on field collisions. The `+:` field syntax enables
  deep merge (patch semantics, preserving unmentioned keys). This is
  Jsonnet's primary composition mechanism.

- **Mixins and inheritance**: Jsonnet uses mixin semantics for inheritance.
  A mixin is a set of overrides that can be passed around as a first-class
  value and applied to any object. `self` refers to the final object (late
  binding), `super` refers to the object being extended.

- **Functions and parameterization**: Top-level functions in library files
  serve as constructors. A library might export
  `newDeployment(name, image, replicas)` that returns a Kubernetes
  Deployment object. Callers customize by passing arguments and using `+`
  to patch the result.

- **Encapsulation**: Fields prefixed with `::` are hidden from output but
  accessible within the object. Libraries expose only their interface.

- **No deep merge in stdlib**: As of 2025, Jsonnet has no built-in deep
  merge function. The `+:` syntax handles most cases, but users wanting
  RFC 7396 JSON Merge Patch semantics need a helper library.

### Relevance to Our Manifest Design

**Strengths**:
- The import/composition model maps well to our manifest directory
  structure. Each manifest file is a self-contained unit (like a
  `.libsonnet`), and the parser composes them based on the include tree.
- Object merging with `+` is analogous to our interface accumulation: a
  parent merges the interfaces of its children, with namespace-resolved
  names as the merge keys.
- The parameterization pattern (library function + `+` override) mirrors
  how our manifests are reusable across namespace contexts -- the manifest
  is the "template" and the include's namespace is the "parameter."

**Weaknesses**:
- Jsonnet is a full programming language (Turing-complete). Our manifests
  are intentionally declarative YAML with no computation. This is a
  feature, not a limitation -- the manifest should describe, not compute.
- No built-in validation or type system. Jsonnet produces JSON; checking
  that the JSON is valid requires external tooling (often CUE or
  JSON Schema). Our manifest crate handles validation internally.
- Late-binding `self`/`super` semantics can make large configurations hard
  to reason about. We avoid this by making composition explicit (the
  parser walks the include tree, not the user).

**Patterns worth adopting**:
- **Isolation of imports**: an imported manifest should not be affected by
  the importing context, except for the namespace parameter. This is
  already our design (manifest files use relative names, parser applies
  namespace).
- **Constructor pattern**: if we ever need programmatic manifest
  generation, a Jsonnet-style function that returns a manifest object
  (parameterized by namespace, vehicle config, etc.) would be natural.

### References
- [Jsonnet Language Design](https://jsonnet.org/articles/design.html)
- [Jsonnet Tutorial](https://jsonnet.org/learning/tutorial.html)
- [Declarative Infrastructure with Jsonnet (Databricks)](https://www.databricks.com/blog/2017/06/26/declarative-infrastructure-jsonnet-templating-language.html)
- [Scaling Kubernetes configuration with Jsonnet](https://tiagomelo.info/jsonnet/kubernetes/devops/configuration/2025/11/03/scaling-configuration-jsonnet.html)

---

## 2. ros-plan

**What it is**: Referenced in our manifest design doc as a "system
description format with sockets, links, and QoS"
([repo](https://github.com/tnagyzambo/ros-plan)).

### Status

The repository at `github.com/tnagyzambo/ros-plan` returns 404 as of
2026-03-23. It appears to have been removed or made private. No cached
documentation, README, or archived content was found via web search or
GitHub API.

### What We Know From Our Reference

The manifest design doc references ros-plan alongside AUTOSAR TIMEX and
CARET, suggesting it defined:
- **Sockets**: typed communication endpoints (analogous to our topic
  declarations with pub/sub roles).
- **Links**: connections between sockets (analogous to our topic-name-based
  implicit connections).
- **QoS**: quality of service annotations on links or sockets.

### Likely Design (Inferred)

Based on the ROS 2 ecosystem context, ros-plan likely defined a
declarative file (TOML, YAML, or custom DSL) that described:
1. Nodes and their communication interfaces (publishers, subscribers,
   services, actions) as typed "sockets."
2. Expected connections ("links") between sockets, making the graph
   explicit rather than relying on name matching.
3. QoS profiles attached to sockets or links.

### Relevance to Our Manifest Design

**Key distinction**: Our manifest uses implicit connection via topic name
matching (same as ROS 2 runtime semantics). An explicit "link" concept
would add a layer of indirection. Our approach is simpler and more aligned
with ROS 2, but explicit links could catch name typos at validation time.

**Design question**: Should we add optional explicit link declarations for
critical paths? Currently, connections are inferred from matching topic
names. An explicit `connections:` section could serve as an assertion that
two specific nodes communicate on a specific topic, independent of name
matching.

---

## 3. Kubernetes Manifests and Reconciliation

**What it is**: Kubernetes uses declarative YAML manifests to describe
desired cluster state. Controllers continuously reconcile actual state
toward desired state. This is the most mature implementation of
"expected vs actual" auditing in production infrastructure.

### Core Concepts

- **Desired state (spec) vs actual state (status)**: Every Kubernetes
  resource has a `spec` (what the user wants) and a `status` (what the
  system observes). The user writes `spec`; controllers populate `status`.
  The gap between them is the work to be done.

- **Reconciliation loop**: Controllers implement a level-triggered (not
  edge-triggered) loop. They compare desired state with actual state and
  take corrective action. The loop is idempotent -- running it multiple
  times produces the same result. This is the "watch, reconcile, repeat"
  pattern.

- **Drift detection**: Configuration drift occurs when runtime state
  diverges from declared state. Common causes: manual `kubectl edit`,
  automated operators, external systems. Detection methods include
  periodic re-reconciliation, GitOps tools (ArgoCD, Flux), and event-based
  watches.

- **Multi-file composition**: Kubernetes supports composing resources from
  multiple files via `kubectl apply -f dir/`. Kustomize adds overlay-based
  composition (base + patches per environment). Helm adds templating with
  parameterized charts.

- **Composition patterns**:
  - **Kustomize**: patch-based. Define a `base/` with shared resources,
    then `overlays/staging/` and `overlays/production/` with patches.
    No templating -- pure declarative patching.
  - **Helm**: template-based. Charts are parameterized Go templates.
    `values.yaml` provides defaults; users override per deployment.
  - **Combined**: Helm renders templates to YAML, Kustomize applies
    overlays on top. Best of both: Helm's ecosystem + Kustomize's
    declarative patching.

### Relevance to Our Manifest Design

**Strengths (patterns to adopt)**:
- **Spec/status separation**: Our manifest is the `spec` (expected graph).
  The runtime `GraphSnapshot` is the `status` (actual graph). The audit
  is the diff. This is exactly the K8s pattern.
- **Level-triggered reconciliation**: Our audit should be level-triggered,
  not edge-triggered. Instead of tracking "topic X was added" events, we
  periodically diff the full expected graph against the full actual graph.
  This is more robust (handles missed events, late starts, etc.).
- **Drift severity classification**: K8s distinguishes between "out of
  spec" (correctable) and "unknown resource" (unexpected). Our audit
  already has this: `new topic` (warn), `missing topic` (info->warn),
  `QoS mismatch` (warn).
- **Overlay composition**: Kustomize's base+overlay pattern maps to our
  per-launch-file manifests composed via the include tree. The base is the
  individual manifest; the "overlay" is the namespace context from the
  include.

**Weaknesses**:
- K8s reconciliation is active (controllers take corrective action). Our
  audit is passive (report deviations, don't fix them). This is
  intentional -- we observe, we don't modify the running system.
- K8s YAML is verbose and repetitive. Our manifest format is deliberately
  compact (shorthand topic declarations, node pub/sub lists).

**Patterns worth adopting**:
- **Idempotent diff**: the audit function should be a pure function
  `diff(expected, actual) -> report`. No side effects. Can be called at
  any time, any number of times.
- **Stabilization period**: K8s controllers have backoff and "ready"
  conditions. Our audit should similarly wait for stabilization before
  escalating "missing topic" from `info` to `warn`.
- **Machine-readable output**: K8s `kubectl diff` outputs structured diffs.
  Our `manifest_audit.json` should be equally structured and parseable.

### References
- [Kubernetes Controllers](https://kubernetes.io/docs/concepts/architecture/controller/)
- [Desired State vs Actual State (downey.io)](https://downey.io/blog/desired-state-vs-actual-state-in-kubernetes/)
- [Kubernetes Configuration Drift (Komodor)](https://komodor.com/learn/kubernetes-configuration-drift-causes-detection-and-prevention/)
- [Kustomize vs Helm (Spacelift)](https://spacelift.io/blog/kustomize-vs-helm)
- [Reconciliation Loop Pattern](https://oneuptime.com/blog/post/2026-02-09-operator-reconciliation-loop/view)

---

## 4. CUE (cuelang.org)

**What it is**: A configuration language with types and constraints built
on lattice theory. CUE unifies types and values into a single hierarchy,
making validation a natural consequence of composition rather than a
separate step.

### Core Concepts

- **Types and values in one lattice**: In CUE, types and values are not
  separate concepts. They exist on a single partially ordered lattice from
  `_` (top/any) to `_|_` (bottom/void). A type like `string` is just a
  value that is more general than `"hello"`. Constraints like `>=0 & <=100`
  are values too.

- **Unification (`&`)**: The core operation. `a & b` computes the greatest
  lower bound (most specific value that satisfies both `a` and `b`).
  Unification is:
  - **Commutative**: `a & b == b & a`
  - **Associative**: `(a & b) & c == a & (b & c)`
  - **Idempotent**: `a & a == a`

  This means the order of combining constraints does not matter. Multiple
  teams can define constraints independently, and CUE guarantees a
  deterministic result.

- **Constraints as first-class**: Constraints are not annotations or
  decorators -- they are values in the lattice. A field can be gradually
  refined: `port: int`, then `port: >0`, then `port: >0 & <65536`, then
  `port: 8080`. Each refinement narrows the lattice position.

- **No inheritance**: CUE deliberately avoids inheritance, mixins, and
  override semantics. Instead, all customization happens through
  unification. This eliminates the "diamond problem" and makes
  configurations easier to reason about.

- **Separation of concerns**: Because unification is commutative,
  constraints can come from different files, different teams, different
  stages of a pipeline. The security team can define `port: >1024`,
  the ops team can define `port: <49152`, and the dev team can define
  `port: 8080`. CUE unifies all three and verifies consistency.

- **Validation by unification**: Validation is not a separate step. When
  you unify a schema (`name: string`) with data (`name: "foo"`), the
  result is the validated data. If they conflict (`name: string` unified
  with `name: 42`), the result is `_|_` (bottom/error).

### Relevance to Our Manifest Design

**Strengths**:
- **Order-independent composition** is extremely valuable for manifests
  composed from multiple include files. CUE's guarantee that order does
  not matter is stronger than Jsonnet's merge semantics (where right-hand
  side wins).
- **Constraints as values** maps directly to our QoS specifications. A
  topic could have `reliability: "reliable" | "best_effort"` as a type
  constraint, and the manifest value `reliability: "reliable"` would be
  validated by unification.
- **Gradual refinement** fits our interface accumulation: a parent can
  add constraints to topics that children declared.
- **Multi-source validation**: the manifest author, the system integrator,
  and the safety engineer could each contribute constraints to the same
  manifest, and CUE would ensure consistency.

**Weaknesses**:
- CUE is a Go-based tool. Integrating it into a Rust codebase would
  require either shelling out to the `cue` CLI or reimplementing the
  lattice evaluator. Neither is practical for our use case.
- CUE's learning curve is steep. The lattice semantics are powerful but
  unfamiliar to most users. Our YAML-based format has near-zero learning
  curve.
- CUE does not natively understand ROS concepts (topics, QoS, namespaces).
  We would need to define CUE schemas for everything, adding a layer of
  abstraction without clear benefit over our direct Rust types.

**Patterns worth adopting**:
- **Commutativity as a design goal**: Our manifest composition should be
  order-independent where possible. Interface accumulation already is
  (set union), but we should verify that all composition operations have
  this property.
- **Constraints on QoS**: Consider allowing partial QoS specifications
  that can be refined at different levels of the include tree. A parent
  manifest could say `reliability: reliable` and a child manifest could
  not contradict it (analogous to CUE's lattice refinement).
- **Validation by construction**: Instead of a separate validation pass,
  make invalid states unrepresentable in the Rust type system (e.g.,
  `enum Reliability { Reliable, BestEffort }` rather than `String`).

### References
- [CUE Introduction](https://cuelang.org/docs/introduction/)
- [How CUE Enables Configuration](https://cuelang.org/docs/concept/how-cue-enables-configuration/)
- [The Logic of CUE](https://cuelang.org/docs/concept/the-logic-of-cue/)
- [How CUE Enables Data Validation](https://cuelang.org/docs/concept/how-cue-enables-data-validation/)

---

## 5. Protocol Buffers / gRPC Service Definitions

**What it is**: Protocol Buffers (protobuf) is Google's language-neutral,
platform-neutral mechanism for serializing structured data. `.proto` files
serve as the single source of truth for data structures and service
interfaces. gRPC uses protobuf as its Interface Definition Language (IDL).

### Core Concepts

- **Contract-first API design**: The `.proto` file defines the interface
  before any implementation. Both client and server are code-generated
  from the same definition, guaranteeing type agreement.

- **Message types**: Structured data definitions with typed, numbered
  fields. Field numbers are the wire-format identity -- they must never
  be reused or changed.

- **Service definitions**: gRPC services define RPC methods with typed
  request/response messages:
  ```protobuf
  service Greeter {
    rpc SayHello (HelloRequest) returns (HelloReply);
  }
  ```

- **Schema evolution**: Protobuf is designed for backward/forward
  compatibility:
  - Adding new fields is safe (old readers ignore them).
  - Removing fields requires reserving the field number.
  - Renaming fields is safe (wire format uses numbers, not names).
  - Changing field types is generally unsafe.

- **Code generation**: `protoc` generates client stubs and server
  interfaces in 10+ languages. The generated code is the executable
  form of the contract.

- **Dual role**: Protobuf serves as both IDL (interface definition) and
  serialization format. The same `.proto` file describes what the
  interface looks like AND how data is encoded on the wire.

### Relevance to Our Manifest Design

**Strengths**:
- **Contract-first is exactly our model**: The manifest defines what the
  communication graph looks like before the system runs. Runtime is then
  audited against the contract.
- **Schema evolution discipline**: Protobuf's rules for backward
  compatibility (never reuse field numbers, reserve removed fields)
  suggest we should think about manifest evolution. What happens when a
  topic is removed from a manifest? Should we track "tombstones" for
  removed topics?
- **Single source of truth**: The `.proto` file is authoritative. Our
  manifest should similarly be the single source of truth for expected
  topology. The runtime graph is an observation, not a source.
- **Type agreement enforcement**: Protobuf enforces that client and server
  agree on message types. Our manifest enforces that all publishers and
  subscribers on a topic agree on message type and (optionally) QoS.

**Weaknesses**:
- Protobuf is point-to-point (client-server). ROS 2 is pub-sub with
  many-to-many connections. Our manifest's implicit connection by topic
  name is fundamentally different from protobuf's explicit service
  definition.
- Protobuf requires code generation. Our manifests are pure data files --
  no compilation step needed.
- gRPC services are synchronous request-response or streaming. ROS 2
  topics are asynchronous fire-and-forget pub-sub with QoS policies. The
  communication models are different enough that direct analogy is limited.

**Patterns worth adopting**:
- **Manifest versioning**: Consider adding a `version:` field to manifests.
  This enables tooling to detect and handle format changes (like protobuf's
  `syntax = "proto3"`).
- **Type-as-contract**: Our topic type declaration
  (`type: sensor_msgs/msg/PointCloud2`) serves the same role as protobuf's
  message type. We should enforce strict type matching (no implicit
  conversions) as protobuf does.
- **Backward compatibility rules**: Define rules for safe manifest
  evolution. Adding a topic: safe. Removing a topic: breaking. Changing a
  topic type: breaking. Widening QoS (reliable -> best_effort): safe.
  Narrowing QoS: breaking.

### References
- [Introduction to gRPC](https://grpc.io/docs/what-is-grpc/introduction/)
- [Protocol Buffers Best Practices for Compatibility](https://earthly.dev/blog/backward-and-forward-compatibility/)
- [Schema Evolution Guide](https://jsontotable.org/blog/protobuf/protobuf-schema-evolution)

---

## 6. ROS 2 Specific Tools

### 6.1 CARET (Chain-Aware ROS 2 Evaluation Tool)

**What it is**: A performance analysis tool from Tier IV that measures
callback latency, node latency, communication latency, and end-to-end
chain latency in ROS 2 applications. Developed for Autoware.

**Core Concepts**:
- **Architecture file**: CARET captures the runtime node/topic topology
  and saves it as a YAML "architecture file." This file describes:
  - Nodes and their callbacks
  - Topic subscriptions and publications per node
  - Intra-node data paths (which subscription triggers which publication)
  - Message context types (how input messages map to output messages)

- **Target paths**: Users define end-to-end paths through the node graph.
  A path is a chain of nodes connected by topics. CARET measures latency
  along these paths.

- **Intra-node data path configuration**: The key user-authored part.
  CARET cannot automatically determine which subscription triggers which
  publication within a node. The user must specify `message_contexts`
  mapping `(subscription, publication)` pairs with a context type:
  - `use_latest_message`: the publisher always uses the latest received
    message (common for periodic nodes).
  - `callback_chain`: direct causal chain from callback to publish.

- **Architecture round-trip**: Capture from LTTng trace -> save to YAML ->
  edit (add paths, message contexts) -> reload for analysis. The YAML file
  is both a snapshot and a user-editable configuration.

**Relevance to Our Manifest**:
- CARET's architecture file is a **descriptive** snapshot of runtime
  topology. Our manifest is a **prescriptive** specification of expected
  topology. They are complementary: CARET tells you what IS; our manifest
  tells you what SHOULD BE.
- CARET's `message_context` concept (mapping sub->pub within a node) is
  related to our `sync` policies. Both describe how data flows through
  multi-input nodes.
- CARET's chain/path definition is equivalent to our `chains:` section
  in the requirements specification (stimulus -> response paths with
  latency budgets).
- **Key difference**: CARET requires LTTng tracing infrastructure and
  post-hoc analysis. Our manifest enables pre-deployment validation and
  live auditing without tracing overhead.

**Patterns worth adopting**:
- CARET's architecture YAML could serve as a capture format. Our
  `--save-manifest-dir` could optionally output a CARET-compatible
  architecture file for teams already using CARET for timing analysis.
- The `message_context` concept could enhance our `sync` policies with
  more precise input-output mapping semantics.

**References**:
- [CARET GitHub](https://github.com/tier4/caret)
- [CARET Architecture Configuration](https://tier4.github.io/CARET_doc/main/configuration/intra_node_data_path/)
- [CARET ROSCon 2022 slides (PDF)](http://download.ros.org/downloads/roscon/2022/Chain-Aware%20ROS%20Evaluation%20Tool%20(CARET).pdf)
- [CARET Paper (IEEE)](https://ieeexplore.ieee.org/document/10086380/)

### 6.2 system_modes (micro-ROS)

**What it is**: A ROS 2 package that adds hierarchical system modes on top
of the lifecycle node concept. Nodes and subsystems can have named modes
(e.g., "active", "degraded", "safe") with specific parameter values.

**Status**: No longer maintained (archived), but the concepts remain
relevant.

**Core Concepts**:
- **System hierarchy**: Nodes are grouped into subsystems. Subsystems can
  contain other subsystems, forming a tree. This is analogous to our
  launch tree scoping (Phase 30).
- **Modes**: Each node or subsystem has named modes. A mode specifies
  parameter values for all nodes in the subtree. Transitioning a
  subsystem to a mode recursively sets its children's parameters.
- **Model file**: A configuration file declares the hierarchy and mode
  definitions. This is a declarative system description -- the closest
  ROS 2 analog to a "system manifest."
- **Integration with launch**: The `launch_system_modes` package provides
  launch actions for declaring systems and nodes with mode support.

**Relevance to Our Manifest**:
- system_modes describes the **configuration space** (what parameters each
  mode sets). Our manifest describes the **communication space** (what
  topics exist and how they connect). These are orthogonal but could be
  combined: a mode change might alter the expected topic graph.
- The hierarchical grouping (systems containing subsystems containing
  nodes) parallels our scope table and component structure.
- system_modes' approach of declaring expected parameter values per mode
  is analogous to our approach of declaring expected topics per launch
  file.

**Patterns worth adopting**:
- **Mode-dependent manifests**: A future extension could allow different
  expected graphs for different system modes (e.g., "full autonomy" mode
  has all perception topics, "manual" mode has only basic topics).

**References**:
- [system_modes GitHub](https://github.com/micro-ROS/system_modes)
- [launch_system_modes ROS Index](https://index.ros.org/p/launch_system_modes/)

### 6.3 irobot ros2-performance

**What it is**: A framework from iRobot for benchmarking ROS 2
communication performance. Uses JSON files to declaratively specify a
system topology, then creates and runs the specified nodes to measure
latency, throughput, and reliability.

**Core Concepts**:
- **Topology JSON**: A declarative description of a ROS 2 system:
  ```json
  {
    "nodes": [
      {
        "node_name": "talker",
        "publishers": [{"topic_name": "/chatter", "msg_type": "String"}],
        "subscribers": []
      }
    ]
  }
  ```
- **Synthetic execution**: The framework creates real ROS 2 nodes from the
  topology description and measures their performance. Nodes are synthetic
  (generate/consume dummy data), but the communication is real DDS.
- **Composable topologies**: Multiple JSON files can be loaded together,
  each running in its own process.

**Relevance to Our Manifest**:
- The topology JSON is structurally similar to our manifest format (nodes
  with pub/sub lists referencing typed topics). The key difference: their
  format is for synthetic benchmarking; ours is for describing real
  application topology.
- Their `msg_type` field corresponds to our topic `type` field.
- No QoS, no interface boundaries, no timing contracts -- it is a
  simpler format focused purely on performance testing.

**Patterns worth adopting**:
- **JSON topology as test input**: We could use our manifest format to
  generate synthetic test topologies for benchmarking, similar to how
  irobot uses theirs.

**References**:
- [ros2-performance GitHub](https://github.com/irobot-ros/ros2-performance)
- [Creating a new topology](https://github.com/irobot-ros/ros2-performance/blob/master/performance_test_factory/create_new_topology.md)

### 6.4 contracts_lite (ros-safety)

**What it is**: A C++ library for defining and enforcing pre/post-condition
contracts, inspired by the C++ Contracts proposal (P0542). Part of the
ROS Safety Working Group.

**Core Concepts**:
- **Preconditions/postconditions**: Functions return `ReturnStatus` with
  a boolean and message. Enforcement macros check at call sites.
- **Build-level enforcement**: Contracts can be compiled out entirely
  (`OFF`), checked only in `DEFAULT` mode, or checked in `AUDIT` mode
  (extended testing). This is a three-tier enforcement model.

**Relevance to Our Manifest**:
- contracts_lite operates at the **function level** (preconditions on
  function calls). Our manifest operates at the **system level**
  (expected communication graph). Different granularity but same
  philosophy: declare expectations, verify at runtime.
- The three-tier enforcement (off/default/audit) is worth considering
  for our auditing: `off` (no manifest), `warn` (log deviations),
  `strict` (fail on deviations).

**References**:
- [contracts_lite GitHub](https://github.com/ros-safety/contracts_lite)

### 6.5 ros_topology_msgs (OSRF)

**What it is**: A ROS message package from OSRF that defines messages for
describing the topology of the ROS computation graph.

**Relevance**: Provides message types for runtime graph topology
(nodes, topics, connections). This is the runtime observation layer --
complementary to our manifest's prescriptive layer.

**References**:
- [ros_topology_msgs GitHub](https://github.com/osrf/ros_topology_msgs)

### 6.6 ROS 2 Discovery and Negotiation

**What it is**: The ROS 2 design article on topological discovery and
communication negotiation describes how nodes discover each other and
negotiate communication parameters at runtime.

**Key insight**: ROS 2's design article states that "combining the life
cycle information of the nodes with the state of graph could allow for
various levels of system verifiability" -- asking "Is the system up and
running?" This is exactly the question our manifest audit answers, but
with a richer vocabulary (not just "is it running" but "is it running
correctly with the expected topology").

**References**:
- [Topological Discovery and Communication Negotiation](https://design.ros2.org/articles/discovery_and_negotiation.html)

---

## Cross-Cutting Design Patterns

### Pattern 1: Spec/Status Separation (K8s)

The most important pattern across all surveyed tools. Clearly separate:
- **Spec** (manifest): what SHOULD exist -- user-authored, version-controlled
- **Status** (runtime): what DOES exist -- observed, ephemeral

Our design already does this. The manifest is the spec; the GraphSnapshot
is the status; the audit is the diff.

### Pattern 2: Composition is Merging (Jsonnet, CUE, Kustomize)

All configuration tools treat multi-file composition as some form of
merging:
- Jsonnet: object `+` (right wins)
- CUE: unification `&` (must agree, order-independent)
- Kustomize: patch overlay (base + patches)

Our manifest composition (interface accumulation through the include tree)
is closest to CUE's model: topics from different manifests must agree on
type, and the result is the union. We should ensure this property holds
rigorously.

### Pattern 3: Contract-First (Protobuf/gRPC)

Define the interface before the implementation. The contract is the source
of truth. Implementation is validated against the contract, not the other
way around.

Our workflow supports both directions:
- **Contract-first**: author manifests, then audit runtime against them.
- **Capture-first**: run the system, capture manifests, then use them as
  baseline for future audits.

The capture-first path is pragmatic for existing systems. The
contract-first path is the ideal for new development.

### Pattern 4: Gradual Adoption (CUE, K8s)

CUE allows mixing fully constrained and unconstrained values. K8s allows
partial specifications (only declare what you care about). Our manifest
supports partial coverage: if a launch file has no manifest, it is
processed normally without auditing. This enables gradual adoption.

### Pattern 5: Level-Triggered Auditing (K8s)

K8s controllers do not track event history. They compare current desired
state with current actual state. This is robust against missed events,
restarts, and timing issues. Our audit should follow this pattern:
periodically compute `diff(expected_graph, actual_graph)`, not track
individual topic creation/destruction events.

### Pattern 6: Schema Evolution (Protobuf)

Protobuf's backward compatibility rules (add is safe, remove requires
reservation, type change is breaking) suggest we should define similar
rules for manifest evolution and document them for users.

---

## Summary Table

| Tool | Model | Composition | Validation | Runtime Audit | Our Analog |
|------|-------|-------------|------------|---------------|------------|
| Jsonnet | Functional templates | `+` merge (right wins) | External (JSON Schema) | No | Import + namespace |
| CUE | Lattice/constraints | `&` unification (order-independent) | By construction | No | QoS constraints |
| K8s | Spec/status | Kustomize overlays, Helm templates | Admission controllers | Reconciliation loop | Manifest audit |
| Protobuf | IDL contract | N/A (single file) | Code generation | N/A | Topic type contracts |
| CARET | Runtime trace | N/A | Post-hoc analysis | No (post-hoc) | Chain latency |
| system_modes | Mode hierarchy | System tree + modes | Mode transitions | Mode monitor | Scope table |
| ros2-perf | Topology JSON | Multi-file loading | Synthetic execution | No | Manifest format |
| contracts_lite | Function contracts | N/A | Build-time | Three-tier | Audit severity |

---

## Recommendations for Our Manifest Design

1. **Preserve order-independence**: Ensure manifest composition through
   the include tree produces the same result regardless of include order
   (CUE's key insight).

2. **Adopt spec/status vocabulary**: Document the manifest as "spec" and
   the runtime graph as "status" to leverage the well-understood K8s
   mental model.

3. **Add manifest version field**: `version: 1` in manifests enables
   future format evolution (protobuf pattern).

4. **Define evolution rules**: Document what changes to a manifest are
   backward-compatible (adding topics: safe; removing: breaking; changing
   type: breaking).

5. **Level-triggered audit**: Implement periodic full-graph diffing rather
   than event-based tracking (K8s pattern).

6. **Three-tier enforcement**: Consider `off`/`warn`/`strict` modes for
   the audit, similar to contracts_lite's build levels.

7. **CARET interop**: Consider exporting manifest + runtime data in a
   format compatible with CARET's architecture YAML, enabling teams to
   use CARET for detailed timing analysis on top of our topology
   specification.
