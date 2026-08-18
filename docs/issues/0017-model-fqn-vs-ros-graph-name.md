---
id: 17
title: "Model FQN and ROS graph name disagree for any node the launch file did not name"
status: resolved
type: correctness
severity: medium
---

# 0017 — the model's node FQN is not the node's ROS name

**Repo:** `play_launch`
**Affects:** `structure.nodes` keys, `execution/startup_order.rs`, anything that
joins model identity to the live ROS graph
**Found by:** phase-61 W2's stage gate, which is the first consumer to require
the two to agree.

## The fact

For a `<node>` with no `name=`, the model keys the instance by its **executable**
while the running node registers whatever name it was compiled with. play_launch
emits `__ns` but **no `__node` remap**, so it never imposes the model's name:

    $ cat play_log/.../autoware_ekf_localizer_node/cmdline
    .../autoware_ekf_localizer_node --ros-args \
        -r __ns:=/localization/pose_twist_fusion_filter ...

    model:  /localization/pose_twist_fusion_filter/autoware_ekf_localizer_node
    graph:  /localization/pose_twist_fusion_filter/ekf_localizer

(CLAUDE.md says `exec_name` is used as a `__node` remap. It is used as the FQN
map *key*; no remap reaches the command line on this path.)

On the golf cart stack, **19 of 144** model FQNs do not appear in
`ros2 node list`. Resolving them against the live graph — two probes, 90 s
apart, `ros2 node list --no-daemon`, identical results — accounts for every one:

| model FQN | ROS graph name | evidence |
|---|---|---|
| `/default_adapi/helpers/initial_pose_adaptor_node` | `/default_adapi/helpers/autoware_initial_pose_adaptor` | exec name, `autoware_`/`_node` normalised |
| `/default_adapi/helpers/routing_adaptor_node` | `/default_adapi/helpers/autoware_routing_adaptor` | exec name, `autoware_`/`_node` normalised |
| `/localization/autoware_localization_error_monitor_node` | `/localization/localization_error_monitor` | exec name, `autoware_`/`_node` normalised |
| `/localization/pose_twist_fusion_filter/autoware_ekf_localizer_node` | `/localization/pose_twist_fusion_filter/ekf_localizer` | exec name, `autoware_`/`_node` normalised |
| `/localization/pose_twist_fusion_filter/autoware_stop_filter_node` | `/localization/pose_twist_fusion_filter/stop_filter` | exec name, `autoware_`/`_node` normalised |
| `/localization/pose_twist_fusion_filter/autoware_twist2accel_node` | `/localization/pose_twist_fusion_filter/twist2accel` | exec name, `autoware_`/`_node` normalised |
| `/localization/twist_estimator/autoware_gyro_odometer_node` | `/localization/twist_estimator/gyro_odometer` | exec name, `autoware_`/`_node` normalised |
| `/localization/util/autoware_pose_initializer_node` | `/localization/util/pose_initializer` | exec name, `autoware_`/`_node` normalised |
| `/localization/util/default_adapi/helpers/autoware_automatic_pose_initializer_node` | `/localization/util/default_adapi/helpers/autoware_automatic_pose_initializer` | exec name, `autoware_`/`_node` normalised |
| `/map/autoware_map_projection_loader_node` | `/map/map_projection_loader` | exec name, `autoware_`/`_node` normalised |
| `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner` | *(absent)* | composable; child process alive to SIGTERM but no node in the graph |
| `/sensing/autoware_vehicle_velocity_converter_node` | `/sensing/vehicle_velocity_converter` | exec name, `autoware_`/`_node` normalised |
| `/sensing/imu/gyro_bias_estimator_node` | `/sensing/imu/gyro_bias_scale_validator` | sole remaining node in the namespace |
| `/sensing/imu/imu_corrector_node` | `/sensing/imu/imu_corrector` | exec name, `autoware_`/`_node` normalised |
| `/sensing/lidar/falcon/seyond_node` | `/sensing/lidar/falcon/seyond` | exec name, `autoware_`/`_node` normalised |
| `/sensing/lidar/vlp32/velodyne_ros_wrapper_node` | *(not running)* | `SocketError: Cannot assign requested address` — no LiDAR on the bench |
| `/system/aggregator_node` | `/system/aggregator` | exec name, `autoware_`/`_node` normalised |
| `/system/converter_node` | `/system/converter` | exec name, `autoware_`/`_node` normalised |
| `/system/service_log_checker_node` | `/system/service_log_checker` | exec name, `autoware_`/`_node` normalised |

- renamed, same node: **17**
- genuinely absent:   **2**

`node_name.is_none()` is an exact discriminator for the renamed set: 17 nodes,
all of them with no `name=` in the launch file.

## The two that are not renames

- **`velodyne_ros_wrapper_node`** — genuinely not running. Its own stderr:
  `terminate called after throwing an instance of
  'nebula::drivers::connections::SocketError' what(): Cannot assign requested
  address`. Expected on a bench with no LiDAR.

- **`motion_velocity_planner`** — a separate defect, since resolved as #0019:
  its constructor ran past the isolated container's then-hardcoded 30 s ready
  timeout, so the container SIGKILLed it and play_launch counted the LoadNode
  response as success anyway. Original observation kept below. play_launch reports `composable 84/84 loaded`; the
  container logs `Spawned isolated child PID ... Component ... loaded as ...`;
  the child's own stderr shows it alive until `signal_handler(SIGINT/SIGTERM)`
  at shutdown — and yet no such node exists in the ROS graph, at 40 s or at
  90 s, while its container siblings (`elastic_band_smoother`, `path_optimizer`,
  `glog_component`) all do. It is also the composable whose
  `ComponentEvent LOADED` never arrived, so play_launch fell back to the LoadNode
  service response after 10 s. A process that is alive with rclcpp initialised
  but has no node in the graph is worth its own investigation; **`84/84 loaded`
  is currently optimistic**.


## Fix

Two halves, matching the two things that were wrong: the mismatch was never
STATED, and the real name was never KNOWN.

### 1. Stated — one home for the rule, and a diagnostic in the artifact

`ros_launch_resolve::ros::graph_identity` is now the single place that answers
"is this key a ROS name?" (`is_ros_graph_name`, `ros_graph_name`,
`non_graph_names`), with the reasoning and the `af7c524` history beside it.
Phase 61's stage gate was the first consumer and had the test inline; it now
calls the shared function, so the next graph-joining consumer finds the rule
instead of rediscovering it.

`build_checked_model` embeds a diagnostic in every model it produces:

    identity: 17 of 145 node key(s) are derived from the executable because the
    launch file declared no `name=`, so they are NOT the names those nodes
    register in the ROS graph and must not be matched against `ros2 node list`
    (/sensing/lidar/falcon/seyond_node-1, /system/service_log_checker_node-1, …)

It rides in `meta.diagnostics`, so it reaches the ARTIFACT rather than only
whichever terminal ran `resolve`. One line rather than one per node.

**17** — the same count this issue measured against the live graph, and it
excludes both nodes that were absent for unrelated reasons. The discriminator
and the observation agree exactly.

### 2. Known — the interceptor reports the real name

`libplay_launch_interception.so` now writes
`play_log/<ts>/interception/node_identity.tsv`:

    /probe/explicitly_named	881643	/probe/explicitly_named
    /probe/talker-1	        881644	/probe/talker

`model key<TAB>pid<TAB>real FQN`. The name is read from the `rcl_node_t*` that
`rcl_publisher_init` / `rcl_subscription_init` already receive — the hooks
already call `rcl_node_get_name`/`rcl_node_get_namespace` on it to expand
relative topics — so this is authoritative, not inferred, and costs a hash-set
lookup per init call and nothing on the hot path.

Three things that shaped it:

- **Not the ring buffer.** `InterceptionEvent` is a fixed 56-byte record that
  carries topics as FNV-1a hashes, which works only because the consumer
  already knows the topic names and can hash them to compare. That is exactly
  what does not hold here: play_launch does not know the name, so a hash it
  cannot invert reports nothing. Hence a file, `O_APPEND`, one line per write.
- **The model key travels with it.** Joining by PID alone breaks for a
  container process hosting several nodes, so play_launch passes the key it
  spawned the process under (`PLAY_LAUNCH_INTERCEPTION_MEMBER`).
- **Not gated on plugins**, and de-duplicated per FQN rather than per process,
  because one process can host many nodes and each creates many publishers.

Limits, unchanged from the analysis above: interception is off by default, and
a node that creates neither a publisher nor a subscription is never seen.
Hooking `rcl_node_init` directly would make it earlier and unconditional, and
is the obvious next step if such a node turns up.

### What was deliberately NOT done

The model key was not changed, and no `-r __node:=` was emitted. That inverts
which of the two names is authoritative and is bug `af7c524`. The remaining
piece is one line of documentation in `ros-launch-manifest`, whose
`structure.nodes` doc comment still reads "Node FQN (`/ns/node_name`) →
instance" — a cross-repo change with a tag bump across three manifests, and
the misreading it invites is now contradicted by a diagnostic in every model
that has the problem.

## Verified

- `identity:` diagnostic fires on the golf cart stack, reporting 17 of 145.
- `test_interception_reports_real_node_names` asserts the DIFFERENCE between a
  named and an un-named node in one launch — a test that checked only the
  un-named one would pass equally if every key were reported wrong.
- Interception suite 8/8; `just test` 85 passed.

## Why it matters

Anything that joins model identity to the live graph is wrong for these nodes:
the phase-61 stage gate (worked around — it requires a graph match only where
`node_name` is set, and falls back to process-running otherwise), and any future
readiness check, liveness probe, or graph-based diagnostic.

## Options explored

### Rejected — force the model's name onto the node (`-r __node:=<leaf>`)

This is the obvious fix and it is wrong twice over.

**It diverges from `ros2 launch`.** Stock `launch_ros` emits the name remap
only when the launch file gave one:

```python
# launch_ros/actions/node.py:493
if self.__node_name is not None:
    ros_specific_arguments['name'] = '__node:={}'.format(self.__expanded_node_name)
```

play_launch already reproduces that exactly. Emitting `__node` unconditionally
would make play_launch produce different node names than `ros2 launch` for the
same file — the one thing this project's parser rule forbids. It would also be
a Rust/Python parser divergence, since the Python path goes through
`launch_ros` and cannot emit it either.

**It has already been tried and reverted.** From `node_cmdline.rs`:

> NOT reintroducing the `af7c524` bug (forcing `__node` onto a `name=None` node
> silently renames it away from its internally-hardcoded default, breaking e.g.
> LifecycleNode service discovery).

A node with no declared name is *entitled* to its compiled-in name, and other
things address it by that name. Renaming it to match a synthetic model key
inverts which of the two is authoritative.

(Note: CLAUDE.md states `exec_name` is used as a `__node` remap. That is stale —
it is used as the FQN map *key* and as the log-directory name, and the remap is
conditional. Corrected in this change.)

### Rejected — infer the name from the ROS graph by namespace

Match each unresolved model FQN against unclaimed graph names in the same
namespace. This is what produced the table above, and it is fine for a one-off
analysis with a human checking it. As a mechanism it is not safe: two un-named
nodes in one namespace are ambiguous, and this corpus contains exactly that case
(`gyro_bias_estimator_node` and `imu_corrector_node` under `/sensing/imu`). It
took a second pass plus reading `libgyro_bias_estimator.so` to settle which was
which. Inference dressed as identity.

### Possible — read the effective name from the rcl interceptor

`libplay_launch_interception.so` is already injected into every child and
already hooks `rcl_publisher_init` / `rcl_subscription_init`, both of which
receive an `rcl_node_t*`. `rcl_node_get_name()` / `rcl_node_get_namespace()` on
that handle give the node's real FQN, and the interceptor knows its own PID — so
this is an authoritative PID → node-FQN map with no inference and no new hook
mechanism (hooking `rcl_node_init` directly would make it earlier and
unconditional).

Limits: interception is off by default, and a node that never creates a
publisher or subscription is never seen unless `rcl_node_init` is hooked.

### Possible — read it from the node's own logs

rclcpp logger names are the dot-separated FQN, and play_launch already captures
every child's stderr:

    [INFO] [...] [planning.scenario_planning.lane_driving.motion_planning.motion_planning_container]: ...

Zero new machinery. But it needs the node to log something, and log formatting
is not a contract.

### Preferred — stop presenting a synthetic key as a ROS name

The real defect is not that the name is unknown; it is that
`structure.nodes`'s documentation says the key is `/ns/node_name` while for
these nodes it is `/ns/<executable>`. A consumer reading the schema has every
reason to join it against `ros2 node list`, and 17 of 144 will silently fail to
match — with no error anywhere, because the manifest checker builds its graph
from the manifest and never looks at the live system.

Two parts, in order of value:

1. **Say so in the model.** `node_name.is_none()` already discriminates the case
   exactly; what is missing is the schema stating that the key is a resolve-time
   identity, not a ROS name, and is joinable against the graph only when
   `node_name` is set. That is a documentation change to
   `ros-launch-manifest`, plus possibly an `effective_name: Option<String>`
   field for anything that later discovers it.
2. **Fill it in where it is cheap**, via the interceptor, for consumers that
   need the live name.

Phase 61's stage gate already implements (1)'s semantics in code — it requires a
graph match only where `node_name` is set — which is the workaround, not the
fix, because every future graph-joining consumer has to rediscover the rule.

### Practical consequence today

When authoring contracts or manifests for a stack like this, endpoints and paths
must be keyed by the **model** FQN, not by what `ros2 node list` prints. For
17 of these 144 nodes those differ, and nothing will report the mismatch.

## Original options sketch


1. **Emit `-r __node:=<model name>`** so the model's FQN is the truth. Correct,
   and invasive: node names key parameters and remappings, so this moves
   user-visible identity and could break param files that address nodes by their
   real name. Not a change to make as a side effect.
2. **Record the observed graph name** on the instance once discovered, leaving
   the model key alone. Non-invasive; makes the join possible without changing
   anyone's node names.
3. **Leave it, document it.** Where things stand now.

Option 2 looks right, but it is a model-schema change and belongs to whoever
owns `ros-launch-manifest`.

## Reproducing

    scripts/edge-storm/… launch the stack, then
    ros2 node list --no-daemon > nodes.txt
    # diff against the keys of `structure.nodes` in the resolved model

`--no-daemon` matters: a cold `ros2 node list` returns an EMPTY list rather than
an error, which reads as "nothing is running".
