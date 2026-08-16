---
id: 17
title: "Model FQN and ROS graph name disagree for any node the launch file did not name"
status: open
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
