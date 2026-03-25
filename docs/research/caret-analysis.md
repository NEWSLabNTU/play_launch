# CARET Analysis: Causal Tracing in ROS 2

**Status**: Research
**Date**: 2026-03-26
**Source**: `external/caret_analyze/`, `external/caret_trace/`

## Overview

CARET (Chain-Aware ROS 2 Evaluation Tool) by Tier IV measures
end-to-end latency through chains of ROS 2 callbacks. It instruments
rclcpp via LTTng tracepoints and provides offline analysis of
cause-effect chains.

## Architecture

```
patched rclcpp → LTTng tracepoints → trace data (.ctf)
                                         ↓
                              caret_analyze (Python)
                                ↓
                         architecture YAML + latency plots
```

CARET requires a **forked rclcpp** with additional tracepoints
(`humble_tracepoint_added` branch). This is the major deployment
cost — all ROS nodes must be built against the patched rclcpp.

## Intra-Node Data Path Model

CARET models each node as a set of (subscription, publisher) pairs
called **node paths**. Each pair has a **message context** that
describes how the subscription's input causally relates to the
publisher's output.

### Four Message Context Types

#### 1. `callback_chain`

The subscription callback directly publishes within the same
callback invocation.

```
sub_callback(msg) {
    result = process(msg);
    publisher.publish(result);  // same callback
}
```

**Causal tracking**: Direct — the take and publish are in the same
callback. Measured by callback entry/exit tracepoints.

**Latency**: Callback execution time.

**Autoware examples**: cropbox_filter, ground_filter, centerpoint
(reactive pipe nodes).

#### 2. `inherit_unique_stamp`

The node copies `header.stamp` from input to output. CARET matches
input and output messages by **equal timestamp values**.

```
sub_callback(msg) {
    result = process(msg);
    result.header.stamp = msg.header.stamp;  // copy stamp
    publisher.publish(result);
}
```

**Causal tracking**: Match by `header.stamp` equality. Requires
stamps to be unique (no two messages with the same stamp on the
same topic).

**Latency**: `t_pub(output) - t_take(input)` where stamps match.

**Autoware examples**: Same as callback_chain — most perception
nodes copy stamps. CARET can use either method for these nodes.

#### 3. `use_latest_message`

A periodic/timer callback reads the latest received message and
publishes. There is no direct causal link within a single callback.

```
sub_callback(msg) {
    latest_data = msg;  // store in member variable
}

timer_callback() {
    result = process(latest_data);  // read latest
    publisher.publish(result);
}
```

**Causal tracking**: No exact matching possible. CARET associates
each output with the **most recent input** at the time of output.
Implemented via `merge_sequential` — for each output timestamp,
find the latest input timestamp that precedes it.

**Latency**: `t_pub(output) - t_take(latest_input_before_output)`.
This is NOT the actual processing time — it includes timer wait.

**Autoware examples**: EKF localizer, multi_object_tracker,
trajectory_follower_controller, vehicle_cmd_gate.

#### 4. `tilde` (TILDE)

CARET's custom tracing mechanism. TILDE patches rclcpp to embed a
**unique message ID** per published message. When a node publishes,
TILDE records which input message IDs contributed to the output.

```
// In patched rclcpp:
rcl_publish(msg) {
    tilde_message_id = generate_unique_id();
    record_tilde_publish(publisher, tilde_message_id, input_message_ids);
}
```

**Causal tracking**: Exact — the message ID chain traces from source
to final output through all intermediate nodes.

**Latency**: Exact end-to-end, regardless of node implementation
pattern.

**Cost**: Requires patched rclcpp. All nodes must be rebuilt. The
TILDE tracepoints add overhead to every publish/take.

## Chain Stitching

CARET constructs end-to-end latency by stitching per-hop records:

### `merge_sequential` algorithm

For two tables of timestamped records (left = upstream publishes,
right = downstream takes):

1. Sort both by timestamp
2. For each right record, find the most recent left record with
   timestamp <= right's timestamp
3. If `join_key` is specified, additionally require matching keys
4. Pair them as one hop in the chain

This implements the **use_latest_message** semantics: the downstream
node "used" whichever input was most recently available.

### End-to-end chain

A chain `Topic_0 → Node_1 → Topic_1 → ... → Topic_n` is stitched by:
1. Get per-node records (take→publish pairs)
2. Get per-topic records (publish→take transport)
3. `merge_sequential` across hops
4. Result: table of (stimulus_time, response_time) pairs
5. Latency = response_time - stimulus_time

## Architecture YAML

CARET saves/loads the node graph and message contexts as a YAML file:

```yaml
nodes:
  - node_name: /perception/centerpoint
    paths:
      - subscription_topic_name: /perception/no_ground
        publisher_topic_name: /perception/objects
        context_type: callback_chain
      - subscription_topic_name: /perception/no_ground
        publisher_topic_name: /perception/debug_cloud
        context_type: callback_chain

  - node_name: /perception/tracker
    paths:
      - subscription_topic_name: /perception/fused
        publisher_topic_name: /perception/tracked_objects
        context_type: use_latest_message
```

This is the **user-authored** part — CARET cannot automatically
determine which inputs cause which outputs. The user must declare
the message context for each (sub, pub) pair.

## Comparison with Our Manifest

| Aspect | CARET | Our manifest |
|--------|-------|-------------|
| Intra-node paths | (sub_topic, pub_topic, context_type) | Named paths with input/output endpoints |
| Causal tracking | 4 context types | Implicit from path trigger type |
| Periodic nodes | `use_latest_message` | `state: true` on sub endpoint |
| Runtime tracing | LTTng + patched rclcpp | LD_PRELOAD + SPSC ring buffer |
| User effort | Declare context per (sub, pub) pair | Declare paths + state/required |
| Static analysis | None (runtime only) | Latency composition from manifest |
| Age tracking | `inherit_unique_stamp` or TILDE | Static: latency sum. Runtime: header.stamp or future tracing |

### Key differences

1. CARET is **runtime-only** — it requires trace data to compute
   latency. Our manifest enables **static analysis** before running.

2. CARET's `tilde` provides exact causal tracing but requires
   **patched rclcpp**. Our LD_PRELOAD approach doesn't modify rclcpp.

3. CARET's `use_latest_message` is equivalent to our `state: true` —
   both model the "read latest, no direct trigger" pattern.

4. CARET's architecture YAML (node paths with context types) is
   structurally similar to our `paths:` declarations. We could
   potentially import/export CARET architecture files.

## Implications for Our Design

### Static analysis of `max_age_ms`

We can compute theoretical worst-case age from the manifest alone:

```
max_age(output) = Σ(latency_ms of each node path along causal chain)
                + Σ(topic transport latency)
                + Σ(periodic wait: P + J for each periodic node)
```

No code inspection needed. The causal chain is determined by:
- Node paths (which inputs cause which outputs)
- Topic wiring (which nodes connect)
- State endpoints (which inputs are periodic/latest)

### Runtime monitoring of `max_age_ms`

Three approaches, in order of increasing accuracy:

**Level 1: Static bound only.** Declare `max_age_ms` in manifest,
verify it against the static composition. No runtime measurement.

**Level 2: `header.stamp` where available.** For chains where nodes
copy stamps (perception pipeline), measure `t_pub - header.stamp`.
Falls back to static bound where stamps break.

**Level 3: Embedded tracing (future).** Our interception .so could
embed source creation time in a side channel (similar to TILDE but
via LD_PRELOAD, no rclcpp patching). Provides exact age measurement
for all nodes.

### CARET interop

Our manifest's `paths:` declarations contain the same information as
CARET's architecture YAML. A converter could:
- Export: manifest → CARET architecture YAML (for teams using CARET)
- Import: CARET architecture YAML → manifest paths (bootstrap manifests
  from existing CARET configs)

## References

- [CARET GitHub](https://github.com/tier4/caret)
- [CARET documentation](https://tier4.github.io/CARET_doc/)
- [CARET architecture configuration](https://tier4.github.io/CARET_doc/main/configuration/intra_node_data_path/)
- Source: `external/caret_analyze/src/caret_analyze/architecture/struct/message_context.py`
- Source: `external/caret_analyze/src/caret_analyze/record/interface.py` (merge_sequential)
