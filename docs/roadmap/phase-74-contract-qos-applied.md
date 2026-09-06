# Phase 74 — the contract's QoS reaches the running node

Status: **complete** (2026-09-06; manifest crate `v0.1.32`). Closes
`contract-axes.md` §4's ruling ("derive always, apply where accepted, report
where it cannot be") and phase 73's "not done".

## What it does

`rclcpp` accepts `qos_overrides.<topic>.<publisher|subscription>.<policy>`
parameters for the policies a node opted in to with `QosOverridingOptions`.
The contract's `qos.deadline`, `qos.liveliness` and `qos.lease_duration` are
exactly those policies, so the resolver writes them into the node's
parameters **on the model** — both the `params` map readers index and the
ordered `param_sources` list a spawn actually renders — where every spawn
path already honours them. A launch-file parameter of the same name wins.
The endpoint QoS lowered to `Pub/SubContract.qos` is the EFFECTIVE one
(topic default overlaid with the endpoint's own), the overlay `qos-match`
checks; the model's `Qos` gained `deadline_ms` and `lease_duration_ms`.

`up` **reports** acceptance by asking each node afterwards — and compares
**values**, not names: a node that opted in declares every `qos_overrides.*`
parameter itself with the profile's default, so the name alone proves
nothing. `[qos-override] <node>: contract QoS applied — liveliness,
liveliness_lease_duration`, or the undeclared / differing policies by name.

The live observer's silence threshold is now the declared lease (or
`max_age`, whichever is shorter), not the topic's cadence.

## What the first runs found

Three things, each a real defect in the apply path.

1. **The parameters never left the model.** The spawn renders
   `param_sources`, the ordered list; the overrides were inserted into
   `params`, the map. The node's cmdline and inline YAML carried none of
   them — while the acceptance report said *accepted*, because rclcpp had
   declared the parameters on its own.
2. **`automatic` liveliness cannot detect a silent publisher.** It is
   asserted by the *participant*; a node that stops publishing but stays up
   never lapses. `manual_by_topic` — the contract's original choice — is
   asserted by each publish, and lapses at the lease.
3. **rclcpp takes a QoS event only when a callback is registered for it.**
   Without `event_callbacks.liveliness_callback` on the subscription the
   lease expiry never surfaces, and the observer falls back to its own
   clock. The demo registers one; a node that does not still gets the
   observer's tick against the declared lease.

## Measured

`just fault`, the lidar's data stopped at 8 s, both nodes opted in:

```
[qos-override] /safety/lidar_driver:       contract QoS applied — liveliness, liveliness_lease_duration
[qos-override] /safety/obstacle_detector:  contract QoS applied — liveliness, liveliness_lease_duration
hazard-detected:  '/safety/scan' reported LivelinessChanged by DDS 99.30ms after its last publish (mechanism: qos)
hazard-reaction:  105.39ms after the fault; + settle 200ms — fits ftti 500ms
```

The detector is DDS, from a number in the contract, with no watchdog code
in the path. The recipe fails if detection is not `by DDS` or the QoS is
not applied by value on both nodes.

## Not done

- `deadline` is applied the same way but the demo does not exercise a
  `RequestedDeadlineMissed` event; the observer treats it as a detection.
- A node that has NOT opted in silently ignores the parameters; the report
  says so, and nothing else can — that is rclcpp's rule
  (`QosOverridingOptions() = default` means "no overrides allowed").
