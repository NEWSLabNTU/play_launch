---
id: 80
title: "graph-deviation-runtime reports every topic of every node, including nodes no contract describes"
status: resolved
type: defect
severity: low
---

# 0080 - `graph-deviation-runtime` judges nodes no contract describes

**Repo:** `play_launch` at `134ce5f1` (0.29.0)
**Affects:** `src/play_launch/src/runtime_enforcement/mod.rs`
(`RuleEngine::observe`, the endpoint-init branch) and `view.rs`
(`ContractView`)

## Symptom

The Autoware 1.5.0 planning simulator was run under `play_launch launch`
with the fixture's overlay contracts, which cover 4 of its 83 launch
scopes. In three minutes it wrote 481 `graph-deviation-runtime` warnings
to `runtime_violations.jsonl`, one per distinct topic:

```
unknown topic at publisher creation — no manifest declares '/tf' (hash 0x...); add to `topics:` or `external_topics:`
```

Most came from nodes in the 79 scopes no contract covers: `/tf`,
`/robot_description`, `/joint_states`, and every `debug/processing_time_ms`.
The advice to add them to `topics:` asks the author of a different launch
file to declare topics of nodes their contract does not describe.

## Cause

The rule asked only "does any manifest declare this topic?" and never "is
the endpoint's node one a contract describes?". With partial contract
coverage, the normal case for a large system, every node outside the
contracted scopes was judged against contracts that make no claim on it.

## Fix

`ContractView::contracted_nodes` holds the node FQNs some contract
describes:

- every node of a launch scope that has a contract (`ScopeInfo.manifest`);
- the owner of any publisher, subscriber, service, client, action-client
  or node-path entry, and any `node_concurrency` or `node_params` key, for
  a contract that states something about a node in another scope.

The first is needed because the model carries no entry for an endpoint
the contract lists with nothing stated (`chatter: {}`). The first draft
used only the second, and two integration tests caught it: their contract
names `talker` that way, so `talker` counted as undescribed.

The interceptor already sends `EndpointOwner` (handle and node FQN hash)
before each endpoint's init. The engine keeps it by handle and takes it at
the init:

- **Owner is a described node:** judged as before.
- **Owner is known but not described:** skipped. Its topic deviates from
  no contract.
- **Owner is unknown:** judged as before. This covers an interceptor that
  sends no `EndpointOwner`.

Same Autoware launch, same contracts: **481 to 39** warnings after 60 s.
All 39 come from nodes in the contracted launch files: the control stack
(`trajectory_follower`, `vehicle_cmd_gate`, `autonomous_emergency_braking`,
`control_validator` and others), the MRM operator and `/service_log`.
These are topics those contracts' nodes create without the contracts
declaring them, which is what the rule exists to report.

Test: `graph_deviation_judges_only_a_node_a_contract_describes` sends the
same undeclared publisher once for a described owner and once for an
undescribed owner. The described owner must fire and the undescribed one
must not. It fails with the filter removed (negative control run).

## Limits

- A node the launch file leaves unnamed has a model key derived from its
  executable, not its ROS name (#0017). Its owner hash therefore matches
  no described node, and its endpoints are skipped. Contracts describe
  nodes by name, so such a node is in practice never described.
- The owner is matched by endpoint handle, which is a process-local
  address. A different process's init with the same address arriving
  between an `EndpointOwner` and its init would take the wrong owner. This
  needs the same address in two processes inside one drain pass. No case
  was seen.
