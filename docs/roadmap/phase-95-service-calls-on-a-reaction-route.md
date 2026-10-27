# Phase 95 - service calls on a reaction route

**Status: planned (2026-10-27).** Replaces phase 85's D2. Design of record:
rlm `docs/design/service-contracts.md` (related work, the six failure modes,
the coverage table, six simulated workflows).

## Why

The reaction walk crosses service edges (phase 82) and charges them nothing.
A call in stock ROS 2 can fail six ways (no server, server dies, request or
response lost, slow, refused, deadlock); services report none of them, and
rclcpp's client waits forever by default. A route through a call is printed
as a number the system may never meet.

## What

A client states how long it waits and what it does then (`timeout:`,
`on_timeout:`, `on_rejected:`), or the rung is supervised (`safe_state.
supervise`, phase 93). A route through a call is the worse of its success
and failure branches. A route crossing a call with neither is
`call-unbounded`. A runtime companion in play_launch reports unanswered calls
and measures the request leg.

## Wave 1 - grammar and model (rlm)

- [x] W1.1 `cli.<ep>`: `timeout` (duration, `$(param)`-capable),
      `on_timeout` (`{ reaction }` or `{ retry: { attempts, backoff }, then:
      { reaction } }`), `on_rejected` (`{ reaction }`), `max_transport`.
      `services.<s>.max_transport`. Field-table rows, generated reference,
      `GRAMMAR_VERSION`, parse tests for every refusal. *(2026-10-27, rlm
      v0.2.5: contexts `OnTimeout`, `Retry`, `CallReaction`; a `$(param)`
      timeout with no value keeps the key's presence (`CallTimeout.read`), so
      "gives up, how soon unknown" never reads as "waits forever"; a retry
      with no `attempts` or no `then:` parses, for the checker to judge.
      `a_clients_wait_and_fallback_parse` (19 refusals),
      `a_clients_timeout_reads_a_parameter`.)*
- [x] W1.2 `srv.<ep>`: `idempotent` (bool), `stale_after` (duration).
      *(2026-10-27, v0.2.5.)*
- [x] W1.3 Model: the client and server facts reach the model (additive,
      serde-defaulted), so a consumer other than play_launch can read them.
      Release. *(2026-10-27: `CliContract.{timeout_ms, on_timeout, on_rejected,
      max_transport_ms (client's, else the service's), sources}`,
      `SrvContract.{idempotent, stale_after_ms}`; lowered by
      `model_builder::cli_contract`. v0.2.6 followed the same day:
      `Manifest::bounds` had left out `services.<s>.max_transport`.)*

## Wave 2 - the checker (play_launch resolver)

- [x] W2.1 The walk charges a call `max(success, failure)`: success =
      `max_transport` (client over service precedence) + the server's route
      (+ the response leg for a blocking call whose route continues at the
      client); failure = `timeout x attempts + backoff x (attempts - 1)` +
      `on_timeout`'s route. A supervised rung takes no failure branch at the
      call. `--explain` names every term. *(2026-10-27. The walk records
      every call on every branch that reaches the sink (`ReactionWalk.calls`,
      `CallCross`); the success branch is charged inside the walk, the
      failure branch and the rules per rung in `judge_calls`. `on_timeout`'s
      route is the fallback rung's own route walked from the guards, so the
      prefix to the client is counted once. The route note names each call:
      `call /sys/handler/operate -> /comfort/operate: failure = 100.00 ms
      timeout (cli.operate) + 10.00 ms sys.horn route = 110.00 ms; route =
      the failure branch 110.00 ms (success 20.00 ms)`, or `no failure
      branch, the rung's effect is supervised (...)`.)*
- [x] W2.2 Rules: `call-unbounded` (error), `call-retry-not-idempotent`
      (error), `call-retry-unbounded` (error), `call-late-effect` (warning),
      `call-rejected-unhandled` (warning), `call-deadlock` (warning). Only
      calls on a reaction route are judged. *(2026-10-27; `call-unbounded`
      ships as a WARNING, see Deviations. A request leg no route charges is
      `declared-not-charged` (info), as a subscriber's transport is.)*
- [x] W2.3 Fixtures: one per simulated workflow of the design (S1 the
      island's supervised async call, S2 upstream Autoware's blocking call
      and its unbounded retry, S3 a startup call on no route, S4 a
      non-idempotent retry, S5 a timeout past the FTTI caught by
      `ladder-rung-budget`, S6 a fallback that is not a lower rung), plus a
      negative control per rule. *(2026-10-27: `contract_call_supervised`
      (S1), `contract_call_unbounded` and `contract_call_retry_forever` (S2's
      two steps), `contract_call_startup` (S3), `contract_call_retry` (S4),
      `contract_call_timeout_budget` (S5), `contract_call_late_effect` (S6),
      `contract_call_rejected` (`call-rejected-unhandled`, which no
      workflow exercises). The verdict table is
      `sched_derive::tests::every_call_fixture_reports_its_design_verdict`
      (each fixture's exact set of phase 95 rules, plus needles and route
      numbers); end to end through the binary,
      `manifest_check::a_call_on_a_reaction_route_is_bounded_or_reported`.
      Negative controls, each taken against the verdict test and each turning
      it red: every one of the six rules renamed away, the supervised
      exemption disabled, the failure branch zeroed, the request leg
      dropped from the success branch, the blocking wait dropped.)*
- [x] W2.4 Every existing contract fixture resolves byte-identically unless
      it crosses a call; any that does is listed with its new verdict.
      *(2026-10-27, every launch file under `tests/fixtures`, `resolve` and
      `check` before and after: no model and no `check` output moved except
      by `call-unbounded` lines (and the rlm version string). Nineteen
      existing fixtures cross a call with no stated wait and gain
      `warning[call-unbounded]`: `contract_budget_rule` (3),
      `contract_budget_sources` (3), `contract_fault_latch` (1),
      `contract_fault_service` (1), `contract_spelling_deprecated` (3),
      `contract_supervised` (2), `contract_supervised_overrun` (3),
      `contract_supervised_unlowered` (3), `contract_takeover` (3),
      `contract_takeover_bad` (3), `contract_takeover_budget` (6),
      `contract_takeover_expiry_owner` (3), `contract_takeover_expiry_tick`
      (3), `contract_takeover_gate_jitter` (3), `contract_takeover_jitter`
      (3), `contract_takeover_jitter_short` (3), `contract_takeover_link`
      (3), `contract_takeover_response` (3), `contract_values_island` (3).
      As warnings their models are unchanged; as the design's error, 11 of
      them refused to emit a model and 22 integration tests failed. One test
      counted warnings and was updated (`(2 errors, 7 warnings)`). No ranked
      plan moved; the eight new fixtures were added with
      `UPDATE_RANKED_PLAN_SNAPSHOTS=1`.)*

## Wave 3 - runtime companion (play_launch interception)

- [x] W3.1 Hook `rcl_send_request`, `rcl_take_request`, `rcl_send_response`,
      `rcl_take_response`; new event kinds (the consumer's `EventKind` must
      list them). Pair by (client, sequence number). *(2026-10-27. Also
      `rcl_client_init`/`rcl_service_init` (the service's FQN through
      `rcl_*_get_service_name`, the node through `rcl_node_get_*`) and the
      `_with_info` takes. rclcpp on Humble calls `rcl_take_request`/
      `rcl_take_response`, which librcl implements by calling the
      `_with_info` form through its PLT (objdump), so the `_with_info` hook
      sees rclcpp's takes and reads the source timestamp; a thread-local flag
      keeps a take reported once (checked: `add_two_ints_server`, one take).
      Event kinds 23-28 (`ServiceClientInit` .. `ServiceResponseTaken`), in
      the `.so`, the consumer's `EventKind`, `process_event`, the rule
      engine's early return and `EndpointTracker`. A request's response is
      paired by (client, sequence number); a server's take with its send by
      (service, sequence number), the take's source timestamp choosing
      between two clients of one service: the writer GUID cannot, since
      under Fast DDS a response's request id names another writer than the
      request's (measured). `interception/calls.rs`, `calls.json` at
      shutdown.)*
- [x] W3.2 Health issue `call-unanswered`: a request still open past its
      client's contract `timeout:` (10 s without one). *(2026-10-27,
      `health/calls.rs`, kind `call_unanswered` (the Health page's
      snake_case), keyed `call_unanswered:<node>:<service>`. The threshold is
      the model's `cli_endpoints.<ref>.timeout_ms` of the client the
      service's wiring lists under that node. A request open ten minutes
      leaves the open set and is counted (`expired`), so the issue closes.)*
- [x] W3.3 `measure` prints `max_transport` per client from the timed
      request legs (maximum, p50/p99 as comments, as for `budget_us`).
      *(2026-10-27, from `calls.json`, which needs no `--record-events`:
      `#   nodes.<node>.cli.<ep>.max_transport: 1.148ms` after the
      percentiles, every line a comment so stdout stays pasteable; a run
      with calls and no `events.jsonl` measures the calls alone. Also prints,
      per server, how many requests carried a source timestamp.)*
- [x] W3.4 Integration tests: a server that never answers, one that dies
      mid-call, one that answers; overhead measured on the hot path.
      *(2026-10-27, `test_calls_unanswered_reach_health_and_request_legs_are_timed`
      on `simple_test/launch/call_probe.launch.xml` (rclpy, three pairs):
      `call_unanswered` opens for the deaf and the dead server's clients and
      never for the answered one; the answered client's legs are timed
      (p50 0.27 ms, max 1.15 ms under Fast DDS; 0.22 / 1.04 under Cyclone).
      Unit-level negative controls: the Health threshold forced off, and the
      leg pairing dropped, each turn its test red. Overhead: a call hook adds
      44 ns to the rcl call it wraps (endpoint lookup + event + ring push,
      `plugins::calls::call_hook_cost`, release); the topic hot path is
      untouched.)*

## Wave 4 - docs

- [ ] W4.1 Guide section (the contract guide), CHANGELOG, this doc's
      evidence.

## Open decisions (decide at evaluation)

Implemented with the PROVISIONAL choice in brackets, so evaluation can judge
it on real runs:

1. **`stale_after:` needs the request's source timestamp at the server.**
   Not confirmed for every RMW or for nano-ros. [Implemented as a declared
   fact the checker trusts; W3 records whether `rmw_service_info_t.
   source_timestamp` is filled on the RMWs we run.]
   *Learned (2026-10-27):* filled on both RMWs installed here, on requests
   and on responses: Fast DDS (the default, `rmw_fastrtps_cpp`) and Cyclone
   (`rmw_cyclonedds_cpp`), rclpy and rclcpp alike; `call_probe.launch.xml`
   took 26/26 and 27/27 requests with it set, `add_two_ints_server` 1/1.
   `measure` now prints the count per server, so a run on another RMW says
   whether it holds there. nano-ros is not checked.
2. **`call-deadlock` severity.** [Warning. Evaluation: whether play_launch's
   view of the executor (a stock container's executable, `--container-mode`)
   justifies an error.]
   *Learned (2026-10-27):* implemented as a warning on a blocking call from
   a node with no `concurrency:`. The resolver already holds what an error
   would need for composables (`ManifestIndex.executor_of`, the container
   each runs in, and the launch dump names the container's executable,
   `component_container` vs `_mt`), but not for a plain node, whose
   executor is in its code; and `--container-mode` is a runtime choice
   `check` never sees. Only S2's fixture fires it.
3. **Actions** (goal, cancel, result). [Out of this phase; evaluation decides
   whether they get the same keys under `act:`.]
4. **nano-ros's agreement** that `timeout:`/`on_timeout:` derive no deadline
   or monitor beyond a subscriber's `on_violation`. [The model carries the
   facts; nothing in play_launch derives a deadline from them.]
   *Learned (2026-10-27):* holds: `CliContract.timeout_ms` is read only by
   the fault-reaction arithmetic (a term of the failure branch) and, in W3,
   by Health's `call-unanswered` threshold, which observes and derives
   nothing.
5. **`call-unbounded` as an error.** [Error. Evaluation: how many real
   contracts it turns red (the island must stay clean through its
   supervised rung).]
   *Learned (2026-10-27):* as an error it turns the island red (2 errors,
   0 before): its comfortable rung is supervised and stays clean, but the
   emergency floor crosses `mrm_handler.cli.emergency_stop_operate` (async,
   no timeout) and nothing supervises the floor. It also turns 19 of the 81
   existing `contract_*` fixtures red, refusing 11 models. Shipped as a warning
   (Deviations 1): the island checks `0 errors, 14 warnings` (12 before, the
   two new ones being that floor call on `hpc_loss` and `odd_exit`). For a
   FLOOR, `timeout:` + `on_timeout:` cannot bound the call (there is no
   lower rung to fall to); only a supervised effect, or a guard on the
   floor's own output (`reaction-unguarded` already asks for one), can.

## Deviations

1. **`call-unbounded` is a warning, not an error** (2026-10-27). Checked as
   the design's error, it refused the model of 11 existing fixtures and
   failed 22 integration tests, and it turned the safety island red: the
   island's comfortable rung is supervised, as the design says, but its
   emergency FLOOR also crosses a call (`mrm_handler` ->
   `/system/mrm/emergency_stop/operate`, `call: async`, no timeout) and
   nothing supervises the floor. The design's S1 considered only the
   comfortable rung, so "error" and "the island stays clean" cannot both
   hold. A warning is the severity every other incomplete-evidence route
   already has (`reaction-unbudgeted`), it never hides the finding, and
   `check --strict` makes it an error. Open decision 5 stays open with this
   evidence.
2. **A blocking call holds the client path's other outputs.** The design's
   "the route continues at the client" is read as: the path that sends a
   blocking call also publishes onward, and every such branch (topic
   outputs, or the path's own safe state) is charged the wait = request leg
   + the server's response (its `max_response`, else its request-triggered
   path's `max_latency`) + the response leg, the request leg's
   `max_transport` again. The contract cannot say whether the publish comes
   before the call in the code, so the walk assumes after (the longer).

## Decisions where the design was silent

- `on_timeout`'s route is the fallback's own route walked from the guards
  (a mode's `reaction:` scope path, or a scope path), its success branch
  only; a call the fallback itself crosses is judged on that rung. Its
  settle is not added: the rung's settle stays the rung's.
- A rung is supervised for a call when the safe state its route reaches has
  a supervised settle, or a node path's `max_latency` at or after the
  client's hop is supervised (one before the call watches the request go
  out, not its effect).
- The retry rules and `call-late-effect` judge a call even on a supervised
  rung (an unsound retry stays unsound under a supervisor);
  `call-rejected-unhandled` does not (the coverage table: a supervised
  effect covers F5). `call-deadlock` fires regardless.
- "Lower rung" for `call-late-effect` is a rung after the call's rung in the
  hazard's ladder, named by mode or by scope path; a call on the floor has
  none, and falling to the same rung is not lower.
- An `on_timeout` naming neither a mode nor a scope path is
  `reaction-unreachable` (error), the rule that already says a reaction
  names nothing.
- A timeout read from the launch tree with no value gives no failure branch
  and no `call-unbounded` (the code does give up); the note says the branch
  is unknown, and phase 94's `param-unset` reports the read.

## Acceptance

- The island's contract checks with unchanged verdicts (its comfortable
  rung is supervised); its route through `operate` gains only
  `max_transport` if declared.
- Each simulated workflow of the design is a fixture with the verdict the
  design states.
- `call-unanswered` fires on a server that never answers and stays silent on
  one that does.
