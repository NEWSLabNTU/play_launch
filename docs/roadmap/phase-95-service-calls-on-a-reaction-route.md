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

- [ ] W1.1 `cli.<ep>`: `timeout` (duration, `$(param)`-capable),
      `on_timeout` (`{ reaction }` or `{ retry: { attempts, backoff }, then:
      { reaction } }`), `on_rejected` (`{ reaction }`), `max_transport`.
      `services.<s>.max_transport`. Field-table rows, generated reference,
      `GRAMMAR_VERSION`, parse tests for every refusal.
- [ ] W1.2 `srv.<ep>`: `idempotent` (bool), `stale_after` (duration).
- [ ] W1.3 Model: the client and server facts reach the model (additive,
      serde-defaulted), so a consumer other than play_launch can read them.
      Release.

## Wave 2 - the checker (play_launch resolver)

- [ ] W2.1 The walk charges a call `max(success, failure)`: success =
      `max_transport` (client over service precedence) + the server's route
      (+ the response leg for a blocking call whose route continues at the
      client); failure = `timeout x attempts + backoff x (attempts - 1)` +
      `on_timeout`'s route. A supervised rung takes no failure branch at the
      call. `--explain` names every term.
- [ ] W2.2 Rules: `call-unbounded` (error), `call-retry-not-idempotent`
      (error), `call-retry-unbounded` (error), `call-late-effect` (warning),
      `call-rejected-unhandled` (warning), `call-deadlock` (warning). Only
      calls on a reaction route are judged.
- [ ] W2.3 Fixtures: one per simulated workflow of the design (S1 the
      island's supervised async call, S2 upstream Autoware's blocking call
      and its unbounded retry, S3 a startup call on no route, S4 a
      non-idempotent retry, S5 a timeout past the FTTI caught by
      `ladder-rung-budget`, S6 a fallback that is not a lower rung), plus a
      negative control per rule.
- [ ] W2.4 Every existing contract fixture resolves byte-identically unless
      it crosses a call; any that does is listed with its new verdict.

## Wave 3 - runtime companion (play_launch interception)

- [ ] W3.1 Hook `rcl_send_request`, `rcl_take_request`, `rcl_send_response`,
      `rcl_take_response`; new event kinds (the consumer's `EventKind` must
      list them). Pair by (client, sequence number).
- [ ] W3.2 Health issue `call-unanswered`: a request still open past its
      client's contract `timeout:` (10 s without one).
- [ ] W3.3 `measure` prints `max_transport` per client from the timed
      request legs (maximum, p50/p99 as comments, as for `budget_us`).
- [ ] W3.4 Integration tests: a server that never answers, one that dies
      mid-call, one that answers; overhead measured on the hot path.

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
2. **`call-deadlock` severity.** [Warning. Evaluation: whether play_launch's
   view of the executor (a stock container's executable, `--container-mode`)
   justifies an error.]
3. **Actions** (goal, cancel, result). [Out of this phase; evaluation decides
   whether they get the same keys under `act:`.]
4. **nano-ros's agreement** that `timeout:`/`on_timeout:` derive no deadline
   or monitor beyond a subscriber's `on_violation`. [The model carries the
   facts; nothing in play_launch derives a deadline from them.]
5. **`call-unbounded` as an error.** [Error. Evaluation: how many real
   contracts it turns red (the island must stay clean through its
   supervised rung).]

## Acceptance

- The island's contract checks with unchanged verdicts (its comfortable
  rung is supervised); its route through `operate` gains only
  `max_transport` if declared.
- Each simulated workflow of the design is a fixture with the verdict the
  design states.
- `call-unanswered` fires on a server that never answers and stays silent on
  one that does.
