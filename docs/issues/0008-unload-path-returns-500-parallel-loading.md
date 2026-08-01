---
id: 8
title: "Integration tests used stale member names after the <kind>:/ rename (originally misfiled as an unload-path 500)"
status: resolved
type: bug
severity: medium
---

# 0008 — Integration tests used stale member names after the `<kind>:/` rename

**Affects:** `tests/tests/parallel_loading.rs`, `tests/src/health.rs`,
`tests/tests/contract_eject.rs`

> **Correction.** This issue was originally filed as *"Composable-node unload
> returns HTTP 500"*, describing a real defect in the unload path. **That
> diagnosis was wrong.** The unload path works correctly; the tests were
> calling it with names it never used. The original bisect was sound in what
> it proved — the failures predated the `machine=` removal branch and were not
> environmental — but it misattributed the cause. Corrected below.

## Summary

Six integration tests failed for one shared reason: member IDs gained a
`<kind>:/` prefix and the tests were never updated.

`5b20c1e` ("feat(phase-50): canonical member identity", 2026-07-25 16:13)
introduced member IDs of the form `<kind>:/<name>` — `node:/rviz2`,
`composable:/fast_talker`, `container:/lc_unload_container`. It touched 25
files including `web/handlers.rs` (the unload endpoint these tests call) and
`web_types.rs`, and **no integration test files**.

(An earlier revision of this issue credited `416b533`, phase-52, 18:33 the
same day. That commit reworked the same actor files later but did not
introduce the ID format — `git show 416b533 -- …/component_events.rs` shows
no naming change. Issue #0001 records phase-50 as the origin of canonical
member ids `kind:/ns/name[#N]`, and that is correct.)

## Evidence

`GET /api/nodes` returns:

```
['composable:/fast_talker', 'container:/lc_unload_container', 'composable:/slow_node']
```

Reproduced directly, outside the test harness:

```
POST /api/nodes/fast_talker/unload                → HTTP 500
  WARN [Web UI] Failed to unload 'fast_talker': Actor not found: fast_talker

POST /api/nodes/composable:%2Ffast_talker/unload  → HTTP 200
  DEBUG Calling UnloadNode service (unique_id: 1)
  DEBUG UnloadNode response: success=true, error_message=
  DEBUG Successfully unloaded composable node 'composable:/fast_talker'
```

The 500 is the handler's own error path
(`web/handlers.rs::unload_node` → `INTERNAL_SERVER_ERROR`) reporting that no
actor by that name exists. Given the real ID, unload succeeds.

The `/` inside a member ID must be percent-encoded (`composable:%2F…`) so the
router treats the ID as a single path segment.

## The six failures

| Test | Stale expectation |
|---|---|
| `parallel_loading::test_unload_via_web_api` | `POST /api/nodes/fast_talker/unload` |
| `parallel_loading::test_unload_and_reload` | same, plus `/load` |
| `parallel_loading::test_unload_during_construction` | `POST /api/nodes/slow_node/unload` |
| `parallel_loading::test_fast_not_blocked_by_slow` | greps `ComponentEvent LOADED for 'fast_talker'` |
| `autoware::test_autoware_smoke_test` | see below |
| `contract_eject::eject_errors_when_provider_ships_neither_file` | unrelated; see below |

The three unload/load tests additionally asserted on log lines reading
`Successfully unloaded composable node 'fast_talker'`, where the actual log
says `'composable:/fast_talker'`.

**`autoware` is the same bug in a second place.** `rviz2` was already listed in
`ignored_exits` (`tests/tests/autoware.rs:202`), but
`HealthReport::is_healthy` did `ignored_exits.contains(&e.name.as_str())` — an
exact match against `node:/rviz2`. Every entry in every ignore list had been
dead since the rename. The `rviz2` exit itself is environmental (no `DISPLAY`);
the mechanism meant to tolerate it was broken.

**`contract_eject` is unrelated to the rename.** `play_launch contract eject`
was moved into the extracted `ros-launch-resolve` CLI by `adc33a7`
("depend on ros-launch-resolve; drop the resolve pipeline", RFC-0060 W3).
`play_launch` has no `contract` subcommand at all now
(`unrecognized subcommand 'contract'`, `tip: a similar subcommand exists:
'context'`). The test still invoked the old path, and `CLAUDE.md` still
documented it under `play_launch`.

## Resolution

- `parallel_loading.rs`: added `composable_api_path()` / `composable_id()`
  helpers and routed every API call and log assertion through them.
- `health.rs`: `is_healthy` now matches ignore entries against the bare name
  via `bare_member_name()`, which strips the `<kind>:/` prefix. Unit-tested.
- `contract_eject.rs`: points at the relocated `ros-launch-resolve` CLI,
  skipping cleanly if that binary is not built.
- `CLAUDE.md`: corrected to say `contract eject` lives in the
  `ros-launch-resolve` CLI.

`just test-all`: **108/108 integration tests pass**, from 99/105. Suite runtime
dropped 200s → 81s, since the four `parallel_loading` tests no longer time out
and retry.

## Lesson

A rename that changes an identifier's *format* does not break compilation when
the identifier is a string, so the type system gives no warning. The two ignore
mechanisms here (`ignored_exits`, and the retry) both silently stopped working
and made the failures look flaky and environmental rather than stale. Worth a
grep for hand-written member names when member ID formats change again.
