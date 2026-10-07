---
id: 59
title: "The cross-scope `dangling-entity` loop never covered actions, so the per-manifest Error it replaced was deleted and nothing re-emitted it"
status: resolved
type: defect
severity: medium
---

# 0059 - an action nobody serves is reported by nothing

**Repo:** `play_launch` at `873b0744`
**Affects:** `src/ros-launch-resolve/resolve/src/ros/manifest_loader.rs`
(`run_cross_scope_checks`, and the suppression in `load_manifests`)
**Found:** reviewing the three items left unfiled after the bugfix campaign.

## What happens

A contract declaring an action with a `client:` and no `server:` — anywhere in
the merged tree, and with no `external: server` to say the server is another
image — is accepted. `check` prints `1 clean, 0 with errors` and exits 0.

```
$ ros-launch-resolve check tests/fixtures/contract_action_unserved/launch/bringup.launch.xml
Parsed: 1 scopes, 3 nodes, 0 containers, 0 composable nodes
1 manifest(s) checked: 1 clean, 0 with errors (0 errors, 0 warnings)
```

`ros-launch-manifest` disagrees. Its own `dangling-entity` rule
(`check/src/rules/dangling_entity.rs`) raises an **Error** for exactly this
shape, in the same loop that raises one for a service:

```rust
// Actions
for (name, act) in &manifest.actions {
    if act.server.is_empty() && !act.client.is_empty() && !server_is_external(act.external) {
        ctx.error(self.id(), &format!("actions.{name}"),
            format!("action '{name}' has no server (goals can't be processed)"));
    }
}
```

## Why it is silent

Not a missing feature — a **deleted check**. `load_manifests` drops every
per-manifest `dangling-entity` diagnostic unconditionally:

```rust
check_result
    .diagnostics
    .retain(|d| d.rule_id != "dangling-entity" && d.rule_id != "service-wiring");
```

The reasoning in the comment above it is sound: in cross-scope mode the merged
index is the authoritative source, and per-manifest emission produces O(n)
duplicate warnings for an entry legitimately served in another scope. The
replacement, `run_cross_scope_checks`, then iterates `index.topics` and
`index.services` — and stops. `index.actions` is populated (it has been since
R1-P2, whose own doc comment records that the loader used to drop `actions:`
entirely and `structure.actions` was always empty), and no loop reads it.

So for actions alone the suppression removed a real Error and nothing
re-emitted it. The two halves are each defensible in isolation, which is why
this survived: the drop looks like deduplication and the cross-scope loop looks
complete. Same family as the rest of this campaign — **the defect arrived with
something that blessed it**, here a comment asserting an authoritative
replacement that was authoritative for two entity kinds out of three.

Worth noting what it is *not*: `service-wiring`, the other suppressed rule, is
driven off a node's `cli:` endpoints and has no action equivalent upstream, so
nothing is missing there.

## Fix

A third loop in `run_cross_scope_checks`, mirroring the service one exactly —
same severity (Error), same `external: server` skip, same `scope_ids` count in
the message:

```
error[dangling-entity]: action '/nav/navigate_to_pose' has 0 servers across the
manifest tree (declared in 1 scope(s)) -- goals can't be processed
```

## Verification

Negative control first, and it caught a second problem on the way: the first
run was against `src/ros-launch-resolve/target/debug/ros-launch-resolve`, a
binary dated a month earlier. The colcon-generated `.cargo/config.toml`
redirects the target directory to `build/.cargo_target/play_launch/`, so
`cargo build` writes there and the path under the workspace is a stale
leftover. A debug `eprintln!` that never printed is what exposed it — issue
#0020's shape again (testing an artifact that is not the one just built), and
the reason the control is run as a control rather than inferred.

With that sorted: a fresh build *without* the fix reports `1 clean, 0 with
errors`, exit 0; with it, one Error, exit non-zero.

Fixture `tests/fixtures/contract_action_unserved` carries **three** actions so
the rule is falsifiable in both directions — a loop that reported every action
holding a client would pass a test asserting only the first row:

| action | shape | expected |
| --- | --- | --- |
| `/nav/navigate_to_pose` | client, no server, not external | **Error** |
| `/nav/dock` | client, server in this tree | clean |
| `/nav/remote_recovery` | client, `external: server` | clean |

Observed: exactly one Error, naming `navigate_to_pose`. Test
`an_action_with_no_server_anywhere_is_reported` in `tests/tests/manifest_check.rs`
asserts the message, the absence of the other two, the count, and the summary
line.

The ranked-plan snapshot gained the new fixture and **no existing plan moved**,
which is the right outcome: a wiring diagnostic must not reach scheduling.
