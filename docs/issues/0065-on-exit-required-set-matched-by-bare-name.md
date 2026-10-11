---
id: 65
title: "The `on_exit=Shutdown()` set is matched by bare name, so an unrelated process with the same name or stem shuts the launch down"
status: resolved
type: defect
severity: medium
---

# 0065 - The required-process set is matched by bare name

**Repo:** `play_launch` at `785cddb7`
**Affects:** `src/play_launch/src/commands/up.rs:1112-1117`,
`src/play_launch/src/member_actor/coordinator/builder.rs:61`
(`declares_shutdown_on_exit`) and `:274`
**Found:** review of the commits pulled 2026-10-11, confirmed by reading; not
yet reproduced.

## What happens

Two ways an exit that should not end the launch does:

1. **Same stem.** Two unnamed `ExecuteProcess(cmd=['sleep', ...])`, only the
   second with `on_exit=Shutdown()`. The dump keys both by the stripped exec
   name `sleep`. The members are `sleep-1` and `sleep-2`, and
   `declares_shutdown_on_exit` strips the `-N` counter, so both match `sleep`.
   The bystander exiting tears the whole launch down.
2. **Same name, different namespace.** `/left/controller` and
   `/right/controller`, only one required. The set holds `controller` with no
   namespace, so either one exiting ends the launch.

The reverse can't happen: a required process is never missed. So this fails
by ending a launch too early, never by keeping one running.

## Why

The set is built from `record.name.or(record.exec_name)` (`up.rs:1116`), which
is neither unique nor the member's identity. The stem fallback exists because
the dump strips the counter (`talker-1` becomes `talker`) while the model keeps
it. It reconciles two spellings of one name by throwing away the part that
told instances apart.

## Fix

Key the set by the identity the member actors already use: the model FQN,
including the `-N` ordinal for unnamed processes (#0017). The cleanest version
falls out of #0063: once the model carries the flag per instance, the builder
reads it from the instance it is building, and no name matching is left. Until
then, match on namespace plus name and drop the stem fallback in favour of the
ordinal.

## Verification, when fixed

Two `on_exit_shutdown.rs` cases:
- Two unnamed `sleep` processes, the bystander exiting first (`sleep 1` beside
  a required `sleep 3`). The launch must last at least 3 s.
- Two same-named nodes in different namespaces, with the same timing check.

## Resolution (2026-10-11)

There is no name matching left. With the flag on the model (#0063), each
member's own record says whether it is required. The builder adds that
member's canonical id (the one its actor reports in `StateEvent::Exited`) to
the required set. `declares_shutdown_on_exit`, its `-N` stem fallback, the
builder's name set and `set_shutdown_on_exit` are deleted.

Test (`on_exit_shutdown.rs`): `a_same_stem_bystander_does_not_end_the_launch`,
two unnamed `sleep`s with only the 3 s one required. The launch must outlast
the 1 s one. Negative control: with the old matching restored, it ends at
about 1 s and fails.
