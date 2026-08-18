---
id: 15
title: "A rebuild drops helper capabilities, and every message blamed the user for not running setcap"
status: resolved
type: build-friction
severity: low
---

# 0015 — Rebuilding disarms real-time scheduling, silently

**Repo:** `play_launch`
**Affects:** `justfile` (`build`, `build-rust`, `setcap`), any RT workflow
**Found:** phase-57, 2026-08-05

## Summary

A file capability lives on the inode. `just build` / `just build-rust` replace
`install/play_launch/lib/play_launch/play_launch_rt_helper`, so the
`cap_sys_nice=ep` granted by `just setcap` is gone the moment the binary is
rebuilt.

This is **documented** — `docs/guide/rt-scheduling.md` says "Capabilities live
on the file inode: re-run `just setcap` after every rebuild", and its
troubleshooting table has the exact symptom ("worked yesterday, EPERM today").
The gap is not knowledge, it is *timing*: nothing says it at the moment of
loss, so the reader has to have retained a line from a 650-line guide across
however many builds. (The guide's quick-start also called `setcap` "one-time",
contradicting that note 480 lines later; corrected 2026-08-06.)

Observed during phase-57. The capability was granted, the experiment ran, a
source change prompted a rebuild, and the next run produced:

```
Error: scheduling: no privilege to apply; run `play_launch setcap`
  (grants cap_sys_nice to play_launch_rt_helper) or run as root
```

```console
$ getcap install/play_launch/lib/play_launch/play_launch_rt_helper
$          # empty — the capability the user granted 20 minutes earlier
```

## Why it matters

`setcap` needs a password, so on any machine where the developer is not root
the loop is: build → run → discover RT is off → ask for a password → re-run.
Each rebuild costs a round trip, and the failure appears at *run* time, far
from the *build* that caused it.

The severity is bounded by existing guards rather than by luck:

- `--sched-apply strict` errors clearly (quoted above), so a privileged run
  never silently degrades into an unprivileged one.
- `examples/rt_av_demo/justfile`'s `ab` recipe checks `getcap` and refuses,
  because an RT-on half that applies nothing would make both halves of the A/B
  identical and the comparison vacuous.

So this is friction, not a correctness hole. It is filed because the friction
is invisible at the moment it is introduced.


## Resolution

**The capability loss is not the defect and was not "fixed".** A file
capability is bound to the exact contents of a binary; replacing the binary
must invalidate it, or privilege would carry across to code nobody vetted.
Ruling on this, 2026-08-18: keep the behaviour, keep the Docker path as an
INTERNAL developer convenience, do not recommend it to users, and fix the
reporting.

### What was actually wrong

Every message said the same thing regardless of cause:

    scheduling: no privilege to apply; run `play_launch setcap`

That is correct for a fresh install and misleading for the common developer
case, where `setcap` WAS run and the helper has been rebuilt since. Such a
user is told they did not do a thing they did do, so the natural reading is
that `setcap` failed. It did not.

### The fix

`play_launch setcap` now records what it granted — path, SHA-256 of the
binary's contents, timestamp — under `$XDG_STATE_HOME/play_launch/`
(`~/.local/state/play_launch/setcap-grants.tsv`). The runtime compares, and
`commands/cap_status.rs` reports one of four states:

- **NeverGranted** — no record for this path.
- **BinaryChanged** — granted, and the contents differ now. Says so, says
  when, and says it is expected.
- **Revoked** — granted, contents identical, capability gone anyway.
  Something removed it: `setcap -r`, a mount without xattr support, a copy
  over the same path.
- **Unknown** — `getcap` unavailable. Reported as unknown, NOT as missing,
  because "cannot check" and "not there" call for different actions.

The hash is what makes it honest: an mtime or inode number would call a
touched-but-identical file changed, and would miss a rebuild that reused the
inode. Recording is best-effort — failing to record must never fail the grant.

Result, on the scheduling path under `--sched-apply strict`:

    Error: scheduling: no privilege to apply.
      …/play_launch_rt_helper lacks cap_sys_nice — real-time scheduling cannot be applied.
      It WAS granted (2026-08-18 09:00:00), but the binary has been replaced since — a
      rebuild, upgrade or reinstall. A file capability is bound to the exact file contents
      and cannot survive that, by design: carrying it across would grant privilege to code
      that was never vetted.
      This is expected after every rebuild; run `play_launch setcap` to grant cap_sys_nice+ep
      to …/play_launch_rt_helper
      Alternatively run as root.

`run` and `up` share one `rt_privilege_report()` so they cannot drift; the I/O
helper's three separate ad-hoc messages collapse onto the same `explain()`.

### Docs: the supported path is `play_launch setcap`

`README.md` and `docs/guide/rt-scheduling.md` led with `just setcap`, which a
`pip install` user does not have — there is no justfile. They now lead with
`play_launch setcap` and mark the recipe a developer shortcut, "not for
deployments". A test asserts that no runtime message mentions `just setcap` or
`docker`: the container path rests on `docker` group membership being
root-equivalent and is refused under rootless Docker, so it must never be
advice given to a user.

### Verified

All four states exercised against the real binaries on this checkout —
never-granted, rebuilt (stale hash), revoked (matching hash, no capability) —
on both the I/O and scheduling paths, plus 4 unit tests including one that
fails if a message ever recommends the Docker shortcut. `just test`: 470 +
241 + 176 + 86, zero failures.

## Possible fixes (as filed)

1. `just build` re-applies the capability if it was present before the build.
   Needs the password again — but at build time, where the user is already
   waiting, rather than mid-experiment.
2. `just build` detects that it dropped a capability and prints a warning
   naming `just setcap`. No password, no automation, just tells the truth at
   the point of loss.
3. A `just check-caps` recipe, and have RT-relevant recipes depend on it.

(2) is the smallest honest fix: it requires recording that the helper *had*
the capability before the build, which is one `getcap` call.

## Not this issue

That `setcap` is needed at all is by design — the unprivileged main process
delegates to a privileged helper rather than running as root. See
`docs/guide/rt-scheduling.md`.
