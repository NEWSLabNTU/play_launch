---
id: 15
title: "just build silently drops CAP_SYS_NICE from play_launch_rt_helper, disabling RT until setcap is re-run"
status: open
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

## Possible fixes

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
