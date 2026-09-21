---
id: 36
title: "help text and guides still describe the phase-38 apply layer and never mention the runtime monitor"
status: open
type: tech-debt
severity: low
---

# 0036 - what `--help` and `docs/guide` say about scheduling and enforcement is one or two phases behind the code

**Repo:** `play_launch` (0.11.0, `5eaa3191`)
**Affects:** `src/play_launch/src/cli/options.rs:117`; `docs/guide/rt-scheduling.md:671`, `:748`;
`docs/superpowers/specs/2026-07-01-shared-scheduling-crate-design.md:3`, `:206-208`;
`docs/superpowers/specs/2026-07-06-linux-sched-apply-layer-design.md:4`, `:28`, `:35`, `:94`;
`docs/superpowers/specs/2026-07-16-rt-config-v2-design.md:4`, `:119`;
`docs/roadmap/README.md:108`; `docs/guide/` (no runtime-monitor guide)

## Symptom

Each of these was read against the 0.11.0 source on 2026-09-21. Verbatim
text, then what the code does.

1. `play_launch setcap --help` (`cli/options.rs:114-117`):

   > Grant CAP_SYS_PTRACE to the I/O helper (for per-process I/O
   > monitoring). Requires sudo. NOTE: the main binary is deliberately NOT
   > capped ... RT scheduling (`--sched`) needs root.

   RT scheduling does not need root. `play_launch setcap` itself grants
   `cap_sys_nice+ep` to `play_launch_rt_helper` (`commands/capabilities.rs:4`,
   `:64-68`), and `execution/rt_helper_client.rs` routes every apply through
   that helper; root is only the fallback when no helper is capped. The
   sentence contradicts the verb it is attached to.

2. `docs/guide/rt-scheduling.md:671`, section 2.4 "Privilege: the RT helper":

   ```
   play_launch_rt_helper  (ROS-free, holds CAP_SYS_NICE only)
       -> sched_setscheduler / sched_setaffinity on every thread of that pid
   ```

   `src/play_launch/src/sched.rs:11-13` says, and does, the opposite: "The
   policy syscall is `sched_setattr(2)`, not `sched_setscheduler(2)`",
   because only the former expresses `SCHED_DEADLINE`, uclamp and
   `SCHED_FLAG_RESET_ON_FORK`.

3. `docs/guide/rt-scheduling.md:748`, section 2.5 "What the kernel ends up
   with":

   > - `SCHED_DEADLINE` is not applied on Linux yet.

   Phase 60 (`docs/roadmap/phase-60-linux-sched-surface.md:3`, "complete
   (2026-08-12) - W1-W8 all landed") added `SchedPolicy::Deadline` and
   `Reservation { runtime, deadline, period }` to the apply layer
   (`sched.rs`, the `SCHED_ATTR_SIZE_VER1` struct). The guide's section 1
   even documents `reservations: required`.

4. `--enforce-rules` (the runtime monitor, phase 36) has no user guide.
   `grep -rl enforce-rules docs` returns `docs/roadmap/phase-36-runtime_enforcement.md`,
   `docs/roadmap/phase-37-crate_split.md` and `docs/roadmap/README.md`
   only. `docs/guide/` has seven files and none covers interception,
   `runtime_violations.jsonl`, the rule table, the `interception.enabled`
   precondition (#0031) or what strict does (#0032, #0033). The design of
   record (`docs/design/fault-reaction-primitives.md`) and phase 73's
   observer are similarly reachable only through the roadmap.

5. `docs/superpowers/specs/2026-07-01-shared-scheduling-crate-design.md`
   status line (`:3`) says "Implemented (Linux side) + merged to main" and
   `:206-208` describes phase 2 as applying `sched_setscheduler`; it is
   built on `record.json` (retired in phase 47) and TOML tiers (now the
   legacy `manual` bridge). The 07-16 spec's own header marks it
   "Supersedes-in-part" but the 07-01 file does not point forward.

6. `docs/superpowers/specs/2026-07-06-linux-sched-apply-layer-design.md`
   status (`:4`) says "Implemented (38.1-38.8) + composable scheduling
   (38.9)" and the body still states (`:28`) "Mechanisms:
   `sched_setscheduler(SCHED_FIFO|SCHED_RR, priority)`", (`:35`) "No
   SCHED_DEADLINE / `sched_setattr`", (`:94`) "`Fifo`/`Rr` ->
   `libc::sched_setscheduler(...)`". All three were reversed by phase 60
   and the 2026-08-10 feature-surface spec, which the 07-06 file does not
   name.

7. `docs/superpowers/specs/2026-07-16-rt-config-v2-design.md:4` is still
   "Status: Approved (design), pending implementation" (it shipped) and
   `:119` says "launch/replay resolve the platform file" (`replay` became
   `up` in 0.9.0; `cli-migration-0.9.md` covers the rename).

## Impact

Someone setting up RT scheduling from `--help` will `sudo` when they should
`play_launch setcap`; someone reading the guide will believe DEADLINE
reservations are unimplemented and not try them; someone asked "does
play_launch enforce the contract at runtime" has no page to point at, only
a roadmap entry, and cannot discover #0031's precondition from the docs.
Brief C section 1.5 is a table of these; the safety-island deck had to
cite source lines instead of the guide.

## Fix direction

- `options.rs:117`: replace the last sentence with "RT scheduling
  (`--sched`) needs `play_launch setcap` to have capped the RT helper, not
  root."
- `rt-scheduling.md:671`: `sched_setattr / sched_setaffinity`; `:748`:
  delete, or replace with a pointer to section 1's `reservations:` and the
  cpuset precondition `execution/cpuset.rs` checks.
- Add `docs/guide/runtime-enforcement.md`: mechanism (LD_PRELOAD, SPSC
  ring), the `interception.enabled` precondition, the mode table
  (`off|warn|strict|record-only`), the rule-id table from
  `runtime_enforcement/mod.rs` with the contract field each reads, the
  JSONL shape, and what `hazard-*` observe. Brief C section 3 is a draft.
- Stamp the 07-01 and 07-06 specs "Superseded by 2026-07-16 (authoring
  model) and 2026-08-10 (apply mechanisms)" in their status lines rather
  than editing their bodies; flip 07-16 to "Implemented" and `replay` to
  `up`.
- A doc test in the spirit of #0015's (which greps runtime messages for a
  forbidden recommendation): grep the guide and `--help` for
  `sched_setscheduler` and "needs root".

## Provenance

Brief C (`brief-C-playlaunch-usage.md`, sections 1.1 and 1.5), 2026-09-18;
every line re-read in the 0.11.0 worktree at `5eaa3191` on 2026-09-21.
