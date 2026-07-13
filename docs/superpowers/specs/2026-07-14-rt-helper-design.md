# Phase 38.10 — Privileged RT Helper (non-root RT scheduling)

**Date:** 2026-07-14
**Status:** Approved (design), pending implementation
**Repo:** `play_launch` (Linux)
**Supersedes:** the "grant `CAP_SYS_NICE` to the main binary" approach, which is
impossible (see §1).

## Goal

Make `--sched` apply real-time scheduling **without running any part of the ROS
stack as root**. Today the only way to get RT is `sudo -E play_launch ...`, which
makes play_launch *and every node it spawns* run as root — an unacceptable trade
for a scheduling knob.

## 1. Why the main binary cannot simply be given `CAP_SYS_NICE`

Granting a file capability to an ELF makes the kernel execute it in
**secure-execution mode** (`AT_SECURE=1`). The dynamic linker then ignores
`LD_LIBRARY_PATH` (and `LD_PRELOAD`).

The main `play_launch` binary links **~22 ROS shared libraries** that are found
*only* via `LD_LIBRARY_PATH` (exported by ROS's `setup.bash`), and it carries no
`DT_RUNPATH`. So `setcap cap_sys_nice+ep play_launch` makes it fail at startup:

```
error while loading shared libraries: libcomposition_interfaces__rosidl_typesupport_c.so
```

This was verified empirically (and shipped by mistake once; see the
`fix(setcap): never capability-grant the main binary` commit). It is also
*exactly why* `play_launch_io_helper` already exists: it links **zero** ROS
libraries, so it is safe to hold a capability.

**Principle:** keep capabilities off the main functional binary. Put each
privileged operation in a small, ROS-free helper that holds exactly the one
capability it needs.

## 2. Architecture — one capability per helper (least privilege)

```
play_launch  (uncapped, ROS, unprivileged — normal LD_LIBRARY_PATH)
    │                                  │
    │ ApplySched{pid, tier}            │ ReadProcIo{pids}
    ▼                                  ▼
play_launch_rt_helper              play_launch_io_helper
  CAP_SYS_NICE only                  CAP_SYS_PTRACE only
  sched_setscheduler/affinity        /proc/[pid]/io
```

A **separate** RT helper (rather than adding `CAP_SYS_NICE` to the io_helper)
so capabilities never pool on one binary: a monitoring-only run carries no RT
privilege, and an RT-only run carries no ptrace privilege. Independent
lifetimes, independent failure domains.

File capabilities do **not** change UID — the helper runs as the invoking user,
merely with `CAP_SYS_NICE` in its effective set.

## 3. The Linux mechanics

### 3.1 Spawn → apply sequence

1. play_launch `fork`+`execve`s the node (`pre_exec` already sets
   `PR_SET_PDEATHSIG` and `setpgid`). `child.id()` yields the PID — which is the
   **TGID** and the TID of the main thread. At this instant the child has
   **exactly one thread**.
2. play_launch sends `ApplySched{pid, tier}` to the RT helper over its pipe.
3. The helper (holding `CAP_SYS_NICE`) issues, **for every TID** (see §3.3):
   ```c
   sched_setscheduler(tid, SCHED_FIFO, &(struct sched_param){ .sched_priority = 20 });
   sched_setaffinity(tid, sizeof(cpu_set_t), &mask);
   ```

### 3.2 Permission model — what the capability actually buys

- `RLIMIT_RTPRIO` is **0** for a normal user (verified: `ulimit -r` → 0), so an
  unprivileged process gets `EPERM` on *any* RT request.
- `CAP_SYS_NICE` **bypasses `RLIMIT_RTPRIO`** and satisfies the kernel's
  act-on-target check (`check_same_owner(p) || capable(CAP_SYS_NICE)`).
- Same-user already holds here, so the capability is doing exactly one job:
  lifting the RT-priority limit. This is the minimum privilege that works.

### 3.3 Scheduling attributes are PER-THREAD — the load-bearing detail

`sched_setscheduler(pid, …)` sets the policy of the **thread whose TID == pid**
(the main thread). Sibling threads are **not** touched. Measured thread topology
of a real ROS node (`demo_nodes_cpp talker`):

| time after `execve` | threads |
|---|---|
| +0.02s | 1 |
| +0.1s  | 1 |
| +0.5s  | **11** (DDS/rmw spawn ~10) |

Threads created *after* a policy change **inherit** it — glibc's default is
`PTHREAD_INHERIT_SCHED`. Verified empirically with a `SCHED_BATCH` probe
(privilege-free stand-in; the inheritance mechanism is policy-independent):

```
main BEFORE setsched   policy=0 (SCHED_OTHER)
main AFTER  setsched   policy=3 (SCHED_BATCH)
thread spawned AFTER   policy=3 (SCHED_BATCH)   <-- inherited
```

Consequences:

- **Regular nodes / containers** — applied at ~+1ms, when only the main thread
  exists. The ~10 DDS threads spawn later and inherit FIFO. The whole process
  ends up RT. ✅
- **Composable nodes** — applied on the `ComponentEvent::LOADED` event, i.e.
  *after* `component_node` has initialized, so **~11 threads already exist**.
  Setting only the TGID leaves the 10 DDS/executor threads at `SCHED_OTHER` —
  and those are the threads doing the actual work. ❌ **This is a real defect in
  the currently-merged 38.9 composable path.**

**Therefore the helper applies per-TID:** enumerate `/proc/<pid>/task/*` and
`sched_setscheduler(tid, …)` for each. Idempotent; correct for both the
1-thread and the 11-thread case; threads created afterwards inherit from an
already-RT creator.

Residual caveat: a library that creates threads with `PTHREAD_EXPLICIT_SCHED`
opts out of inheritance. The per-TID sweep covers everything existing at apply
time; anything created later with explicit attrs would be missed. The
acceptance test (§7) checks **all** TIDs, so this would be caught empirically.

### 3.4 Races and safety

- **PID reuse: impossible.** play_launch holds the `Child` and has not
  `wait()`ed, so a dead child remains a zombie and its PID stays reserved. If it
  died before the helper acted, `sched_setscheduler` returns `ESRCH` — the
  helper can never hit an unrelated process. Same reasoning for composables (the
  C++ container holds the child's `pidfd`).
- **Window:** between `execve` and the apply the node runs `SCHED_OTHER` for
  ~1ms — it is still in dynamic linking / `main()`. Harmless.

### 3.5 Kernel gotchas (measured on the dev host)

- **RT throttling:** `sched_rt_runtime_us = 950000` / `sched_rt_period_us =
  1000000` → RT tasks are capped at 95% of each period. Kernel anti-lockup
  valve; a runaway FIFO task cannot wedge the machine.
- **cgroup RT bandwidth:** `CONFIG_RT_GROUP_SCHED` is **not** enabled here
  (cgroup v2), so there is no per-cgroup `rt_runtime` gate. On kernels that *do*
  enable it, a systemd user slice with `rt_runtime=0` makes `sched_setscheduler`
  fail with `EPERM` **even with `CAP_SYS_NICE`**. The preflight should detect
  this and say so, rather than reporting a bare permission error.
- **Priority inversion:** an RT node sharing a mutex with a non-RT one can
  invert. Out of scope; it is part of why tiers exist.

### 3.6 Verification surfaces

- `/proc/<tid>/stat` — field **41** = policy (`0`=OTHER, `1`=FIFO, `2`=RR),
  field **40** = `rt_priority`
- `/proc/<pid>/task/` — the TID list (**the one that matters**)
- `/proc/<pid>/status` — `Cpus_allowed_list`, `CapEff`
- userspace: `chrt -p <tid>`, `taskset -cp <tid>`

Note: `chrt -p <pid>` reports the **main thread only**. Any verification that
checks just the PID proves less than it appears to.

## 4. Components

### 4.1 Lib split (prerequisite)

`play_launch`'s lib currently exposes only `pub mod ipc`; `sched_apply` lives in
the *binary* module tree, so a `src/bin/*` helper cannot reach it.

- New lib module `pub mod sched` — `SchedPolicy`, `AppliedTier`,
  `SchedApplyError`, `apply_tier(pid, &AppliedTier)` (per-TID),
  `has_sched_privilege()`. **serde-derived** (they cross the wire), **no clap**
  (so the helper stays small and ROS-free).
- `execution/sched_apply.rs` keeps `SchedApplyMode` (the `clap::ValueEnum`) and
  re-exports the rest → no churn at existing call sites.

### 4.2 Protocol + binary

- `ipc::sched_protocol` — its **own** `Request`/`Response` pair, reusing the
  existing `encode_message`/`decode_message` (length-prefixed bincode) framing.
  Separate from the io_helper's enums so the two independently-capped binaries
  share no wire-compat entanglement.
- New `[[bin]] play_launch_rt_helper` — reads `--request-fd` / `--response-fd`,
  runs the same request loop shape as the io_helper, dispatches `ApplySched` →
  `apply_tier`. **Must be `ldd`-asserted ROS-free.**

### 4.3 Client + handle

`RtHelperClient` mirrors `IoHelperClient` (pipe pair, spawn, ping, shutdown).
Its pipes are **ordering-correlated** (no request IDs), so it cannot be shared
by `&mut`. A single **owner task** owns the client and serializes traffic;
actors hold:

```rust
#[derive(Clone, Debug)]
pub struct SchedHelper(mpsc::Sender<SchedJob>);   // Sender is Clone + Debug
```

`Clone + Debug` matters: `ActorConfig` derives `Debug`, and
`Arc<tokio::Mutex<…>>` is not `Debug`.

### 4.4 Lifetime (explicit)

- child: `PR_SET_PDEATHSIG` → parent dies, helper dies. **No orphan.**
- parent: `kill_on_drop(true)` + `Drop` → reaped on panic/unwind. **No zombie.**
- graceful: `Shutdown` → ack → `wait()` with timeout → escalate to kill.
- **request timeout** on every round-trip → a wedged or dead helper degrades per
  `--sched-apply` (warn/strict); it never hangs the launch.
- spawned **only** when `--sched` is used, with a lifetime independent of
  monitoring's.

### 4.5 Apply sites

`ActorConfig` gains `sched_helper: Option<SchedHelper>`. The three existing
sites — regular node (post-spawn), container (`handle_running`), composable
(`LOADED`) — become: **helper if present**, else direct `apply_tier` when running
as root (keeps `sudo -E` working as a fallback). All three are already `async`.

### 4.6 Capabilities

- `setcap`: `cap_sys_nice+ep` → **rt_helper**; `cap_sys_ptrace+ep` → **io_helper**;
  **nothing** on the main binary (the existing self-heal that strips a stray cap
  stays).
- `verify`: per-binary, per-capability `contains` checks (an exact-match breaks
  as soon as a binary carries more than one cap).
- preflight: privileged iff *running as root* **or** *rt_helper carries
  `cap_sys_nice`*.

## 5. IPC cost

RT dispatch is a **control-plane** operation, not data-plane. `sched_setscheduler`
runs **once per process** (spawn, respawn, composable LOAD). Autoware ≈ 120 nodes
+ 64 composables ≈ **~200 round-trips, once, at startup** — each a ~30-byte
bincode message over a pipe. After that the kernel does the scheduling:
**steady-state IPC cost is zero.** (The io_helper's existing monitoring use is
*periodic* and already accepted, so RT is strictly cheaper than what the helper
pattern already pays for.)

## 6. Non-goals

- `SCHED_DEADLINE` (still deferred).
- Threads created after apply with `PTHREAD_EXPLICIT_SCHED` (see §3.3 caveat).
- cgroup `rt_runtime` provisioning — detect and report, don't manage.

## 7. Acceptance test — the ship criterion

`just setcap`, then run **unprivileged** with `--sched`:

- every node's **every TID** shows `policy=SCHED_FIFO`, `rt_priority=20`,
  `Cpus_allowed_list=0`
- **no process in the tree runs as root**

Checking only the PID (`chrt -p <pid>`) is insufficient — it reads the main
thread. The test enumerates `/proc/<pid>/task/*`.

## 8. Waves

1. **W1** — lib `sched` split (+ per-TID `apply_tier`). No behavior change; existing tests green.
2. **W2** — `ipc::sched_protocol` + `play_launch_rt_helper` binary (+ build/bundle wiring, `ldd` ROS-free assertion).
3. **W3** — `RtHelperClient`, owner task, `SchedHelper` handle, lifetime, spawn wiring, reroute the 3 apply sites (helper-first, root fallback).
4. **W4** — capabilities (`setcap`/`verify`/preflight), docs.
5. **W5** — acceptance: unprivileged per-TID RT proof.
