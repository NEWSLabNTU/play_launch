# cgroup probes — phase 66

Every claim in `docs/design/cgroup-per-container.md` and
`docs/roadmap/phase-66-cgroup-per-container.md` was measured with one of these,
on the bench Orin (`5.15.148-tegra`, 12 cores, 61 GiB). They live here rather
than in `tmp/` because that directory is gitignored, and a design document whose
evidence cannot be re-run is a document asking to be believed.

Most need a writable cgroup subtree, which a login shell does not have:

```sh
systemd-run --user --scope -p Delegate=yes bash scripts/cgroup-probes/<probe>.sh
```

| probe | question it answers |
|---|---|
| `delegation_detect.sh` | Can this process own a cgroup subtree? Three contexts, and why the answer cannot be read off the path or the controller list. |
| `feature_inventory.sh` | Which interface files a delegated leaf actually exposes on this kernel. |
| `cgroup_probe.sh` | Which of them are writable, and whether PSI exists. |
| `mechanics_probe.sh` | Can a process join by writing `0`? Does a fork child inherit? Does `cgroup.kill` reach a grandchild? |
| `oom_group_probe.sh` | `memory.oom.group` — the bit that chooses between container semantics and fault isolation. |
| `accounting_probe.sh` | Summed per-process RSS vs the cgroup's own charge (the 2.4x over-count). |
| `threaded_probe.sh` | Why `cgroup.type=threaded` and the memory controller are mutually exclusive. |
| `freeze_probe.sh` | Does `cgroup.freeze` actually stop execution? |
| `freeze_discovery.sh` | What a freeze costs a node's DDS discovery — the W3 blocker. |
| `freeze_cpu.sh` | What freezing a *constructed* node returns. Needs `PL=<path to play_launch>` and a launch file argument. |

Two harness defects bit repeatedly while writing these, and are worth knowing
before adding another:

- **`grep -c` and `pgrep -c` already print `0` and exit non-zero** when nothing
  matches. A `|| echo 0` fallback therefore prints a *second* zero, and the
  result is a corrupted table or `[: 0\n0: integer expression expected`.
- **`ros2 node list` spawns a fresh participant** that must complete discovery
  from scratch, so it cannot answer "did the peer purge this node" — it
  conflates that with "my new observer has not found it yet". Ask an existing
  subscriber instead.
