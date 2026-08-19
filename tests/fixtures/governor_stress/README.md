# governor_stress

Stress fixture for play_launch's startup governor. Results and analysis live in
`docs/roadmap/phase-61-edge-startup-storm.md` § W3 — read that first; this file
is only how to run it.

## Safety

This fixture deliberately consumes RAM. Five independent guards, all verified
firing (see the roadmap):

1. `mem_hog` stops short of `safety_reserve_mb` (8 GiB default)
2. `max_lifetime_secs` (180 s) releases and exits any orphan
3. a `systemd-run --user --scope` with `MemoryMax` at half of RAM — the only
   guard the KERNEL enforces at allocation time. **The harness refuses to run
   without it**; `GOVERNOR_STRESS_NO_CGROUP=1` overrides, knowingly.
4. `scripts/edge-storm/oom_guard.sh` on the process group at a 6 GiB floor
5. `timeout` + an EXIT trap that kills the group

Kill switch, safe to run any time, from any shell:

    just abort

## Running

    just build
    just baseline        # ALWAYS read this first — floors must come from it
    just ab-mem          # gate_off vs gate_on
    just ab-cpu          # re-check the two gates that ship OFF
    just check-oom-bias  # +300 on children, read from /proc

## The trap this fixture fell into

`just ab-mem` as configured shows **no gating**, and that is not a bug in the
governor. Admission is far faster than allocation — 24 processes admitted in
0.32–0.70 s, each taking ~1421 ms to become resident — so every admission
happens before any memory moves. A floor cannot gate on pressure that does not
exist yet.

To see the gate work, the pressure has to PRE-EXIST the launch:

    # hold 10 GiB outside play_launch, then set a floor above what remains
    ros2 run governor_stress mem_hog --ros-args \
      -p alloc_mb:=10240 -p hold_secs:=200 -p max_lifetime_secs:=210 &
    just arm-auto mem_wave -4000

`arm-auto` computes the floor from live `MemAvailable`; a negative headroom puts
it above, which is what makes the gate bind.
