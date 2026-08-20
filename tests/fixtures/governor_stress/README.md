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

## What `ab-cpu` shows, and what it cannot

Both gates that ship OFF do pace when enabled: `max_concurrent` costs 3.0x
startup for 39% off peak runnable, `max_runnable_factor` 2.0x for 30%.

Two caveats before quoting those numbers:

- **This fixture flatters the gates.** `cpu_burn` is pure CPU, so holding it
  back is maximally effective. A real node spends much of its startup blocked
  on discovery and I/O, where the delay costs full price and saves nothing.
  W1 measured ~10% on Autoware; treat 30-39% as an upper bound.
- **It cannot reproduce the runnable-ceiling pathology** W1 found (that the
  ceiling is *worse* than no gate, because it cannot tell "busy starting" from
  "busy running"). These burners settle after 8 s and go idle, so the gate
  never gets stuck.

`cpu_sustained.launch.xml` is the arm that does reproduce it — the same nodes
with `sustain_duty_pct` keeping them busy afterwards:

    just arm cpu_sustained gate_off
    just arm cpu_sustained runnable_on

At 50% duty the ceiling costs 7x the spawn time for a HIGHER peak runnable
count than no gate (77 vs 69); at 90% it costs 44x and stalls five admissions
to the bypass. That is W1's finding, reproduced.

## Seeing the memory gate actually fire

    just ab-mem-preload          # holds 10 GiB outside play_launch first

This is the only configuration in which the floor fires, for the reason above:
it can only see pressure that already exists. Expect all 24 admissions held to
the bypass.
