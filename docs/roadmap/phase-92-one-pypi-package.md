# Phase 92 - one PyPI package: `pip install play_launch`, the distribution picked at run time

Status: **complete** — P1 done (2026-10-15), P2-P4 done (2026-10-16), P5 done (2026-10-17).
Follows the 0.17.0 release, whose PyPI upload failed and was yanked. 0.18.0
and 0.18.1 were tagged with the old workflow and their release runs
cancelled, so the first release of this design is 0.18.2.

## Why

0.17.0 (commit 978fe252) split the wheels into one PyPI project per ROS
distribution, `play_launch_<distro>`, behind a code-free `play_launch` meta
package installed as `pip install "play_launch[<distro>]"`. Two problems:

- **The upload failed.** The release workflow's trusted publisher cannot
  create the five new projects (`400 Non-user identities cannot create new
  projects`), so only the empty meta wheel reached PyPI and 0.17.0 had to
  be yanked. The wheels themselves built (GitHub release v0.17.0 holds all
  ten).
- **Installation got harder.** A plain `pip install play_launch` installed
  nothing useful; users must know and type their distribution.

The split exists because the binaries link one distribution's libraries
(rcl, rclcpp, message typesupport) and its libpython, and no wheel tag tells
Jazzy from Kilted or Lyrical from Rolling. But wheel tags do separate every
pair that matters for pip:

| Wheel tag | Distributions |
|---|---|
| cp310, manylinux_2_35 | humble |
| cp312, manylinux_2_39 | jazzy, kilted |
| cp314, manylinux_2_43 | lyrical, rolling |

(times x86_64 and aarch64). A wheel per tag that carries every
distribution sharing it, and picks one at run time, keeps one project and
one plain install.

## Goal

`pip install play_launch` installs a working tool on every supported
distribution and CPU; the command and the module stay `play_launch`; the
distribution is chosen when the tool runs; one PyPI project, published by
the existing trusted publisher.

## Design

- **One project, six wheels.** `play_launch-X.Y.Z-<pytag>-none-manylinux_<glibc>_<arch>.whl`
  for the three tags above and two CPUs. pip selects the wheel by Python,
  glibc and CPU as it does for any binary wheel.
- **Per-distribution payload inside the wheel.** Everything that links a
  distribution moves under `play_launch/dist/<distro>/` (the binary, the
  container plugin, the interception library, the Python-exec library and
  anything else the build finds linked to `/opt/ros/<distro>`); everything
  distribution-free (Python code, web assets, data) stays shared. The cp312
  and cp314 wheels carry two payloads each (about 2 x 14 MB; PyPI's file
  limit is 100 MB).
- **Selection at run time.** The `play_launch` entry point and the Python
  module resolve the payload once, in this order:
  1. `PLAY_LAUNCH_ROS_DISTRO` (an override for tests and odd setups);
  2. `ROS_DISTRO` (set by sourcing any ROS setup file);
  3. the only `/opt/ros/<d>` present that the wheel carries;
  4. otherwise an error naming what the wheel carries, what was found, and
     the fix (`source /opt/ros/<d>/setup.bash`).
  A distribution the wheel does not carry (for example kilted under a cp310
  Python) is an error that names the Python the distribution needs.
  The resolved payload's binary is exec'd; its library directory is added to
  `LD_LIBRARY_PATH` (or set as the binary's RPATH at build time, preferred,
  so the environment does not change for child processes).
- **Reporting.** `play_launch --version` names the payload in use
  (`play_launch 0.17.1 (humble payload, rlm v0.1.59)`); `play_launch
  doctor` (or its equivalent) reports the payloads the wheel carries and
  why one was chosen.
- **Build.** The release workflow keeps its per-distribution matrix (each
  row builds in its distribution's container, as now) and uploads each
  payload as an artifact; a new `assemble-wheels` job groups the rows by
  wheel tag and packs one wheel per tag (shared files once, payloads under
  `dist/<distro>/`), with a check that every row landed in exactly one
  wheel. The meta package, the extras and the `play_launch_<distro>`
  rename go away. `publish-pypi` uploads the six wheels to the existing
  `play_launch` project with the existing trusted publisher.
- **Source fallback.** An sdist is published too; building it needs Rust,
  CMake and a sourced ROS, and builds only the sourced distribution's
  payload. The README says so; wheels are the supported path.
- **Local builds.** `just build-wheel` keeps building the sourced
  distribution's payload into a one-payload wheel (same layout), so
  developers are unaffected.

## Units

### P1. Payload layout and the run-time selector

- Move the distribution-linked files under `play_launch/dist/<distro>/`;
  the entry point and module resolve them as above; RPATH set at build so
  no environment leaks to children.
- Gate: a one-payload local wheel installs and runs `play_launch --version`
  and a launch on the sourced distribution; with `ROS_DISTRO` unset and one
  `/opt/ros/<d>` it still runs; with a distribution the wheel lacks it fails
  with the documented message; tests for each selection branch.

**Done (2026-10-15).** `python/play_launch/payload.py` selects; both console
entry points (`cli.py`, `io_helper_cli.py`) run the selected payload's
binaries, and a wheel that carries payloads but none for the environment
is an error, never a fall-through to another `play_launch` on `PATH`.
`scripts/bundle_wheel.sh` writes `dist/$ROS_DISTRO/{bin,lib,share}` and
removes any other payload and the old flat `bin/lib/share` first.
`play_launch --version` names the distribution the binary was built for
(`play_launch 0.17.0 (ROS humble, v0.17.0-7-g027f6868, rlm 336d6c2)`; a
commit pin of rlm now prints its short hash instead of `unknown`). The
"doctor" is `python3 -m play_launch.payload`: what the wheel carries, what
is under `/opt/ros`, which payload this shell gets and why. It is not a
verb, so the binary's verb surface is unchanged. 12 tests in
`tests/python/test_payload.py`, run by `just test`.

Gate, on a humble host: the wheel ran `--version` sourced and with an empty
environment (branch 3: the binary's RPATH already names
`/opt/ros/humble/lib`), refused `ROS_DISTRO=jazzy` and
`PLAY_LAUNCH_ROS_DISTRO=kilted` naming Python 3.12, and ran
`composition composition_demo.launch.py` with the payload's
`component_container` loading 2/2 composables and exiting cleanly on
SIGINT.

**One departure from the design: the environment still carries the
payload.** RPATH was to replace `LD_LIBRARY_PATH` so nothing leaked to
children, but children need it. The container is found through
`AMENT_PREFIX_PATH`, and rosidl in the container (a process play_launch
does not link) finds play_launch_msgs' typesupport libraries through
`LD_LIBRARY_PATH`. An RPATH on our binaries serves neither lookup. Both
variables point at the selected payload only, as they pointed at the flat
`lib/` before.

### P2. Assemble multi-payload wheels in CI

- `assemble-wheels` job; the matrix rows upload payloads; one wheel per tag;
  completeness check; wheel contents listed in the job log.
- Gate: a workflow run on a branch (no publish) produces six wheels; each
  cp312/cp314 wheel holds two payloads; each installs in its distribution's
  container and passes a smoke test (`play_launch --version`, `play_launch
  check` on a fixture, one short `play_launch launch` of a demo node) for
  every payload it carries.

**Done (2026-10-16).** `scripts/assemble_wheels.py` merges the matrix
rows' one-payload wheels by tag (rows of one tag build wheels of the same
file name, so each is downloaded into its own directory), rewrites RECORD
and lists every wheel's contents in the job log. It refuses a row with no
payload or several, a distribution twice under one tag, a shared file that
differs between rows, and, with `--expect`, a distribution missing from or
unexpected in an architecture. `scripts/smoke_wheel.sh` is the smoke test:
`--version` with an empty environment (the only `/opt/ros/<d>`) and
sourced, `python3 -m play_launch.payload`'s choice, `check` on a
talker/listener file, and composition's demo launch file run to `Startup
complete` with 2/2 composables and a clean exit on SIGINT. The workflow's
`smoke-test` job runs it in a plain Ubuntu container per distribution and
CPU (QEMU for arm64) with only the distribution's `ros-core`, the demo
packages and the wheel installed; `release` and `publish-pypi` need it.

Result: six wheels, 20-46 MB (the cp312 and cp314 wheels carry two
payloads), every one of the ten (distribution, CPU) smoke tests passing.
Getting there found three defects the old pipeline shipped, none of which
any earlier check could see, because nothing ran a wheel outside the
container that built it:

- **libz3 was not in the wheel.** `play_launch` links it through the
  manifest checker's `smt` feature; Humble's `ros-core` happens to pull in
  `libz3-4` and nothing later does, so every Jazzy-and-later wheel, 0.17.0's
  included, failed to start (`libz3.so.4: cannot open shared object
  file`). `bundle_wheel.sh` copies the libz3 the binary resolves into the
  payload's `lib/`.
- **Lyrical and Rolling crashed on the first LoadNode.** From Lyrical on
  (ros2/rosidl#942) primitive and string C sequences carry two trailing
  flags and are 32 bytes; `rosidl_runtime_rs` 0.6.0 laid every sequence out
  as 24, so every field after one was misread
  (`get_serialized_size_rcl_interfaces__msg__Parameter`, SIGSEGV). Upstream
  fixed it in 0.7.0; the generated message crates ask for `"0.6"`, so our
  fork's `play_launch` branch carries that fix on 0.6.0 instead. Its ABI
  probe compares against the installed C headers and passes on Humble and
  Rolling.
- composition's demo launch file is `composition_demo.launch.py` on Humble
  and `composition_demo_launch.py` later (a smoke-script fix only).

### P3. Retire the meta package and the per-distribution projects

- Remove `packaging/meta`, the extras, the rename step, and the README's
  `play_launch[<distro>]` instructions; keep `play_launch[<distro>]` as a
  harmless no-op extra for one release if pip allows it, so existing
  install lines do not break.
- Gate: `pip install play_launch` in each distribution's container installs
  the right wheel and runs; `pip install "play_launch[humble]"` still works.

**Done (2026-10-16).** `packaging/meta/` and the workflow's rename step are
gone; `pyproject.toml` keeps `humble`, `jazzy`, `kilted`, `lyrical` and
`rolling` as empty extras, to be removed at 0.20, and
`pip install "<wheel>[jazzy]"` installs. README says `pip install
play_launch`. `just publish-pypi` refuses a local `py3-none-any` wheel,
since it carries one payload and claims every platform; releases come
from CI on a `v*` tag.

**One departure: no sdist.** The design published one as a source
fallback. Built from source it would produce a tool with no payload at all
unless the build environment held a sourced ROS, Rust and the C++
packages, and pip would pick it silently wherever no wheel matched. With
no sdist, an unsupported platform gets pip's own "no matching
distribution", which is the true answer.

### P4. Release 0.18.2

(Planned as 0.17.1. 0.18.0 and 0.18.1 were tagged before this phase
landed, with the per-distribution workflow, and their release runs were
cancelled; PyPI's newest is still 0.16.0. A tag is never moved, so the
release is the next unused version, 0.18.2.)

- Version bump, CHANGELOG (the 0.17.0 split reverted in favour of run-time
  selection; 0.17.0 yanked, 0.18.0 and 0.18.1 never published), tag, workflow, PyPI.
- Gate: `pip install play_launch==0.18.2` from PyPI works in a fresh
  container for each distribution and CPU (QEMU for arm64 as the build
  does); the GitHub release lists the six wheels.

**Done (2026-10-16).** Tagged `v0.18.2` at 23e6e9bf. The release run
(38856862764) rebuilt all ten rows, smoke-tested every wheel on both CPUs,
and published; PyPI and the GitHub release each list the six wheels
(20-46 MB). `pip install play_launch==0.18.2` from PyPI into a fresh
`ros:<d>-ros-core` container, then `scripts/smoke_wheel.sh`, passed for
humble, jazzy, kilted, lyrical and rolling on x86_64, each `--version`
naming its own distribution (`play_launch 0.18.2 (ROS lyrical, v0.18.2,
rlm v0.1.62)`). The aarch64 wheels on PyPI are the files the release run
smoke-tested under QEMU before publishing; they were not installed from
PyPI again.

### P5. Consumers

- nano-ros pins (play_launch gitlink) and any consumer install lines that
  say `play_launch[<distro>]` or install from release assets go back to
  `pip install play_launch==0.18.2`.

**Done (2026-10-17), with nothing to change.** No consumer ever adopted
0.17.0's `play_launch[<distro>]` lines or its release assets: they never
reached PyPI. Every install line in the sibling repositories (golf cart,
AutoSDV, carla-scenario-bridge, csb-seps, crdt-tensor, LCTK) is a plain
`pip install play_launch`, some with a floor (`>=0.10.0`, `>=0.15.1`),
and resolves to 0.18.2 on its next upgrade. nano-ros installs no wheel:
it builds layer 2 from its `packages/cli/third-party/play_launch` gitlink
(at 0.18.1's 76ae4f91) and takes `play_launch_parser` from its SDK index.
`src/ros-launch-resolve` is identical between that pin and `v0.18.2`, so
the gitlink stays.

One behaviour to know: on a Python no 0.18.2 wheel serves (3.11, 3.13, a
glibc below 2.35), pip does not fail; it falls back to the newest release
that matches, 0.16.0, whose `py3-none-manylinux_2_35` wheels carry a
Humble build. That was already so before this phase.

## Not in this phase

- Distributions beyond the five in the matrix.
- Windows and macOS wheels.
