# Phase 92 - one PyPI package: `pip install play_launch`, the distribution picked at run time

Status: **planned** (2026-10-15). Follows the 0.17.0 release, whose PyPI
upload failed and was yanked.

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

### P2. Assemble multi-payload wheels in CI

- `assemble-wheels` job; the matrix rows upload payloads; one wheel per tag;
  completeness check; wheel contents listed in the job log.
- Gate: a workflow run on a branch (no publish) produces six wheels; each
  cp312/cp314 wheel holds two payloads; each installs in its distribution's
  container and passes a smoke test (`play_launch --version`, `play_launch
  check` on a fixture, one short `play_launch launch` of a demo node) for
  every payload it carries.

### P3. Retire the meta package and the per-distribution projects

- Remove `packaging/meta`, the extras, the rename step, and the README's
  `play_launch[<distro>]` instructions; keep `play_launch[<distro>]` as a
  harmless no-op extra for one release if pip allows it, so existing
  install lines do not break.
- Gate: `pip install play_launch` in each distribution's container installs
  the right wheel and runs; `pip install "play_launch[humble]"` still works.

### P4. Release 0.17.1

- Version bump, CHANGELOG (the 0.17.0 split reverted in favour of run-time
  selection; 0.17.0 yanked), tag, workflow, PyPI.
- Gate: `pip install play_launch==0.17.1` from PyPI works in a fresh
  container for each distribution and CPU (QEMU for arm64 as the build
  does); the GitHub release lists the six wheels and the sdist.

### P5. Consumers

- nano-ros pins (play_launch gitlink) and any consumer install lines that
  say `play_launch[<distro>]` or install from release assets go back to
  `pip install play_launch==0.17.1`.

## Not in this phase

- Distributions beyond the five in the matrix.
- Windows and macOS wheels.
