#!/usr/bin/env bash
# Grant the helper binaries their file capabilities using a container, so the
# edit-build-test loop needs no password.
#
# Every colcon build COPIES the helpers into `install/` (fresh inodes, nlink 1)
# and a capability lives on the inode, so this has to run after every build.
# Doing that with `sudo` means a password prompt mid-cycle.
#
# WHY A CONTAINER WORKS
#   Setting a file capability needs only CAP_SETFCAP — you do not need to hold
#   the capability you are granting. A rootful container's default bounding set
#   already has CAP_SETFCAP, and a bind mount writes the `security.capability`
#   xattr to the same inode the host sees. So the capability is host-valid.
#
# WHY A ROOTLESS CONTAINER DOES NOT
#   Inside a user namespace the xattr is written as VFS_CAP_REVISION_3, which
#   carries a rootid. The kernel honours it only for processes whose userns
#   maps that rootid. A host process executing the binary gets EPERM while
#   `getcap` still prints the capability — a silent wrong answer, which is
#   worse than a clear failure. Every rootless runtime is therefore REFUSED
#   here, not fallen back on.
#
# Scope: a DEVELOPER convenience. The user-facing install path is still
# `sudo setcap`, typed by the user — see docs/guide/rt-scheduling.md.
#
#   setcap_container.sh [install-dir]
#
# Exit codes: 0 applied · 1 usage/build error · 2 no usable runtime (the
# message says what one-time admin action would fix it).
set -uo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DIR=${1:-$REPO/install/play_launch/lib/play_launch}
MAIN=$DIR/play_launch
IO_HELPER=$DIR/play_launch_io_helper
RT_HELPER=$DIR/play_launch_rt_helper
IMAGE=play-launch-setcap:local

for f in "$IO_HELPER" "$RT_HELPER"; do
  if [ ! -f "$f" ]; then
    echo "error: $(basename "$f") not found under $DIR — run 'just build' first." >&2
    exit 1
  fi
done

# The work itself, as a shell fragment run as root inside the container.
#
# Removing any capability from the MAIN binary is not tidiness: a file
# capability makes the loader treat the process as AT_SECURE, which makes it
# ignore LD_LIBRARY_PATH, and it then cannot find its ROS libraries.
read -r -d '' WORK <<'SH' || true
set -e
[ -n "$(getcap /w/play_launch 2>/dev/null)" ] && setcap -r /w/play_launch || true
setcap cap_sys_ptrace+ep /w/play_launch_io_helper
setcap cap_sys_nice+ep   /w/play_launch_rt_helper
SH

# Why each candidate runtime was rejected, so the failure names a cause rather
# than just an absence.
REASONS=()

# Is this runtime rootless? Checked per runtime, because the answer decides
# whether the result would be host-valid or a convincing-looking no-op.
docker_is_rootless() { docker info --format '{{.SecurityOptions}}' 2>/dev/null | grep -q rootless; }
podman_is_rootless() { [ "$(podman info --format '{{.Host.Security.Rootless}}' 2>/dev/null)" = "true" ]; }

try_docker_like() {
  local rt=$1 rootless_check=$2
  command -v "$rt" >/dev/null 2>&1 || { REASONS+=("$rt: not installed"); return 1; }
  if ! "$rt" info >/dev/null 2>&1; then
    local sock="" hint=""
    case $rt in
      docker) sock=/var/run/docker.sock ;;
      podman) sock="" ;;
    esac
    if [ -n "$sock" ] && [ -S "$sock" ] && [ ! -w "$sock" ]; then
      hint=" (socket $sock is $(stat -c '%U:%G %a' "$sock" 2>/dev/null); you are $(id -un), groups: $(id -Gn))"
    fi
    REASONS+=("$rt: daemon unreachable$hint")
    return 1
  fi
  if "$rootless_check"; then
    REASONS+=("$rt: ROOTLESS — a capability set in a user namespace is not valid to host processes, so this is refused rather than silently producing one that reads correct and does not work")
    return 1
  fi
  if ! "$rt" image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Building the setcap helper image (one time)..."
    "$rt" build -q -f "$REPO/docker/setcap.Dockerfile" -t "$IMAGE" "$REPO" >/dev/null || {
      REASONS+=("$rt: image build failed")
      return 1
    }
  fi
  # Mount ONLY the helper directory — this runs as root inside the container,
  # so the smaller the bind mount the better.
  "$rt" run --rm -v "$DIR:/w" "$IMAGE" sh -c "$WORK" || {
    REASONS+=("$rt: container run failed")
    return 1
  }
  VIA="$rt (no sudo)"
  return 0
}

try_sudo() {
  command -v sudo >/dev/null 2>&1 || { REASONS+=("sudo: not installed"); return 1; }
  if ! sudo -n true 2>/dev/null; then
    REASONS+=("sudo: needs a password (no non-interactive rule for setcap)")
    return 1
  fi
  [ -f "$MAIN" ] && [ -n "$(getcap "$MAIN" 2>/dev/null)" ] && sudo -n setcap -r "$MAIN"
  sudo -n setcap cap_sys_ptrace+ep "$IO_HELPER" || return 1
  sudo -n setcap cap_sys_nice+ep "$RT_HELPER" || return 1
  VIA="sudo (non-interactive)"
  return 0
}

VIA=""
try_docker_like docker docker_is_rootless \
  || try_docker_like podman podman_is_rootless \
  || try_sudo \
  || true

if [ -z "$VIA" ]; then
  echo "Could not grant file capabilities without a password. Tried:" >&2
  for r in "${REASONS[@]}"; do echo "  - $r" >&2; done
  cat >&2 <<EOF

None of these can be worked around by a script: writing a host-valid file
capability requires CAP_SETFCAP in the INITIAL user namespace, which means
either a rootful container runtime or root itself. A rootless runtime cannot
substitute — see the header of this script for why its result would look
correct and not work.

One-time admin action, whichever suits the machine:
  sudo usermod -aG docker $(id -un)     # then log out and back in
  sudo setcap cap_sys_ptrace+ep $IO_HELPER
  sudo setcap cap_sys_nice+ep   $RT_HELPER

Note that docker group membership is root-equivalent, which is why this is an
explicit decision rather than something the build does for you.
EOF
  exit 2
fi

getcap "$IO_HELPER"
getcap "$RT_HELPER"
echo "✓ helpers ready (reapply after every build) [$VIA]"
