# A container used as a non-interactive `setcap` for the dev loop.
#
# Every colcon build COPIES the helper binaries into `install/` (verified:
# differing inodes, nlink 1), and a fresh inode has an empty capability set —
# so `just setcap` has to run after every build. Doing that with `sudo` means a
# password prompt in the middle of an edit-build-test cycle.
#
# A throwaway container solves it without changing any host policy:
#   - Docker's DEFAULT capability bounding set already contains CAP_SETFCAP
#     (0xa80425fb), and setting a file capability needs only CAP_SETFCAP — you
#     do not need to hold the capability you are granting.
#   - The helper directory is bind-mounted, so the `security.capability` xattr
#     is written to the same inode the host sees. The capability is host-valid.
#
# Deliberately NOT the CI builder image: `just setcap` would then break every
# time `docker/builder.Dockerfile` changed and the image had not been rebuilt
# and pushed yet. This image depends on nothing but `ubuntu:22.04` and is built
# on demand, once, then cached.
#
# Scope: this is a DEVELOPER convenience. The user-facing install procedure is
# still `sudo setcap` typed by the user — see docs/guide/rt-scheduling.md.
#
# Built on demand by the `setcap` recipe in the justfile:
#   docker build -q -f docker/setcap.Dockerfile -t play-launch-setcap:local .

FROM ubuntu:22.04

RUN apt-get update && \
    apt-get install -y --no-install-recommends libcap2-bin && \
    rm -rf /var/lib/apt/lists/*
