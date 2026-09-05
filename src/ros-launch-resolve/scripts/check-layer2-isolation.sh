#!/usr/bin/env bash
#
# RFC-0060 layer-2 isolation gate.
#
# The workspace manifest states the invariant:
#
#   "this workspace must resolve and build under PLAIN CARGO, with no ROS
#    install, no ament environment and no colcon. CPython is the one external
#    requirement, and only because `.launch.py` must be executed against the
#    user's interpreter."
#
# Until this script existed the invariant was a comment. It held by accident of
# nobody having added the wrong dependency yet, and it would have broken
# silently: a developer shell and CI both have ROS sourced, so a resolver that
# had started needing rclrs or an ament index would keep passing every test in
# the tree while becoming unbuildable for the consumer the layer exists for
# (nano-ros, issue 0285).
#
# The gate runs the build and the resolver under a STRIPPED environment and
# checks four properties:
#
#   1. builds with no ROS environment
#   2. no rclrs / rosidl / ament crate anywhere in the dependency graph
#   3. links no ROS shared library, and does link libpython
#   4. resolves BOTH frontends -- .launch.xml and .launch.py -- with no ROS,
#      and the declared node actually appears in the emitted model
#
# Property 4 checks model CONTENTS rather than the exit status. `resolve` can
# exit 0 having produced a model with no nodes in it, which is exactly the
# failure a boundary regression would produce.
#
# VACUITY GUARD. Issue 0012 was a differential oracle that treated a missing
# ROS install as a skip, so it reported success precisely on the machines where
# drift was guaranteed. The same trap applies here in reverse: if the
# environment is not genuinely stripped, every check below passes for the wrong
# reason. So the preconditions are asserted and a violated one is a FAILURE,
# never a skip.
#
# INHERITED CARGO CONFIG. Cargo walks parent directories for
# `.cargo/config.toml`, and that walk does NOT stop at a workspace boundary or
# at the `exclude` list. Checked out inside play_launch, this workspace
# therefore picks up play_launch's colcon-generated `[patch.crates-io]` and
# `-L native=.../install/...` rustflags. Today that is inert -- the patches
# land in Cargo.lock as `[[patch.unused]]`, which is itself evidence the graph
# does not want them -- but it means an in-tree run is not identical to a clean
# standalone clone. `--standalone` copies the workspace outside the parent tree
# and runs there, which is the real W1 acceptance condition; the CI job needs
# no such trick because `.cargo/config.toml` is gitignored and never checked
# out.
#
# Usage: scripts/check-layer2-isolation.sh [--release] [--standalone]

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
REPO="$PWD"

PROFILE_ARGS=()
PROFILE_DIR="debug"
STANDALONE=0
for arg in "$@"; do
    case "$arg" in
        --release) PROFILE_ARGS=(--release); PROFILE_DIR="release" ;;
        --standalone) STANDALONE=1 ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

# Re-exec in a copy placed outside every parent that could contribute a cargo
# config. `_L2_ISOLATION_COPY` stops the recursion.
if (( STANDALONE )) && [[ -z "${_L2_ISOLATION_COPY:-}" ]]; then
    src="$PWD"
    dest="$(mktemp -d -p /tmp l2-isolation-XXXXXX)"
    trap 'rm -rf "$dest"' EXIT
    echo "Standalone mode: copying the workspace to ${dest}"
    tar -c --exclude=target --exclude=tmp --exclude=.git -C "$src" . | tar -x -C "$dest"
    # A stray config INSIDE the copy would defeat the point of moving it.
    find "$dest" -name config.toml -path '*/.cargo/*' -delete
    _L2_ISOLATION_COPY=1 "$dest/scripts/check-layer2-isolation.sh" "${PROFILE_ARGS[@]}"
    exit $?
fi

# PATH needs filtering, not unsetting: the build needs cargo and the resolver
# needs python3, but any `/opt/ros/...` entry would leave `ros2` and the ament
# index reachable, which is most of what we are claiming is absent.
CLEAN_PATH="$(printf '%s' "${PATH}" | tr ':' '\n' | grep -v '/opt/ros/' | grep -v '/ros2_ws/' | paste -sd: -)"

# The variables that carry a ROS installation into a child process. Unsetting
# them is what makes this test mean anything.
STRIP=(
    env
    -u ROS_DISTRO
    -u ROS_VERSION
    -u ROS_PYTHON_VERSION
    -u ROS_LOCALHOST_ONLY
    -u ROS_PACKAGE_PATH
    -u AMENT_PREFIX_PATH
    -u CMAKE_PREFIX_PATH
    -u COLCON_PREFIX_PATH
    -u LD_LIBRARY_PATH
    -u PYTHONPATH
    -u RMW_IMPLEMENTATION
    "PATH=${CLEAN_PATH}"
)

failures=0
pass() { printf '  \033[32mPASS\033[0m  %s\n' "$1"; }
fail() { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; failures=$((failures + 1)); }

echo "RFC-0060 layer-2 isolation gate (${PROFILE_DIR})"
echo

# --- 0. Preconditions -------------------------------------------------------
# A stripped environment that still finds ROS makes every later check vacuous.
echo "Preconditions"

leaked="$("${STRIP[@]}" sh -c 'env | grep -E "^(ROS_|AMENT_|COLCON_)" || true')"
if [[ -n "$leaked" ]]; then
    fail "ROS variables survived the strip -- the rest of this run would be vacuous:"
    printf '        %s\n' $leaked
else
    pass "no ROS_/AMENT_/COLCON_ variables in the stripped environment"
fi

if "${STRIP[@]}" sh -c 'command -v ros2 >/dev/null 2>&1'; then
    fail "'ros2' is still on PATH -- strip is incomplete, results would be vacuous"
else
    pass "'ros2' not on PATH"
fi

if ! "${STRIP[@]}" python3 -c 'import sys' >/dev/null 2>&1; then
    fail "no usable python3 -- CPython is a real requirement of this layer"
else
    pass "python3 available (the one dependency this layer is allowed)"
fi

# Not a failure: `launch` can be pip-installed independently of ROS. But if it
# IS importable, property 4 no longer proves the parser's mock modules are
# doing the work, so say so rather than quietly claiming more than was tested.
# Cargo's config walk crosses the workspace/exclude boundary. Report it rather
# than let an in-tree run be quietly reported as a clean-clone result.
inherited=""
probe="$(dirname "$PWD")"
while [[ "$probe" != "/" && -n "$probe" ]]; do
    [[ -f "${probe}/.cargo/config.toml" ]] && inherited="${inherited} ${probe}/.cargo/config.toml"
    probe="$(dirname "$probe")"
done
if [[ -n "$inherited" ]]; then
    printf '  \033[33mNOTE\033[0m  a parent .cargo/config.toml applies to this workspace:\n'
    printf '        %s\n' $inherited
    printf '        Cargo'"'"'s config walk ignores workspace boundaries, so this run is\n'
    printf '        NOT clean-clone-equivalent. Re-run with --standalone for that.\n'
else
    pass "no parent .cargo/config.toml leaking into this workspace"
fi

if "${STRIP[@]}" python3 -c 'import launch' >/dev/null 2>&1; then
    printf '  \033[33mNOTE\033[0m  a real `launch` package is importable; the .launch.py check\n'
    printf '        still passes but no longer isolates the parser mock modules\n'
else
    pass "'import launch' fails -- the .launch.py check exercises the pyo3 mocks"
fi
echo

# --- 1. Builds with no ROS environment --------------------------------------
echo "Build"
if "${STRIP[@]}" cargo build -q "${PROFILE_ARGS[@]}" -p ros-launch-resolve-cli 2>&1; then
    pass "cargo build -p ros-launch-resolve-cli"
else
    fail "cargo build -p ros-launch-resolve-cli"
    echo
    echo "Build failed under a stripped environment -- the layer boundary is broken."
    exit 1
fi

# The Python half is a SECOND artifact, and it lives in a different cargo
# workspace (`parser/`), so the build above cannot produce it. After 0897 W3
# the driver links no libpython for the Rust parser's Python evaluation — it
# discovers a CPython at runtime and dlopens it, then this cdylib. Building
# only the driver leaves a binary that warns and carries on for every
# `.launch.py` file and every `$(eval ...)`, which reads as a parser
# limitation rather than a missing file. That is exactly the state `main` was
# in when this gate went red.
#
# `extension-module` leaves the `Py_*` symbols undefined so they resolve
# against the interpreter the driver dlopened first; `abi3` restricts them to
# the stable ABI so a CPython other than the build's own works.
if (cd "${REPO}/parser" && "${STRIP[@]}" cargo build -q "${PROFILE_ARGS[@]}" \
        -p play_launch_parser_pyexec --features extension-module,abi3 2>&1); then
    pass "cargo build -p play_launch_parser_pyexec (cdylib)"
else
    fail "cargo build -p play_launch_parser_pyexec (cdylib)"
    echo
    echo "The driver builds but its Python half does not -- .launch.py and \$(eval ...) cannot work."
    exit 1
fi
echo

# --- 2. Dependency graph ----------------------------------------------------
echo "Dependency graph"
# rclrs / rosidl_runtime_rs are colcon-generated and only exist inside a built
# ROS workspace; an ament-* crate would mean the same coupling by another name.
banned="$("${STRIP[@]}" cargo tree "${PROFILE_ARGS[@]}" -p ros-launch-resolve-cli 2>/dev/null \
    | grep -oE '\b(rclrs|rosidl[a-z_]*|ament[a-z_]*|r2r)\b' | sort -u || true)"
if [[ -n "$banned" ]]; then
    fail "ROS crates reached the dependency graph:"
    printf '        %s\n' $banned
else
    total="$("${STRIP[@]}" cargo tree "${PROFILE_ARGS[@]}" -p ros-launch-resolve-cli 2>/dev/null | wc -l)"
    pass "no rclrs/rosidl/ament crate among ${total} graph entries"
fi
echo

# --- 3. Link -----------------------------------------------------------------
echo "Link"
target_dir="$(cargo metadata --format-version 1 --no-deps -q \
    | python3 -c 'import sys, json; print(json.load(sys.stdin)["target_directory"])')"
bin="${target_dir}/${PROFILE_DIR}/ros-launch-resolve"

if [[ ! -x "$bin" ]]; then
    fail "binary not found at ${bin}"
else
    libs="$(ldd "$bin" | sed 's/^[[:space:]]*//' | cut -d' ' -f1)"
    ros_libs="$(printf '%s\n' "$libs" | grep -E '^lib(rcl|rmw|rcutils|rosidl|ament|rclcpp|rclpy)' || true)"
    if [[ -n "$ros_libs" ]]; then
        fail "binary links ROS shared libraries:"
        printf '        %s\n' $ros_libs
    else
        pass "no rcl/rmw/rcutils/rosidl/ament shared library"
    fi

    if printf '%s\n' "$libs" | grep -q '^libpython'; then
        pass "libpython linked ($(printf '%s\n' "$libs" | grep '^libpython'))"
    else
        # Not a leak -- but it means .launch.py cannot work, so the binary is
        # not the thing this layer is supposed to ship.
        fail "libpython NOT linked -- .launch.py support would be gone"
    fi
fi
echo

# The driver looks for the Python half beside itself first. The two are built
# from two workspaces, and under --standalone they do not even share a target
# directory, so place it rather than assume cargo did.
if [[ -x "$bin" ]]; then
    pyexec_dir="$(cd "${REPO}/parser" && "${STRIP[@]}" cargo metadata --format-version 1 --no-deps -q \
        | python3 -c 'import sys, json; print(json.load(sys.stdin)["target_directory"])')"
    pyexec_so="${pyexec_dir}/${PROFILE_DIR}/libplay_launch_parser_pyexec.so"
    if [[ -f "$pyexec_so" ]]; then
        if [[ "$pyexec_so" != "$(dirname "$bin")/libplay_launch_parser_pyexec.so" ]]; then
            cp "$pyexec_so" "$(dirname "$bin")/"
        fi
        # An undefined Py_* set with no libpython DT_NEEDED is the point: a
        # cdylib that linked one would load only against its build interpreter.
        if ldd "$pyexec_so" 2>/dev/null | grep -q libpython; then
            fail "the Python half links libpython -- a runtime-chosen interpreter cannot work"
        else
            pass "Python half present and names no interpreter"
        fi
    else
        fail "Python half not found at ${pyexec_so}"
    fi
fi
echo

# --- 4. Both frontends resolve, and the model has the node in it -------------
echo "Resolve (stripped environment)"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

check_resolve() {
    local fixture="$1"
    local expect="$2"
    local out="${work}/$(basename "$fixture").yaml"
    local log="${work}/$(basename "$fixture").log"
    if ! "${STRIP[@]}" "$bin" resolve "$fixture" -o "$out" >"$log" 2>&1; then
        fail "resolve ${fixture}"
        sed 's/^/        /' "$log" | tail -15
        return
    fi
    if ! grep -q "$expect" "$out" 2>/dev/null; then
        # Exit 0 with an empty model is the shape a boundary regression takes.
        fail "resolve ${fixture} succeeded but '${expect}' is absent from the model"
        return
    fi
    pass "resolve ${fixture} -- '${expect}' present in the model"
}

check_resolve tests/isolation/isolation.launch.xml isolation_talker
check_resolve tests/isolation/isolation.launch.py isolation_py_talker

# $(eval '21 * 2') routes unconditionally to CPython. A correct 42 proves
# libpython is FUNCTIONAL, not merely present in ldd output.
if grep -qE 'answer: 42$' "${work}/isolation.launch.xml.yaml" 2>/dev/null; then
    pass "\$(eval '21 * 2') evaluated to 42 -- embedded CPython is functional"
else
    fail "\$(eval ...) did not evaluate -- embedded CPython is linked but not working"
fi
echo

if (( failures > 0 )); then
    echo "FAILED: ${failures} check(s). The layer-2 boundary is broken."
    exit 1
fi
echo "All checks passed. Layer 2 builds and runs with no ROS installation."
