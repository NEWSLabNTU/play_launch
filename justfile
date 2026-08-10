# Detect ROS2 distro based on Ubuntu version
ros_distro := if `lsb_release -rs 2>/dev/null || echo "22.04"` == "24.04" { "jazzy" } else { "humble" }
colcon_flags := "--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release"

# ros-launch-resolve binary (layer 2 CLI) — the developer/integration binary
# that resolves launch trees with no ROS runtime. Same verbs as play_launch's
# resolve/dump/check/plot/contract, same `ros_launch_resolve::verbs::*`
# implementation; used by the tooling below so parity checks don't need ROS.
# Not part of the colcon install/ tree — it builds under plain cargo, no ROS —
# so it lives in its own workspace's target/. Release preferred, falls back
# to debug (mirrors tests/src/fixtures.rs::ros_launch_resolve_bin()).
resolve_bin := `
    for p in src/ros-launch-resolve/target/release/ros-launch-resolve \
             src/ros-launch-resolve/target/debug/ros-launch-resolve; do
        [ -f "$p" ] && { echo "$p"; exit 0; }
    done
    echo "src/ros-launch-resolve/target/debug/ros-launch-resolve"
`

# Show available recipes
default:
    @just --list

# Install all dependencies
install-deps:
    #!/usr/bin/env bash
    set -e

    # Check for conflicting colcon packages
    if pip show colcon-ros-cargo &>/dev/null || pip show colcon-cargo &>/dev/null; then
        echo "Warning: Conflicting packages detected:"
        pip show colcon-ros-cargo 2>/dev/null | grep -E '^(Name|Version):' || true
        pip show colcon-cargo 2>/dev/null | grep -E '^(Name|Version):' || true
        echo
        read -p "Uninstall colcon-ros-cargo and colcon-cargo? (Required for colcon-cargo-ros2) [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            pip uninstall -y colcon-ros-cargo colcon-cargo 2>/dev/null || true
        else
            echo "Warning: colcon-cargo-ros2 may conflict with existing packages."
        fi
    fi

    # Install colcon-cargo-ros2
    pip install 'colcon-cargo-ros2>=0.4.0'

    source /opt/ros/{{ros_distro}}/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

# Full build: colcon + bundle + wheel
build:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon build {{colcon_flags}} --base-paths src
    just build-interception
    # The layer-2 developer binary. `play_launch` carries every verb a user
    # needs, so this is NOT bundled into the wheel — but the integration
    # tests, `just compare-dumps` and the layer-2 isolation gate all run it,
    # and it's a standalone cargo workspace outside colcon, so `colcon build`
    # above never touches it. Release, to match colcon_flags'
    # `--cargo-args --release` for play_launch itself.
    (cd src/ros-launch-resolve && cargo build --release --bin ros-launch-resolve)
    # See build-wheel: setuptools' build/lib staging is never pruned.
    rm -rf build/lib build/bdist.*
    scripts/bundle_wheel.sh
    uv build --wheel
    echo ""
    echo "Build complete:"
    ls -lh dist/*.whl
    # colcon COPIES the helpers into install/, and a fresh inode has an empty
    # capability set — so every build drops them. Reapply automatically when it
    # costs nothing (rootful Docker: no prompt); otherwise just say so, because
    # a sudo password prompt at the end of a build is exactly what this avoids.
    if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1 \
       && ! docker info --format '{{ "{{" }}.SecurityOptions{{ "}}" }}' 2>/dev/null | grep -q rootless; then
        echo ""
        just setcap
    else
        echo ""
        echo "Helper capabilities were dropped by the build — run 'just setcap'."
    fi

# C++ only (msgs + container)
build-cpp:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon build {{colcon_flags}} --packages-select play_launch_msgs play_launch_container --base-paths src

# Rust only (assumes C++ install/ exists)
build-rust:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon build {{colcon_flags}} --packages-select play_launch --base-paths src

# Build interception .so (standalone, not in colcon workspace)
build-interception:
    #!/usr/bin/env bash
    set -e
    (cd src/spsc_shm && cargo build --release)
    (cd src/play_launch_interception && cargo build --release --features {{ros_distro}})
    echo "Built: src/play_launch_interception/target/release/libplay_launch_interception.so"

# Bundle colcon artifacts + build wheel (no colcon rebuild)
build-wheel:
    #!/usr/bin/env bash
    set -e
    # setuptools stages into build/lib and NEVER prunes it, so a file deleted
    # from python/play_launch/ keeps shipping in the wheel until this is
    # cleared. That is how a removed module and an unbundled binary both
    # survived into a rebuilt 0.9.0 wheel. (build/<pkg>/ is colcon's; only the
    # setuptools staging dirs are removed here.)
    rm -rf build/lib build/bdist.*
    scripts/bundle_wheel.sh
    uv build --wheel

# Run play_launch with optional arguments (e.g., just run launch demo_nodes_cpp talker_listener.launch.py)
run *ARGS:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash
    ros2 run play_launch play_launch {{ARGS}}

# Apply CAP_SYS_PTRACE to the I/O helper and CAP_SYS_NICE to the RT helper.
# One capability per helper — never pooled onto one binary.
#
# Every colcon build COPIES the helpers into `install/` (differing inodes,
# nlink 1), and a fresh inode has an empty capability set, so this must run
# after every build. It therefore prefers a throwaway container over `sudo`:
# Docker's DEFAULT bounding set already holds CAP_SETFCAP, granting a file
# capability needs only CAP_SETFCAP (not the capability being granted), and the
# bind mount means the xattr lands on the host inode. Non-interactive, no host
# policy change. Falls back to `sudo` when Docker is unusable.
#
# This is a DEVELOPER convenience. The user-facing install procedure is still
# `sudo setcap` typed by the user.
#
# NOTE: the main play_launch binary is deliberately NOT capped — a file capability
# puts it in secure-execution mode (AT_SECURE), which makes the loader ignore
# LD_LIBRARY_PATH, and it needs LD_LIBRARY_PATH to find its ~22 ROS libraries.
# After this, RT scheduling (--sched) works WITHOUT root — play_launch delegates
# the apply syscalls to play_launch_rt_helper over IPC.
setcap:
    #!/bin/bash
    set -e
    dir=install/play_launch/lib/play_launch
    main=$dir/play_launch
    io_helper=$dir/play_launch_io_helper
    rt_helper=$dir/play_launch_rt_helper

    if [ ! -f "$io_helper" ]; then
        echo "Error: play_launch_io_helper not found. Run 'just build' first."
        exit 1
    fi
    if [ ! -f "$rt_helper" ]; then
        echo "Error: play_launch_rt_helper not found. Run 'just build' first."
        exit 1
    fi

    # Decide the mechanism. Rootless Docker is REJECTED, not fallen back on
    # quietly for the wrong reason: inside a user namespace `security.capability`
    # is namespaced (it carries a rootid) and is honored only within that
    # namespace, so `getcap` on the host would show the capability and host
    # processes would still get EPERM. A silent wrong answer is worse than sudo.
    use_docker=0
    if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
        if docker info --format '{{ "{{" }}.SecurityOptions{{ "}}" }}' 2>/dev/null | grep -q rootless; then
            echo "! rootless Docker detected — file capabilities set inside a user"
            echo "  namespace are not valid to host processes. Using sudo instead."
        else
            use_docker=1
        fi
    fi

    if [ "$use_docker" = "1" ]; then
        # Built on demand, then cached. Depends only on ubuntu:22.04 — NOT on
        # the CI builder image, which would make this break whenever that image
        # was edited but not yet rebuilt.
        if ! docker image inspect play-launch-setcap:local >/dev/null 2>&1; then
            echo "Building the setcap helper image (one time)..."
            docker build -q -f docker/setcap.Dockerfile -t play-launch-setcap:local . >/dev/null
        fi
        # Mount ONLY the helper directory: this runs as root inside the
        # container, so the smaller the bind mount the better.
        docker run --rm -v "$PWD/$dir:/w" play-launch-setcap:local sh -c '
            set -e
            [ -n "$(getcap /w/play_launch 2>/dev/null)" ] && setcap -r /w/play_launch || true
            setcap cap_sys_ptrace+ep /w/play_launch_io_helper
            setcap cap_sys_nice+ep   /w/play_launch_rt_helper
        '
        via="docker (no sudo)"
    else
        # Self-heal: a capability on the main binary is ALWAYS wrong (AT_SECURE
        # would make the loader ignore LD_LIBRARY_PATH and it could not find its
        # ROS libs).
        if [ -f "$main" ] && [ -n "$(getcap "$main" 2>/dev/null)" ]; then
            echo "! main binary has file capabilities — removing (they break ROS lib loading)"
            sudo setcap -r "$main"
            echo "  removed."
        fi
        sudo setcap cap_sys_ptrace+ep "$io_helper"
        sudo setcap cap_sys_nice+ep "$rt_helper"
        via="sudo"
    fi

    getcap "$io_helper"
    echo "✓ I/O helper ready (reapply after rebuild) [$via]"
    getcap "$rt_helper"
    echo "✓ RT helper ready (reapply after rebuild) [$via]"

    echo
    echo "RT scheduling (--sched) now works WITHOUT root — no sudo/setuid needed"
    echo "to run play_launch itself."

# Verify capabilities: main binary has NONE, io_helper has CAP_SYS_PTRACE,
# rt_helper has CAP_SYS_NICE.
verify:
    #!/bin/bash
    ok=0

    # The main binary must have NO capabilities: one would set AT_SECURE, the
    # loader would ignore LD_LIBRARY_PATH, and it could not find its ROS libs.
    if [ -f install/play_launch/lib/play_launch/play_launch ]; then
        cap=$(getcap install/play_launch/lib/play_launch/play_launch 2>/dev/null)
        if [ -n "$cap" ]; then
            echo "✗ play_launch has file capabilities set: $cap"
            echo "  This BREAKS ROS library loading (AT_SECURE drops LD_LIBRARY_PATH)."
            echo "  Remove: sudo setcap -r install/play_launch/lib/play_launch/play_launch"
            ok=1
        else
            echo "✓ play_launch has no file capabilities (correct)"
        fi
    fi

    if [ ! -f install/play_launch/lib/play_launch/play_launch_io_helper ]; then
        echo "✗ I/O helper not found. Run 'just build' first."
        ok=1
    else
        cap=$(getcap install/play_launch/lib/play_launch/play_launch_io_helper 2>/dev/null)
        if [ -z "$cap" ]; then
            echo "✗ I/O helper has no capabilities set"
            echo "  Run 'just setcap' to enable I/O monitoring for privileged processes"
            ok=1
        elif echo "$cap" | grep -qE "cap_sys_ptrace[=+]ep"; then
            # getcap prints "=ep" while setcap takes "+ep" — accept both.
            echo "✓ I/O helper ready: $cap"
        else
            echo "⚠ I/O helper has unexpected capabilities: $cap"
            echo "  Expected: cap_sys_ptrace=ep"
            echo "  Run 'just setcap' to fix"
            ok=1
        fi
    fi

    if [ ! -f install/play_launch/lib/play_launch/play_launch_rt_helper ]; then
        echo "✗ RT helper not found. Run 'just build' first."
        ok=1
    else
        cap=$(getcap install/play_launch/lib/play_launch/play_launch_rt_helper 2>/dev/null)
        if [ -z "$cap" ]; then
            echo "✗ RT helper has no capabilities set"
            echo "  Run 'just setcap' to enable non-root RT scheduling"
            ok=1
        elif echo "$cap" | grep -qE "cap_sys_nice[=+]ep"; then
            # getcap prints "=ep" while setcap takes "+ep" — accept both.
            echo "✓ RT helper ready: $cap"
        else
            echo "⚠ RT helper has unexpected capabilities: $cap"
            echo "  Expected: cap_sys_nice=ep"
            echo "  Run 'just setcap' to fix"
            ok=1
        fi
    fi

    exit $ok

# Prove RT scheduling actually applies UNPRIVILEGED (no sudo — this is now the
# supported path: `just setcap` grants CAP_SYS_NICE to play_launch_rt_helper,
# which the unprivileged play_launch process delegates the apply syscalls to).
#
# Checks EVERY TID of every scheduled node (not just the main thread): Linux
# scheduling attributes are per-thread, and `chrt -p <pid>` alone only reads
# the main thread, which would silently miss composables (scheduled after
# ~11 DDS/rmw threads already exist). Fails non-zero if any thread of a
# scheduled node is not SCHED_FIFO.
verify-sched-rt:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash

    toml=$(mktemp /tmp/pl_sched_XXXXXX.toml)
    printf '%s\n' \
      '[tiers.rt]' \
      'class = "real_time"' \
      '[tiers.rt.posix]' \
      'priority = 20' \
      'sched_class = "SCHED_FIFO"' \
      'core = 0' \
      '[[assign]]' \
      'tier = "rt"' \
      'scope = "/"' > "$toml"
    echo "=== sched spec ==="; cat "$toml"

    logdir=$(mktemp -d /tmp/pl_rt_log_XXXXXX)
    echo; echo "=== launching UNPRIVILEGED (no sudo) with --sched (running ~8s) ==="
    echo "    (requires: 'just setcap' has granted cap_sys_nice to play_launch_rt_helper)"
    install/play_launch/lib/play_launch/play_launch launch \
        tests/fixtures/simple_test/launch/pure_nodes.launch.xml \
        --sched "$toml" --sched-apply warn \
        --log-dir "$logdir" --disable-web-ui --disable-diagnostics >/dev/null 2>&1 &
    pl=$!
    sleep 8

    echo; echo "=== per-TID check: did EVERY thread of EVERY node get SCHED_FIFO? ==="
    found=0
    fail=0
    for f in "$logdir"/latest/node/*/pid; do
        [ -f "$f" ] || continue
        name=$(basename "$(dirname "$f")")
        pid=$(cat "$f")
        if [ ! -d "/proc/$pid/task" ]; then
            printf '  %-22s pid=%-7s (process exited before check)\n' "$name" "$pid"
            continue
        fi
        found=1

        threads=0
        fifo=0
        prio="?"
        for tdir in /proc/"$pid"/task/*; do
            [ -d "$tdir" ] || continue
            threads=$((threads + 1))
            stat=$(cat "$tdir/stat" 2>/dev/null) || continue
            # comm (field 2) is parenthesized and may itself contain ')' or
            # whitespace — strip through the LAST ') ' before splitting the
            # remaining whitespace-separated fields.
            rest="${stat##*) }"
            read -ra fields <<< "$rest"
            # fields[] is 0-indexed starting at stat field 3 (state), so
            # field 40 (rt_priority) is fields[37] and field 41 (policy) is
            # fields[38].
            rt_priority="${fields[37]:-?}"
            policy="${fields[38]:-?}"
            # SCHED_FIFO == 1 (linux/sched.h)
            if [ "$policy" = "1" ]; then
                fifo=$((fifo + 1))
                prio="$rt_priority"
            fi
        done

        aff=$(taskset -cp "$pid" 2>/dev/null | sed -n 's/.*list: //p')
        printf '  %-22s pid=%-7s threads=%-3s fifo=%s/%s prio=%-3s cpu=%s\n' \
            "$name" "$pid" "$threads" "$fifo" "$threads" "$prio" "${aff:-?}"

        if [ "$fifo" != "$threads" ]; then
            fail=1
        fi
    done

    if [ "$found" != 1 ]; then
        echo "  (no live pid files under $logdir/latest/node/ — did nodes start?)"
        fail=1
    fi

    kill -TERM -"$(ps -o pgid= -p $pl | tr -d ' ')" 2>/dev/null || true
    wait $pl 2>/dev/null || true
    rm -f "$toml"

    echo
    if [ "$fail" = 1 ]; then
        echo "✗ FAIL: not every thread of every scheduled node is SCHED_FIFO"
        echo "  (did you run 'just setcap' to grant cap_sys_nice to play_launch_rt_helper?)"
        exit 1
    fi
    echo "✓ PASS: every thread of every scheduled node is SCHED_FIFO (prio=20, cpu=0)"

# Clean build artifacts
clean:
    rm -rf build install log dist target
    rm -rf tests/target
    rm -rf src/ros-launch-resolve/parser/target src/ros-launch-resolve/parser/build src/ros-launch-resolve/parser/install src/ros-launch-resolve/parser/log
    rm -rf .cargo/config.toml
    rm -rf play_log tmp *.egg-info
    scripts/bundle_wheel.sh --clean

# Deep clean (clean + node_modules, __pycache__, etc.)
clean-all: clean
    rm -rf node_modules __pycache__ python/**/__pycache__
    rm -f record.json junit.xml .tmp-extracted.js .tmp-extracted.css

# Build source distribution
build-sdist:
    #!/usr/bin/env bash
    set -e
    uv build --sdist

# Install wheel locally for testing
install-wheel:
    #!/usr/bin/env bash
    set -e
    # Install the newest wheel in dist/
    whl=$(ls -t dist/play_launch-*.whl | head -1)
    echo "Installing $whl"
    pip install "$whl" --force-reinstall

# Publish wheel to PyPI (requires PYPI_TOKEN env var)
publish-pypi:
    #!/usr/bin/env bash
    set -e

    if [ -z "$PYPI_TOKEN" ]; then
        echo "Error: PYPI_TOKEN environment variable not set."
        echo "Get a token from https://pypi.org/manage/account/token/"
        exit 1
    fi

    if ! ls dist/*.whl &>/dev/null; then
        echo "Error: No wheel found in dist/. Run 'just build' first."
        exit 1
    fi

    pip install twine
    twine upload dist/*.whl -u __token__ -p "$PYPI_TOKEN"

# Publish wheel to TestPyPI (for testing)
publish-testpypi:
    #!/usr/bin/env bash
    set -e

    if [ -z "$TESTPYPI_TOKEN" ]; then
        echo "Error: TESTPYPI_TOKEN environment variable not set."
        echo "Get a token from https://test.pypi.org/manage/account/token/"
        exit 1
    fi

    if ! ls dist/*.whl &>/dev/null; then
        echo "Error: No wheel found in dist/. Run 'just build' first."
        exit 1
    fi

    pip install twine
    twine upload --repository testpypi dist/*.whl -u __token__ -p "$TESTPYPI_TOKEN"

# Run standard tests — parser unit tests + fast integration tests (~3s)
test:
    #!/usr/bin/env bash
    set -e
    echo "=== C++ build check ==="
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon build --packages-select play_launch_msgs play_launch_container --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    echo ""
    echo "=== Parser unit tests ==="
    (cd src/ros-launch-resolve/parser && cargo nextest run -p play_launch_parser --no-fail-fast --failure-output final)
    echo ""
    # `resolve` moved off play_launch onto the layer-2 CLI (Task 6, CLI verb
    # reshape) — many fast-suite tests drive it directly via
    # `fixtures::ros_launch_resolve_cmd`, which panics (not skips) if it
    # isn't built, so it has to be here rather than only in `test-all`.
    echo "=== ros-launch-resolve CLI (layer 2) ==="
    (cd src/ros-launch-resolve && cargo build --bin ros-launch-resolve)
    echo ""
    echo "=== Integration tests (fast) ==="
    (cd tests && cargo nextest run -E 'not binary(autoware) & not binary(io_stress) & not binary(rt_workspace) & not test(/launch/)' --no-fail-fast --failure-output final)

# Run all tests — parser unit + all integration including Autoware (~30s)
test-all:
    #!/usr/bin/env bash
    set -e
    echo "=== C++ build check ==="
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon build --packages-select play_launch_msgs play_launch_container --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    echo ""
    echo "=== Parser unit tests ==="
    (cd src/ros-launch-resolve/parser && cargo nextest run -p play_launch_parser --no-fail-fast --failure-output final)
    echo ""
    echo "=== Fixture workspaces (guarded tests skip silently without these) ==="
    # A test that skips still reports as PASSED, so an unbuilt fixture hides
    # its whole suite behind a green summary. 27 of 108 integration tests
    # were skipping this way, concealing 4 real failures. Build them here so
    # `test-all` means what it says. Each is ~6s.
    (cd tests/fixtures/rt_workspace && just build)
    (cd tests/fixtures/io_stress && just build)
    # The relocated CLI (`dump`, `contract eject`, `plot` moved out of
    # play_launch in adc33a7) — several tests drive it directly.
    (cd src/ros-launch-resolve && cargo build --bin ros-launch-resolve)
    echo ""
    echo "=== Integration tests (all) ==="
    (cd tests && cargo nextest run --no-fail-fast --failure-output final)
    echo ""
    echo "=== Silently-skipped tests ==="
    # Surface what still skipped, so a guard that starts always-skipping is
    # visible rather than reported as a pass.
    (cd tests && cargo nextest run --no-capture 2>&1 | grep -iE "^\s*(SKIP|Skipping|skip):" | sort | uniq -c) || echo "  none"

# Run parser unit tests only
test-unit:
    #!/usr/bin/env bash
    set -e
    cd src/ros-launch-resolve/parser
    cargo nextest run -p play_launch_parser --no-fail-fast --failure-output final

# Run all integration tests (simple + Autoware)
test-integration:
    #!/usr/bin/env bash
    set -e
    cd tests
    cargo nextest run --no-fail-fast --failure-output final

# Run simple workspace integration tests
test-simple:
    #!/usr/bin/env bash
    set -e
    cd tests
    cargo nextest run -E 'binary(simple_workspace)' --no-fail-fast --failure-output final

# Run Autoware smoke test (health check: node exits + LoadNode failures)
smoke-test-autoware:
    #!/usr/bin/env bash
    set -e
    cd tests
    cargo nextest run -E 'test(smoke_test)' --no-capture

# Run Autoware integration tests
test-autoware:
    #!/usr/bin/env bash
    set -e
    cd tests
    cargo nextest run -E 'binary(autoware)' --no-fail-fast --failure-output final

# Compare Rust vs Python parser outputs — SystemModel shape (Phase 47:
# scripts/compare_records.py + compare_parsers.sh are retired; the
# parser-parity gate runs on the model now, scripts/compare_models.py).
compare-parsers:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash
    # Put the colcon-built binary ahead of PATH — a stale pip-installed
    # play_launch on the host would otherwise silently win (same fix the
    # fixture justfiles' compare-dumps recipes already apply).
    export PATH="$(pwd)/install/play_launch/lib/play_launch:$PATH"
    export PYTHONPATH="$(pwd)/python:$PYTHONPATH"
    TMPDIR=$(mktemp -d)
    trap "rm -rf $TMPDIR" EXIT
    FAILED=0
    for case in \
        "tests/fixtures/simple_test/launch/pure_nodes.launch.xml:Simple nodes" \
        "tests/fixtures/simple_test/launch/composition.launch.xml:Composable nodes" \
    ; do
        file="${case%%:*}"
        desc="${case##*:}"
        echo "----------------------------------------"
        echo "Test: $desc ($file)"
        {{resolve_bin}} resolve --parser rust -o "$TMPDIR/rust.yaml" "$file"
        {{resolve_bin}} resolve --parser python -o "$TMPDIR/python.yaml" "$file"
        if python3 scripts/compare_models.py "$TMPDIR/rust.yaml" "$TMPDIR/python.yaml"; then
            echo "PASS: $desc"
        else
            echo "FAIL: $desc"
            FAILED=1
        fi
        echo ""
    done
    exit $FAILED

# Compare scope tables between Rust and Python parsers
compare-scopes PKG LAUNCH *ARGS:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash
    TMPDIR=$(mktemp -d)
    trap "rm -rf $TMPDIR" EXIT
    # Driven through the layer-2 binary because this is a parser-parity
    # harness that must not need a ROS runtime; `play_launch dump` is the
    # same verb over the same implementation. It emits the SystemModel now
    # that record.json is retired (Phase 47.B2) — hence `.yaml`, which
    # compare_scopes.py adapts.
    echo "=== Dumping with Rust parser ==="
    {{resolve_bin}} dump -o "$TMPDIR/rust.yaml" launch {{PKG}} {{LAUNCH}} {{ARGS}}
    echo "=== Dumping with Python parser ==="
    {{resolve_bin}} dump -o "$TMPDIR/python.yaml" launch --parser python {{PKG}} {{LAUNCH}} {{ARGS}}
    echo "=== Comparing scopes ==="
    python3 scripts/compare_scopes.py "$TMPDIR/rust.yaml" "$TMPDIR/python.yaml"

# Benchmark Rust vs Python parser performance
benchmark-parsers ITERATIONS="5":
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash
    ITERATIONS={{ITERATIONS}} ./scripts/benchmark_parsers.sh

# Run checks (clippy + rustfmt check + ruff + cpplint + clang-format)
check:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash

    echo "=== play_launch (clippy) ==="
    (
    cd src/play_launch &&
    cargo clippy --all-targets --all-features -- -D warnings
    )

    echo ""
    echo "=== play_launch (rustfmt) ==="
    (cd src/play_launch && cargo +nightly fmt --check)

    echo ""
    echo "=== play_launch_parser (clippy + rustfmt) ==="
    (cd src/ros-launch-resolve/parser && just check)

    echo ""
    echo "=== Python (ruff) ==="
    python3 -m ruff check python/

    echo ""
    echo "=== C++ (cpplint) ==="
    ament_cpplint {{cpp_packages}}

    echo ""
    echo "=== C++ (clang-format check) ==="
    ament_clang_format {{cpp_packages}}

    echo ""
    echo "=== Web UI (html/css/js lint + TypeScript) ==="
    just check-web-ui

    echo ""
    just check-layer2-isolation

# RFC-0060 layer-2 isolation gate: prove ros-launch-resolve still builds and
# resolves with NO ROS installation. The script strips the environment itself,
# so this is safe to run from a sourced shell -- that is the point, since every
# shell that would notice a regression has ROS sourced and would not notice.
# Phase 55 W1's acceptance criterion; useful on its own whether or not the
# merge happens.
check-layer2-isolation:
    ./src/ros-launch-resolve/scripts/check-layer2-isolation.sh

# Generate TypeScript bindings from Rust types (ts-rs)
generate-bindings:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash
    echo "Generating TypeScript bindings..."
    (cd src/play_launch && cargo test -- export_bindings)
    echo "Generated $(ls src/play_launch/bindings/*.ts | wc -l) type files in src/play_launch/bindings/"

# Check Web UI (HTML, CSS, JavaScript, TypeScript types)
check-web-ui:
    #!/usr/bin/env bash
    set -e

    # `npm ci` (not `install`): it installs exactly what package-lock.json
    # pins, so a lint failure is always our code and never a dependency that
    # drifted underneath us. Skipped entirely when node_modules is present —
    # which is what CI's cache restores.
    if [ ! -d "node_modules" ]; then
        echo "Installing Node.js dependencies..."
        npm ci
    fi

    echo "=== Web UI lint ==="
    npm run lint:html
    npm run lint:css
    npm run lint:js

    echo ""
    echo "=== TypeScript type check ==="
    npm run typecheck

# C++ packages to lint/format
cpp_packages := "src/play_launch_container"

# Format code (Rust + Python + C++)
format:
    #!/usr/bin/env bash
    (cd src/play_launch && cargo +nightly fmt)
    (cd src/ros-launch-resolve/parser && cargo +nightly fmt)
    ruff format python/
    ament_clang_format --reformat {{cpp_packages}}

# Run format and check
quality:
    #!/usr/bin/env bash
    set -e
    echo "Running format..."
    just format
    echo "Running checks..."
    just check

# Run Autoware planning simulation using the project build
run-autoware *ARGS:
    #!/usr/bin/env bash
    set -e
    source tests/fixtures/autoware/activate_autoware.sh
    source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export DISPLAY=${DISPLAY:-:1}
    ros2 run play_launch play_launch launch \
        --web-addr 0.0.0.0:8080 \
        {{ARGS}} \
        autoware_launch planning_simulator.launch.xml \
        map_path:="$HOME/autoware_map/sample-map-planning" \
        vehicle_model:=sample_vehicle \
        sensor_model:=sample_sensor_kit

# Profile play_launch during Autoware execution (default DDS config)
profile-autoware:
    scripts/profile_autoware.sh

# Profile play_launch with optimized CycloneDDS config
profile-autoware-tuned:
    scripts/profile_autoware.sh --dds-tuned

# Check version consistency across all files
version-check:
    python3 scripts/bump_version.py --check

# Bump version (patch, minor, or major)
bump-version TYPE:
    python3 scripts/bump_version.py {{TYPE}}

# Set explicit version
set-version VERSION:
    python3 scripts/bump_version.py --set {{VERSION}}
