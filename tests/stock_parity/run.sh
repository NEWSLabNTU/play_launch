#!/usr/bin/env bash
# Differential check: play_launch resolve vs stock `launch`/`launch_ros`.
#
#   tests/stock_parity/run.sh                 every case under cases/
#   tests/stock_parity/run.sh subs ns         the named cases
#   tests/stock_parity/run.sh --file <launch_file> [name:=value ...]
#                                             one arbitrary launch file
#
# Stock is resolved by stock_oracle.py, which runs a real LaunchService with
# process spawning and composable-node loading recorded instead of performed,
# so no ROS process is ever started. Needs ROS sourced; uses this repository's
# build (install/) ahead of any installed play_launch.
#
# A case is a directory holding parent.launch.{xml,yaml,py} and optionally:
#   ARGS             launch arguments, one line, name:=value ...
#   EXPECTED_DIFFS   known differences, one per line (prefix match on the
#                    compare.py output line; '#' lines are comments)
#   EXPECT_ERROR     both sides must refuse the file
#
# Exits 1 if any case has an unexpected difference or an unexpected failure.

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
OUT="${STOCK_PARITY_OUT:-$REPO_ROOT/tmp/stock_parity}"
mkdir -p "$OUT"

if ! python3 -c 'import launch_ros' 2>/dev/null && [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
    set -u
fi
if [ -f "$REPO_ROOT/install/local_setup.bash" ]; then
    # colcon's setup scripts read unset variables.
    set +u
    # shellcheck disable=SC1091
    source "$REPO_ROOT/install/local_setup.bash"
    set -u
    export PATH="$REPO_ROOT/install/play_launch/lib/play_launch:$PATH"
fi
command -v play_launch >/dev/null || { echo "play_launch not found: build first (just build)"; exit 2; }
python3 -c 'import launch_ros' 2>/dev/null || { echo "launch_ros not importable: source ROS first"; exit 2; }

# Cases read these to exercise $(env) and <unset_env>.
export PL_PARITY_SET=set_value
unset PL_PARITY_UNSET

# compare one launch file; $1 = label, $2 = file, rest = launch args.
# Prints the diff lines to stdout; returns compare.py's status, or 3 when a
# side failed (its first error line is printed).
compare_one() {
    local label="$1" file="$2"; shift 2
    local model="$OUT/$label.model.yaml" stock="$OUT/$label.stock.json"
    rm -f "$model" "$stock"
    local pl_ok=1 st_ok=1
    play_launch resolve "$file" "$@" -o "$model" >"$OUT/$label.resolve.log" 2>&1 || pl_ok=0
    STOCK_OUT="$stock" python3 "$HERE/stock_oracle.py" "$file" "$@" >"$OUT/$label.stock.log" 2>&1
    local rc
    rc=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['rc'])" "$stock" 2>/dev/null || echo crash)
    [ "$rc" = "0" ] || st_ok=0
    echo "pl_ok=$pl_ok st_ok=$st_ok" >"$OUT/$label.status"
    if [ $pl_ok = 0 ] || [ $st_ok = 0 ]; then
        [ $pl_ok = 0 ] && echo "play_launch FAILED: $(grep -m1 -i error "$OUT/$label.resolve.log" | cut -c1-300)"
        [ $st_ok = 0 ] && echo "stock FAILED (rc=$rc): $(grep -m1 -i 'error\|exception' "$OUT/$label.stock.log" | cut -c1-300)"
        return 3
    fi
    python3 "$HERE/compare.py" "$model" "$stock"
}

if [ "${1:-}" = "--file" ]; then
    shift
    file="$1"; shift
    compare_one "$(basename "$file" | tr . _)" "$file" "$@"
    exit $?
fi

cases=("$@")
[ ${#cases[@]} -eq 0 ] && mapfile -t cases < <(ls "$HERE/cases")

failed=()
for n in "${cases[@]}"; do
    dir="$HERE/cases/$n"
    file=$(ls "$dir"/parent.launch.* 2>/dev/null | head -1)
    [ -z "$file" ] && { echo "=== $n: no parent.launch.*"; failed+=("$n"); continue; }
    args=()
    [ -f "$dir/ARGS" ] && read -r -a args <"$dir/ARGS"
    out=$(compare_one "$n" "$file" "${args[@]}")
    status=$?
    if [ -f "$dir/EXPECT_ERROR" ]; then
        if grep -q '^pl_ok=0 st_ok=0$' "$OUT/$n.status"; then
            echo "=== $n: ok (both refuse, as expected)"
        else
            echo "=== $n: FAIL (expected both sides to refuse)"; echo "$out" | sed 's/^/  /'; failed+=("$n")
        fi
        continue
    fi
    if [ $status = 3 ]; then
        echo "=== $n: FAIL"; echo "$out" | sed 's/^/  /'; failed+=("$n"); continue
    fi
    # Drop expected differences; anything left (other than the summary) fails.
    unexpected=$(echo "$out" | grep -E '^(DIFF|EXTRA|MISSING)' | while IFS= read -r line; do
        known=0
        if [ -f "$dir/EXPECTED_DIFFS" ]; then
            while IFS= read -r pat; do
                case "$pat" in ''|'#'*) continue ;; esac
                [ "${line#"$pat"}" != "$line" ] && { known=1; break; }
            done <"$dir/EXPECTED_DIFFS"
        fi
        [ $known = 0 ] && echo "$line"
    done)
    if [ -n "$unexpected" ]; then
        echo "=== $n: FAIL"; echo "$unexpected" | sed 's/^/  /'; failed+=("$n")
    else
        echo "=== $n: ok $(echo "$out" | tail -1 | sed 's/^ *-- //')"
    fi
done

echo
if [ ${#failed[@]} -gt 0 ]; then
    echo "stock parity: ${#failed[@]} of ${#cases[@]} case(s) failed: ${failed[*]}"
    exit 1
fi
echo "stock parity: all ${#cases[@]} case(s) match stock (known differences in EXPECTED_DIFFS)"
