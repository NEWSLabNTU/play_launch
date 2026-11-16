#!/bin/sh
# probe.sh LABEL [CMD...]
#
# Records what this process started IN (its working directory) and WITH (a
# few environment variables, "<unset>" when absent) into $PL_PROBE_OUT/LABEL,
# then runs CMD in its place, or sleeps. Used both as a plain process and as
# a launch-prefix, where CMD is the node the prefix wraps (#0085, #0087).
label=$1
shift
out="$PL_PROBE_OUT/$label"
{
    echo "pwd=$(pwd -P)"
    for v in DISPLAY PL_PROBE_INHERITED PL_PROBE_TOP PL_PROBE_PFX PL_PROBE_SET PL_PROBE_SHELL; do
        if eval "[ -n \"\${$v+x}\" ]"; then
            eval "echo \"$v=\$$v\""
        else
            echo "$v=<unset>"
        fi
    done
} >"$out.tmp" && mv "$out.tmp" "$out"
if [ $# -gt 0 ]; then
    exec "$@"
fi
exec sleep 600
