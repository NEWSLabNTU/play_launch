#!/usr/bin/env python3
"""Issue #0077: the environment each process of launch/set_env.launch.xml gets.

    expect_set_env.py model.yaml

The values are stock `ros2 launch`'s: a `<set_env>` reaches every process in
its scope, included files' too, and a process's own `<env>` wins.
"""

import sys

import yaml

GROUP = {"PL_SET_ENV_TOP": "top", "PL_SET_ENV_GROUP": "group"}
EXPECTED = {
    "own_env": {"PL_SET_ENV_TOP": "top", "PL_SET_ENV_GROUP": "own"},
    "in_group": GROUP,
    "in_xml_include": GROUP,
    "in_py_include": GROUP,
    "own_wins": {"PL_SET_ENV_TOP": "top", "PL_SET_ENV_GROUP": "own"},
    "after_group": {"PL_SET_ENV_TOP": "top"},
    "exec_in_scope": {"PL_SET_ENV_TOP": "top"},
}


def processes(model):
    """name -> effective env of every member carrying `env`."""
    out = {}

    def walk(o):
        if isinstance(o, dict):
            for key, value in o.items():
                if isinstance(value, dict) and "env" in value:
                    env = {}
                    for e in value["env"] or []:
                        env[e["name"]] = e["value"]
                    out[str(key).rstrip("/").rsplit("/", 1)[-1]] = env
                walk(value)
        elif isinstance(o, list):
            for value in o:
                walk(value)

    walk(model)
    return out


def main(path):
    with open(path) as f:
        got = processes(yaml.safe_load(f))
    bad = 0
    for name, want in EXPECTED.items():
        have = {k: v for k, v in got.get(name, {}).items() if k.startswith("PL_SET_ENV_")}
        if have != want:
            print(f"  FAIL {path}: {name}: {have}, expected {want}")
            bad += 1
    print(f"  {path}: {len(EXPECTED) - bad} of {len(EXPECTED)} processes as stock")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
