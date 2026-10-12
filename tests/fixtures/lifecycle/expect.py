#!/usr/bin/env python3
"""Assert a SystemModel's per-node lifecycle facts (phase 86).

    expect.py MODEL FQN=KIND...

KIND is `plain` (not a lifecycle node, no autostart), `lifecycle` (a
lifecycle node, no autostart) or `active` (a lifecycle node with
`lifecycle_autostart: active`). Parity alone cannot catch both parsers being
wrong the same way, which is how `<lifecycle_node>` went unmodelled; this
names the answer.
"""

import sys

import yaml

EXPECT = {
    "plain": (False, None),
    "lifecycle": (True, None),
    "active": (True, "active"),
}


def main() -> int:
    path, *pairs = sys.argv[1:]
    with open(path) as fh:
        nodes = yaml.safe_load(fh)["structure"]["nodes"]
    bad = []
    for pair in pairs:
        fqn, kind = pair.split("=", 1)
        want = EXPECT[kind]
        node = nodes.get(fqn)
        if node is None:
            bad.append(f"{fqn}: missing (have {sorted(nodes)})")
            continue
        got = (bool(node.get("lifecycle", False)), node.get("lifecycle_autostart"))
        if got != want:
            bad.append(f"{fqn}: (lifecycle, lifecycle_autostart) = {got}, want {want}")
    if bad:
        print(f"{path}:\n  " + "\n  ".join(bad))
        return 1
    print(f"{path}: {len(pairs)} node(s) as expected")
    return 0


if __name__ == "__main__":
    sys.exit(main())
