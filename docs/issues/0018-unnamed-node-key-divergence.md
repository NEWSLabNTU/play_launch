---
id: 18
title: "The two parsers give un-named nodes different model keys, and the reference one is the unstable one"
status: resolved
type: correctness
severity: high
---

# 0018 — un-named node keys diverge between parsers

**Repo:** `play_launch`
**Affects:** `structure.nodes` keys, `python_dump/visitor/node.py`,
`model_builder.rs` (`insert_node`)
**Related:** #0017 (model FQN is not the ROS graph name)

## The divergence

Same launch file, two parsers, different keys:

```xml
<group>
  <push-ros-namespace namespace="probe"/>
  <node pkg="demo_nodes_cpp" exec="talker"/>
  <node pkg="demo_nodes_cpp" exec="talker"/>
  <node pkg="demo_nodes_cpp" exec="listener" name="explicitly_named"/>
  <node pkg="demo_nodes_cpp" exec="talker"/>
  <node pkg="demo_nodes_cpp" exec="listener"/>
</group>
```

| rust (before) | rust (after) | python |
|---|---|---|
| `/probe/talker` | `/probe/talker-1` | `/probe/talker-1` |
| `/probe/talker#2` | `/probe/talker-2` | `/probe/talker-2` |
| `/probe/explicitly_named` | `/probe/explicitly_named` | `/probe/explicitly_named` |
| `/probe/talker#3` | `/probe/talker-3` | `/probe/talker-4` |
| `/probe/listener` | `/probe/listener-1` | `/probe/listener-5` |

Neither is the node's ROS name (#0017). Both are synthetic keys, and they are
different synthetic keys.

**Where each comes from.** Python takes `exec_name=node.name`
(`python_dump/visitor/node.py:181`), and `launch_ros.Node.name` is
`ExecuteProcess`'s **console process label** — `<executable>-<global process
index>`, counting named nodes in the index, which is why `3` is missing above.
Rust uses the executable name, and `model_builder.rs`'s `insert_node` appends
`#2`, `#3` … only on an actual key collision.

## Why the usual rule does not settle it

The project rule is "when Rust and Python differ, Python is correct", because
Python *is* the ROS 2 reference implementation. That rule does not apply here:
`talker-1` is not a ROS 2 semantic. It is a label `launch` prints in front of
console output so a human can tell two processes apart. It was never an
identity, and ROS 2 assigns nothing by it — the node's actual name remains its
compiled-in default either way.

So this is a choice about *our* key, not about reproducing ROS 2 behaviour.

## Measured: the reference scheme is the unstable one

Add one unrelated node at the top of the group — the kind of edit that should
not touch the identity of anything below it:

| parser | identities preserved |
|---|---|
| rust (`#N`) | **5 / 5** — only the new node appears |
| python (`-N`) | **2 / 5** |

Python renumbers everything downstream: `talker-1`→`talker-2`,
`talker-2`→`talker-3`, `talker-4`→`talker-5`, `listener-5`→`listener-6`. Its
index is global and positional, so any insertion anywhere earlier shifts every
un-named node after it.

Rust's ordinal is per colliding key, so it only moves when duplicates of the
*same* executable in the *same* namespace change.

Both are deterministic for an unchanged file — verified, three consecutive
resolves of the golf cart stack are **byte-identical** (144 keys, same order,
same md5). There is no parallel scan to destabilise: no `rayon`/`par_iter`
anywhere in the parser or resolver, and the only `thread::spawn` is the Python
bridge's I/O pump. `structure.nodes` is an order-preserving `IndexMap`
specifically to keep launch-traversal order (manifest issue 0382).

## Decision

**Python keeps its scheme.** It is ported from ROS launch code and reflects the
original behaviour; that is worth more than symmetry, and changing it would put
this parser's output further from the reference implementation rather than
closer.

**Rust keeps its stable per-key ordinal and adopts the hyphen**, so the two at
least share a surface form. `model_builder.rs`'s `insert_node` now produces
`/probe/talker-1`, `/probe/talker-2`, `/probe/talker-3` instead of
`talker`/`#2`/`#3`.

An un-named node is numbered **from `-1` even when it is the only one**. The
suffix is what marks a key as executable-derived rather than authored — exactly
the set whose key is not the node's ROS name (#0017) — so the form says so
instead of leaving `/probe/talker` looking like a name somebody chose. A node
the launch file DID name keeps its bare key.

## What makes the names stable

Three separate properties, each enforced by a test rather than observed:

**1. The same input always gives the same keys.** This is structural, not luck.
`dump.node` / `dump.container` / `dump.load_node` are `Vec`s in parser order,
`index.topics` is a `BTreeMap`, and `structure.nodes` is an order-preserving
`IndexMap` (manifest issue 0382). Nothing in the insertion path iterates a
`HashMap` — whose order Rust randomises per process, which would shuffle every
ordinal between runs of the same file. `the_same_dump_always_produces_the_same_keys`
builds the same dump nine times and compares; it fails if someone reintroduces a
`HashMap` there or adds a parallel pass over the records. Confirmed end to end
as well: three consecutive resolves of the golf cart stack are byte-identical.

**2. An unrelated edit does not renumber anything.** The ordinal counts
collisions of *this* key, so nodes with a different executable or namespace are
invisible to it — as is an interleaved *named* node, which is why the probe's
`explicitly_named` does not consume a number the way it does under Python's
global process index. `an_earlier_insertion_does_not_renumber_later_nodes`
inserts a node at the top and asserts every prior identity survives.

**3. Adding a duplicate of the SAME key does shift the ones after it.** No
positional ordinal can avoid this, and pretending otherwise would be the
dangerous part, so it is asserted rather than left to be discovered:
`inserting_a_same_key_duplicate_does_shift_later_ordinals`. The remedy available
to an author is `name=`, which takes the node out of the numbered set entirely.

So "stable whenever the launch file is not changed" is guaranteed by (1);
"stable when the launch file is changed elsewhere" by (2); and (3) is the
documented boundary.

`-` is as safe a discriminator as `#` was: `rclpy.validate_node_name` rejects
both (`node name must not contain characters other than alphanumeric`), so a
synthetic `talker-2` can never be confused with a node the launch file
explicitly named `talker-2`. Verified, and pinned by a test that asserts the
builder keeps all three instances when a hand-built dump contains exactly that
collision.

Two properties are locked by tests in `model_builder.rs`:

- `unnamed_duplicates_get_hyphen_ordinals` — the ordinal counts collisions of
  the key, so an interleaved *named* node does not consume a number (unlike
  Python's global process index, where it does).
- `an_earlier_insertion_does_not_renumber_later_nodes` — every prior identity
  survives an unrelated insertion.

Not done, and left open: `insert_node` pushes a
`structure: duplicate node instance … kept as …` diagnostic that nothing
surfaces prominently. Promoting it would tell an author that identity here
rests on file order and that `name=` would fix it. Worth doing; not required by
the decision.

### Blast radius

**17 of the golf cart's 144 keys change** — every un-named node gains `-1`
(`/sensing/lidar/falcon/seyond_node` → `…/seyond_node-1`, and so on). That is
the same 17 identified in #0017, which is not a coincidence: they are precisely
the nodes whose key was never their ROS name. Node count is unchanged and the
stack still brings up cleanly.

Anything that referenced those keys by hand has to move with them: a platform
file's `overrides:`, a manifest's endpoint or path keys, `play_launch context
--node <FQN>`, and per-node log directories. Nothing in-tree did.

Unaffected by design: the phase-50 **member id** (`kind:/ns/name#N`,
`member_id.rs`) is a different identifier used in REST paths and the web UI, and
keeps its own `#`. `split_model_fqn` splits on `/` only, so nothing parses the
ordinal separator.

## Caveat blocking verification on the real corpus

The Python parser cannot parse the golf cart stack at all:

    Error: Failed to call dump_launch main()
    Caused by: KeyError: ''

so cross-parser parity is currently unverifiable on the launch file this came
from. That failure deserves its own issue.

The golf cart model contains **zero** duplicate keys today, so the divergence is
latent there — it bites the first time two un-named nodes of the same executable
land in one namespace.

## Reproducing

`tmp/naming/dup.launch.xml` in the working tree; resolve with
`--parser rust` and `--parser python` and compare `structure.nodes` keys.
