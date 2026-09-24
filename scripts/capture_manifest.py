#!/usr/bin/env python3
"""Emit a launch-manifest contract's STRUCTURE from a recorded run.

The inverse walk of `scripts/verify_graph.py`. That one grades a model's
inferred topic graph against what a run created; this one writes the graph the
run created out as a contract file, so an author starting from a system with no
contract at all starts from its real wiring rather than a blank file.

What a run can say, and what it cannot
--------------------------------------

Structure and facts are observable. Requirements are not. A run can say a topic
published at 9.97 Hz; it cannot say whether 10 Hz was required, and a contract
states requirements. So this emits `nodes:`, their endpoints and `topics:` with
types and wiring -- and puts every measured number in as a COMMENT. It emits no
`rate_hz`, no `min_rate_hz`, no `max_latency`, no `max_age`, no `max_jitter`,
no `paths:` and no `criticality`. Same discipline `play_launch measure` follows
when it prints costs as comments under a header saying where they belong; you
will meet both.

It also emits no CONDITIONS. A run is ONE branch of the launch file, with every
`if:`/`unless:` already resolved, and nothing on disk records which way they
went. A capture of a system launched twice with different arguments is two
different captures, and diffing either against a hand-written contract that
carries conditions will show phantom differences. The emitted header says so.

What it reads, under `play_log/<ts>/interception/`
--------------------------------------------------

  endpoints.tsv        every publisher and subscription CREATED (phase 77),
                       with remap-resolved topic names. The wiring comes from
                       here and only here: an endpoint exists whether or not a
                       message ever flowed through it, while `events.jsonl`
                       records MESSAGES and a subscription on a pipeline whose
                       sensor is absent looks there exactly like one that does
                       not exist.
  node_identity.tsv    model key -> pid -> the REAL ROS node name (issue
                       #0017). A node the launch file did not name is keyed in
                       the model by its EXECUTABLE, which is not its ROS name;
                       the contract has to use the key the launch dump knows.
  stats_summary.json   per topic: `msg_type`, `pub_count`, `take_count`,
  frontier_summary.json  `avg_pub_rate_hz`. The ONLY place a message type
                       reaches disk -- `endpoints.tsv` has no type column --
                       and these files are keyed by traffic, so a topic that
                       carried no message has no type here and CANNOT be
                       emitted. Those are listed, with the reason, rather than
                       dropped.

Usage:
  scripts/capture_manifest.py <run-dir> --model <model.yaml> [> out.contract.yaml]

The model is required for the same reason `play_launch measure` requires one:
the contract's node keys have to be the keys the launch dump uses, and only the
model knows them.
"""

import argparse
import json
import os
import re
import sys
from collections import defaultdict

try:
    import yaml
except ImportError:
    sys.exit("capture_manifest.py needs PyYAML (pip install pyyaml)")

# Topics every ROS node touches that no launch file describes. Same set
# `verify_graph.py` excludes, for the same reason: comparing them only ever
# produces noise on both sides, and a contract that declared them would put
# `/rosout` and `/parameter_events` on all 144 nodes of a real system.
INFRA_TOPICS = {
    "/parameter_events",
    "/rosout",
    "/clock",
    "/tf",
    "/tf_static",
}
INFRA_SUFFIXES = ("/_container/component_events",)


def is_infra(topic):
    return topic in INFRA_TOPICS or topic.endswith(INFRA_SUFFIXES)


# ---------------------------------------------------------------------------
# Inputs
# ---------------------------------------------------------------------------


def load_endpoints(path):
    """`(member, node FQN, direction, topic)` rows a run CREATED.

    One line is `member<TAB>pid<TAB>node FQN<TAB>pub|sub<TAB>topic`. Both names
    travel because neither alone identifies the node: the member is the model
    key play_launch spawned the PROCESS under, and under an isolated container
    every forked composable inherits the container's member.
    """
    rows = set()
    malformed = 0
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if not line:
                continue
            parts = line.split("\t")
            if len(parts) != 5 or parts[3] not in ("pub", "sub"):
                malformed += 1
                continue
            member, _pid, fqn, direction, topic = parts
            rows.add((member, fqn, direction, topic))
    return rows, malformed


def load_identity(path):
    """`model key -> {real ROS FQN}`, from the interceptor's own report.

    Authoritative: read off the `rcl_node_t*` the publish/subscribe init hooks
    already hold, so there is no inference in it. One key maps to several names
    when one process hosts several nodes (a container).
    """
    identity = defaultdict(set)
    if not os.path.exists(path):
        return identity
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) == 3:
                identity[parts[0]].add(parts[2])
    return identity


def load_topic_facts(run_interception_dir):
    """`topic name -> {type, pub_count, take_count, rate_hz, duration_ms}`.

    Merged from `stats_summary.json` and `frontier_summary.json`, both keyed by
    topic hash with a `name` and an optional `msg_type`. The type is the load-
    bearing half: a topic without one cannot be emitted at all, because the
    manifest grammar requires `type:` on every topic entry.
    """
    facts = {}
    for fname in ("stats_summary.json", "frontier_summary.json"):
        path = os.path.join(run_interception_dir, fname)
        if not os.path.exists(path):
            continue
        try:
            with open(path) as f:
                blob = json.load(f)
        except (json.JSONDecodeError, OSError):
            continue
        for key, entry in (blob or {}).items():
            if key.startswith("_") or not isinstance(entry, dict):
                continue  # `_events_dropped_total_best_effort` and friends
            name = entry.get("name")
            if not name:
                continue  # a hash whose chunked name never assembled
            rec = facts.setdefault(name, {})
            if entry.get("msg_type") and "type" not in rec:
                rec["type"] = entry["msg_type"]
            for field in ("pub_count", "take_count", "avg_pub_rate_hz",
                          "duration_ms"):
                if field in entry and field not in rec:
                    rec[field] = entry[field]
    return facts


def load_model(path):
    """`(model node keys, un-named node keys, model path)`.

    `node_name: null` on a model node is the discriminator from issue #0017 --
    exactly the set whose model key is its EXECUTABLE and therefore NOT its ROS
    name. Those keys get a comment naming the real name the run reported.
    """
    with open(path) as f:
        model = yaml.safe_load(f) or {}
    nodes = ((model.get("structure") or {}).get("nodes")) or {}
    keys = set(nodes)
    unnamed = {k for k, v in nodes.items() if not (v or {}).get("node_name")}
    return keys, unnamed


def run_paths(arg):
    """Accept a run directory, its `interception/` dir, or a file inside it."""
    run = arg
    if os.path.isfile(run):
        run = os.path.dirname(run)
    if os.path.basename(os.path.normpath(run)) == "interception":
        idir = run
        run = os.path.dirname(os.path.normpath(run))
    else:
        idir = os.path.join(run, "interception")
    return run, idir


# ---------------------------------------------------------------------------
# Joining a run's endpoints onto the model's node keys
# ---------------------------------------------------------------------------


def resolve_node(member, fqn, model_nodes):
    """Which of the run's two names the CONTRACT may be written in.

    A contract's node key is reconciled against the launch dump, so it has to
    be a key the model carries. The registered FQN is the ROS truth and wins
    when the model knows it; a node the launch file did not name registers a
    FQN the model cannot know, and the model key is the only join there. When
    neither is a model node the endpoint is refused -- claiming it would put a
    node in the contract that the launch file does not produce.

    Same rule as `verify_graph.py::resolve_node`, minus its fallback: that one
    is grading and reports the mismatch, this one is WRITING and must not.
    """
    if fqn in model_nodes:
        return fqn, None
    if member in model_nodes:
        return member, fqn
    return None, None


def endpoint_names(per_node):
    """Pick one endpoint name per `(node, direction, topic)`.

    The endpoint name inside a node is arbitrary -- the `topics:` block does
    the wiring, by `<node key>/<endpoint>` ref -- so the only requirements are
    that it is unique within its node and contains no `/`, which would make the
    ref ambiguous to split. Last topic segment where that is unique, the whole
    flattened topic path where it is not, and a numeric suffix in the (unseen)
    case that two topics still flatten alike.

    Uniqueness is across BOTH directions, not per direction: a node that
    publishes and subscribes the same topic would otherwise give two endpoints
    one ref.
    """
    names = {}
    for node, entries in per_node.items():
        taken = set()
        # Sorted so the same run always produces the same file.
        for direction, topic in sorted(entries):
            base = re.sub(r"[^A-Za-z0-9_]", "_", topic.lstrip("/").split("/")[-1])
            if not base or base in taken:
                base = re.sub(r"[^A-Za-z0-9_]", "_", topic.lstrip("/"))
            candidate = base
            n = 2
            while candidate in taken or not candidate:
                candidate = f"{base}_{n}"
                n += 1
            taken.add(candidate)
            names[(node, direction, topic)] = candidate
    return names


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------


def break_cycles(by_topic):
    """Drop the fewest subscriber refs that leave the emitted graph acyclic.

    `causal-dag` is the one rule that fires at ERROR severity on a file this
    shape. The checker builds a vertex per declared endpoint and an edge for
    every (publisher, subscriber) pair on a topic, and refuses a cycle. Real
    systems have them -- Autoware's planning simulator closes the physical
    loop, and a controller reading back what it commanded is a feedback pair --
    so a capture that ignored this would routinely emit a file its own checker
    rejects, which is the failure this script exists to avoid.

    The contract's answer to a legitimate cycle is `state: true` on the
    subscriber that closes it, marking that edge as a stored value rather than
    a causal dependency. This script cannot use it, for two reasons and the
    second is the hard one:

      1. A RUN CANNOT TELL WHICH EDGE THAT IS. Both look identical on the wire.
         Guessing would invent a semantic every downstream rule inherits, where
         an omission only leaves the graph sparse -- the same ruling the topic
         inference makes about an undecidable remap direction.
      2. It would not work anyway. MEASURED on the pinned checker (v0.1.41):
         with the absolute node keys this file uses, `state: true` does NOT
         silence `causal-dag`, because `is_state_endpoint` splits an endpoint
         ref on its FIRST slash and so looks up a node named "". It does
         silence the cross-scope `causal-dag-global` warning, whose split is on
         the last slash. So on this file the flag is half-inert.

    The dropped refs are named in the refusal block, with the cycle they
    closed. Deterministic: topics are walked in sorted order and the first
    hop of the discovered cycle, by (topic, ref), is the one cut.
    """
    def node_of(ref):
        return ref.rsplit("/", 1)[0]

    dropped = []
    while True:
        edges = defaultdict(set)     # node -> {node}
        via = defaultdict(list)      # (src, dst) -> [(topic, sub ref)]
        for topic in sorted(by_topic):
            sides = by_topic[topic]
            for p in sorted(sides["pub"]):
                for s in sorted(sides["sub"]):
                    sp, sn = node_of(p), node_of(s)
                    if sp == sn:
                        continue  # a node reading its own output is not an edge
                    edges[sp].add(sn)
                    via[(sp, sn)].append((topic, s))

        cycle = find_cycle(edges)
        if cycle is None:
            return dropped
        hops = sorted(
            (t, ref, a, b)
            for a, b in zip(cycle, cycle[1:] + cycle[:1])
            for t, ref in via[(a, b)]
        )
        topic, ref, _a, _b = hops[0]
        by_topic[topic]["sub"].remove(ref)
        dropped.append((topic, ref, list(cycle)))


def find_cycle(edges):
    """Any directed cycle, as a list of nodes, or None. Iterative depth-first."""
    WHITE, GREY, BLACK = 0, 1, 2
    color = defaultdict(int)
    for root in sorted(edges):
        if color[root] != WHITE:
            continue
        stack = [(root, iter(sorted(edges.get(root, ()))))]
        path = [root]
        color[root] = GREY
        while stack:
            node, it = stack[-1]
            nxt = next(it, None)
            if nxt is None:
                color[node] = BLACK
                stack.pop()
                path.pop()
                continue
            if color[nxt] == GREY:
                return path[path.index(nxt):]
            if color[nxt] == WHITE:
                color[nxt] = GREY
                path.append(nxt)
                stack.append((nxt, iter(sorted(edges.get(nxt, ())))))
    return None


def yaml_key(name):
    """A topic or node key is a ROS name, which YAML reads as a plain scalar.

    Quoted anyway when it could be read as anything else -- an empty string, or
    a name whose characters would start a flow collection or a tag.
    """
    if name and re.fullmatch(r"[A-Za-z0-9_/~.:-]+", name) and not name.startswith("-"):
        return name
    return json.dumps(name)


def fmt_hz(v):
    return f"{v:.2f}" if v is not None else "?"


def header(run, idir, model_path, counts):
    return f"""\
# Captured from a run by scripts/capture_manifest.py -- STRUCTURE ONLY.
#   run:   {run}
#   model: {model_path}
#   {counts['endpoints']} endpoint(s) created, {counts['nodes']} node(s), \
{counts['topics']} topic(s) emitted
#
# This file states what the run WIRED. It states no requirement, because a run
# cannot observe one: a measured rate is a fact about one execution, and
# whether that rate is REQUIRED is a decision only you can record. So there is
# no `rate_hz`, no `min_rate_hz`, no `max_latency`, no `max_age`, no
# `max_jitter`, no `paths:` and no `criticality` below -- add them, and the
# file becomes a contract instead of a description. Observed numbers are here
# as comments, which is where they belong until you decide otherwise
# (`play_launch measure <run-dir> --model <m.yaml>` does the same for costs).
#
# It also states no CONDITIONS. A run is ONE branch of the launch file, with
# every `if:`/`unless:` already resolved and nothing on disk recording which
# way each went. Launch the same file with different arguments and you get a
# different capture; a contract that should mirror the launch file's structure
# has to have the conditions put back by hand. Do not read a diff against a
# hand-written contract as a disagreement until you have accounted for that.
#
# `type:` comes from the interceptor's introspection of a message that actually
# flowed. An endpoint that was created but never carried one has no type on
# disk, so its topic could not be emitted; those are listed at the end.
#
# Node keys below are ABSOLUTE FQNs, which the checker passes through verbatim.
# That is the only spelling guaranteed to name the node the run observed: a bare
# name is reconciled through the launch dump, and for a node the launch file did
# not name that lookup MISSES and the key silently resolves to a node that does
# not exist (measured: `talker-1` -> `/talker-1`, with `check` reporting clean).
# One rule reads an absolute ref wrongly and you should know which: the checker
# splits an endpoint ref on its FIRST slash when deciding whether `state: true`
# is set, so that flag has no effect here. Everything else -- vertex lookup
# included -- matches the ref exactly.
"""


def render(nodes, topics, refusals, run, idir, model_path):
    out = []
    counts = {
        "endpoints": sum(len(v) for v in nodes.values()),
        "nodes": len(nodes),
        "topics": len(topics),
    }
    out.append(header(run, idir, model_path, counts))
    out.append("version: 1\n")

    if nodes:
        out.append("\nnodes:")
        for node in sorted(nodes):
            out.append(f"  {yaml_key(node)}:")
            real = nodes[node].get("_real_name")
            if real:
                out.append(
                    f"    # The launch file did not name this node, so its model key is its\n"
                    f"    # EXECUTABLE and is NOT its ROS name (issue #0017). The run registered\n"
                    f"    # it as {real}. The key below is the one the contract\n"
                    f"    # must use -- it is the one the launch dump knows."
                )
            for direction in ("sub", "pub"):
                eps = nodes[node].get(direction) or {}
                if not eps:
                    continue
                out.append(f"    {direction}:")
                for ep in sorted(eps):
                    out.append(f"      # {eps[ep]}")
                    out.append(f"      {ep}: {{}}")
        out.append("")

    if topics:
        out.append("\ntopics:")
        for topic in sorted(topics):
            spec = topics[topic]
            out.append(f"  {yaml_key(topic)}:")
            for line in spec["comments"]:
                out.append(f"    # {line}")
            out.append(f"    type: {spec['type']}")
            out.append(f"    pub: [{', '.join(spec['pub'])}]")
            out.append(f"    sub: [{', '.join(spec['sub'])}]")
        out.append("")

    if refusals:
        out.append(
            "\n# ------------------------------------------------------------------\n"
            "# NOT emitted. Each of these is an observation this file cannot honestly\n"
            "# carry; none is a defect on its own.\n"
            "# ------------------------------------------------------------------"
        )
        for reason in sorted(refusals):
            items = sorted(refusals[reason])
            out.append(f"#\n# {reason} ({len(items)}):")
            for item in items[:60]:
                out.append(f"#   {item}")
            if len(items) > 60:
                out.append(f"#   ... and {len(items) - 60} more")
        out.append("")

    return "\n".join(out) + "\n"


# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(
        description="emit a launch-manifest contract's structure from a run",
    )
    ap.add_argument("run", help="a play_log/<ts> directory")
    ap.add_argument(
        "--model",
        required=True,
        help="the SystemModel the run was launched from -- supplies the node "
        "keys a contract has to be written in",
    )
    ap.add_argument(
        "--include-infra",
        action="store_true",
        help="keep /rosout, /parameter_events, /tf and friends, which every "
        "node touches and no launch file describes",
    )
    args = ap.parse_args()

    run, idir = run_paths(args.run)
    ep_path = os.path.join(idir, "endpoints.tsv")
    if not os.path.exists(ep_path):
        sys.exit(
            f"no endpoints.tsv under {idir}.\n"
            "The wiring can only come from the endpoints a run CREATED, and only\n"
            "the interceptor records those. Re-run with interception enabled\n"
            "(`interception: {enabled: true}` in --config) and try again."
        )

    model_nodes, model_unnamed = load_model(args.model)
    rows, malformed = load_endpoints(ep_path)
    identity = load_identity(os.path.join(idir, "node_identity.tsv"))
    facts = load_topic_facts(idir)

    refusals = defaultdict(set)
    if malformed:
        refusals[f"{malformed} unparseable line(s) in endpoints.tsv"].add(
            "a truncated last line is expected on a killed run"
        )

    # 1. Join every observed endpoint onto a model node key.
    observed = defaultdict(set)   # node key -> {(direction, topic)}
    real_names = {}               # node key -> registered ROS FQN, when different
    for member, fqn, direction, topic in rows:
        if is_infra(topic) and not args.include_infra:
            continue
        node, aka = resolve_node(member, fqn, model_nodes)
        if node is None:
            refusals[
                "endpoints on a node the model does not carry -- a process "
                "play_launch spawned can host nodes the launch file never named "
                "(a container's own node, an rclcpp-internal one), and a contract "
                "may not claim one"
            ].add(f"{fqn} (member {member})  {direction}  {topic}")
            continue
        observed[node].add((direction, topic))
        # The #0017 comment is only honest when the model key names exactly one
        # running node. A container's model key maps to every composable it
        # forked, and naming one of them "the real name of this node" would be
        # a false claim about the other four.
        if aka and node in model_unnamed and len(identity.get(node, ())) == 1:
            real_names[node] = aka

    if not observed:
        sys.exit(
            f"no endpoint in {ep_path} joins to a node in {args.model}.\n"
            "Either the run and the model are from different launches, or every\n"
            "endpoint was on a node play_launch did not spawn."
        )

    names = endpoint_names(observed)

    # 2. Group by topic, and refuse any topic whose type never reached disk.
    #    `type:` is mandatory in the grammar, so an untyped topic is not a
    #    degraded entry -- it is an unparseable file.
    by_topic = defaultdict(lambda: {"pub": [], "sub": []})
    for node, entries in observed.items():
        for direction, topic in entries:
            by_topic[topic][direction].append(
                f"{node}/{names[(node, direction, topic)]}"
            )

    cycle_cut = set()
    for topic, ref, cycle in break_cycles(by_topic):
        cycle_cut.add(topic)
        refusals[
            "subscriber endpoints CUT to break a causal cycle -- `causal-dag` "
            "refuses a cyclic dataflow graph outright (an error), and a cycle "
            "is legitimate only when one edge carries a stored value rather "
            "than a causal dependency (`state: true` on that subscriber). A run "
            "cannot tell which edge that is, and on this file's absolute node "
            "keys the flag would not take effect anyway -- the checker's "
            "`is_state_endpoint` splits a ref on its FIRST slash. Decide which "
            "edge is stored, shorten that node's key to a bare name, and put "
            "the edge back with `state: true`"
        ].add(f"{ref}  on {topic}   (cycle: {' -> '.join(cycle)} -> {cycle[0]})")

    topics = {}
    emitted_endpoints = set()
    for topic, sides in by_topic.items():
        fact = facts.get(topic) or {}
        msg_type = fact.get("type")
        if not msg_type:
            refusals[
                "topics whose message type is not on disk -- the type is read by "
                "introspection off a message that FLOWED, so an endpoint created "
                "but never exercised has none, and the grammar requires `type:` on "
                "every topic. Fill these in by hand (`ros2 topic info -v`), or "
                "re-run exercising them"
            ].add(
                f"{topic}  ({len(sides['pub'])} pub, {len(sides['sub'])} sub)"
            )
            continue
        comments = []
        if fact.get("pub_count") is not None:
            comments.append(
                f"observed: {fact.get('pub_count', 0)} published, "
                f"{fact.get('take_count', 0)} taken, "
                f"{fmt_hz(fact.get('avg_pub_rate_hz'))} Hz mean over "
                f"{fact.get('duration_ms', 0) / 1000.0:.1f} s. A FACT about this "
                f"run, not a requirement -- if a rate is required, say so with "
                f"`rate_hz:` and the tool will check it."
            )
        if not sides["pub"]:
            comments.append(
                "nothing in this run published it. Either a publisher outside "
                "the launch (`external: pub`) or a node that did not start -- "
                "the run cannot tell you which."
            )
        if topic in cycle_cut:
            comments.append(
                "a subscriber on this topic was CUT to break a causal cycle -- "
                "see the end of this file. The wiring below is deliberately "
                "less than what the run created."
            )
        elif not sides["sub"]:
            comments.append(
                "nothing in this run subscribed to it. Either a consumer "
                "outside the launch (`external: sub`) or one that did not start."
            )
        topics[topic] = {
            "type": msg_type,
            "pub": sorted(sides["pub"]),
            "sub": sorted(sides["sub"]),
            "comments": comments,
        }
        for direction in ("pub", "sub"):
            for ref in sides[direction]:
                emitted_endpoints.add((ref, direction, topic))

    # 3. Nodes carry only the endpoints whose topic survived step 2: an
    #    endpoint ref pointing at a topic that is not declared is a dangling
    #    reference, which is a worse output than an omission.
    nodes = defaultdict(dict)
    for node, entries in observed.items():
        for direction, topic in sorted(entries):
            ep = names[(node, direction, topic)]
            if (f"{node}/{ep}", direction, topic) not in emitted_endpoints:
                continue
            nodes[node].setdefault(direction, {})[ep] = f"-> {topic}"
        if node in nodes and node in real_names:
            nodes[node]["_real_name"] = real_names[node]
    nodes = {k: v for k, v in nodes.items() if any(
        d in v for d in ("pub", "sub"))}

    if not topics:
        sys.exit(
            f"nothing could be emitted from {run}: no topic in it has a message\n"
            "type on disk. The type is read by introspection off a message that\n"
            "FLOWED, so a run in which nothing published leaves none. Re-run with\n"
            "the system exercised."
        )

    sys.stdout.write(
        render(nodes, topics, refusals, run, idir, args.model)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
