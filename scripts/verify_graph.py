#!/usr/bin/env python3
"""Verify a SystemModel's topic graph against a recorded run.

The model's graph is INFERRED: from contracts an author wrote, and (phase 76)
from the `~/input/` / `~/output/` remap convention. A recorded run is a
MEASUREMENT that can confirm or contradict every edge it claims.

Two sources, and they answer different questions:

  interception/endpoints.tsv   every publisher and subscription CREATED, from
                               the `rcl_publisher_init` /
                               `rcl_subscription_init` hooks. Preferred: an
                               endpoint exists whether or not anything ever
                               flows through it.
  interception/events.jsonl    every message PUBLISHED or TAKEN. A subscription
                               on a pipeline whose sensor is absent looks here
                               exactly like a subscription that does not exist,
                               so grading by traffic grades the run's coverage
                               rather than the graph.

Verdicts, per (node, direction, topic):

  confirmed     the model claims it, the run created it
  CONTRADICTED  the model claims the OPPOSITE direction for that node on that
                topic. The one verdict that is always a defect: a wrong
                direction invents a causal edge, and every downstream rule
                inherits it
  missing       the run created it, the model does not claim it. Under-coverage
                -- for a safety classification this fails OPEN
  unexercised   the model claims it, the run never created it. Not a defect on
                its own: a node that failed to start, or an endpoint created
                only on a condition the run did not meet, looks like this

Usage:
  scripts/verify_graph.py <run-dir> --model <model.yaml> [--json]
"""

import argparse
import json
import os
import sys
from collections import defaultdict

try:
    import yaml
except ImportError:
    sys.exit("verify_graph.py needs PyYAML (pip install pyyaml)")

# Topics every ROS node touches that no launch file describes. Comparing them
# only ever produces noise on both sides.
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


def load_endpoints(path):
    """Raw `(member, node FQN, direction, topic)` rows a run CREATED.

    Each line is `member<TAB>pid<TAB>node FQN<TAB>pub|sub<TAB>topic`. The
    member is the model key play_launch spawned the process under; the node FQN
    is the name the process registered. For a node the launch file did not name
    these differ (issue #0017), and under an isolated container every forked
    composable inherits the CONTAINER's member -- so neither alone identifies
    the node, and which one joins to the model is decided per row.
    """
    rows = set()
    with open(path) as f:
        for line in f:
            parts = line.rstrip("\n").split("\t")
            if len(parts) != 5:
                continue
            member, _pid, fqn, direction, topic = parts
            if direction in ("pub", "sub"):
                rows.add((member, fqn, direction, topic))
    return rows


def load_traffic(path):
    """`(node, direction, topic)` triples a run made messages on."""
    names = {}
    counts = defaultdict(int)
    unresolved = 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if rec.get("r") == "t":
                names[rec["h"]] = rec["n"]
            elif rec.get("r") == "e":
                h = rec.get("h")
                if h not in names:
                    unresolved += 1
                    continue
                direction = "pub" if rec.get("d") == "pub" else "sub"
                counts[(rec["n"], direction, names[h])] += 1
    return set(counts), unresolved


def node_of(entry, nodes):
    """A model endpoint is `<node FQN>/<path name>`; a node FQN has slashes of
    its own, so the split is by longest known-node prefix, never by rsplit."""
    best = None
    for n in nodes:
        if entry == n or entry.startswith(n + "/"):
            if best is None or len(n) > len(best):
                best = n
    return best if best is not None else entry.rsplit("/", 1)[0]


def load_model(path):
    with open(path) as f:
        model = yaml.safe_load(f)
    structure = model.get("structure") or {}
    nodes = set((structure.get("nodes") or {}).keys())
    topics = structure.get("topics") or {}
    contracts = model.get("contracts") or {}
    # An endpoint an author wrote is a FACT; everything else in the graph got
    # there by inference from the remap convention. The model does not label
    # topics with their provenance, but the contract endpoint tables are keyed
    # by the same `<node>/<path>` refs the wiring uses, so it is recoverable
    # exactly: a topic is authored if any endpoint on it was declared.
    declared_refs = set(contracts.get("pub_endpoints") or {})
    declared_refs |= set(contracts.get("sub_endpoints") or {})
    declared_refs |= set(contracts.get("topics") or {})

    claimed = set()
    inferred = set()
    for topic, spec in (topics or {}).items():
        spec = spec or {}
        refs = list(spec.get("pub") or []) + list(spec.get("sub") or [])
        if topic not in declared_refs and not any(r in declared_refs for r in refs):
            inferred.add(topic)
        for entry in spec.get("pub") or []:
            claimed.add((node_of(entry, nodes), "pub", topic))
        for entry in spec.get("sub") or []:
            claimed.add((node_of(entry, nodes), "sub", topic))
    return claimed, set(topics), inferred, nodes


def resolve_node(member, fqn, model_nodes):
    """Which of the two names the model can be joined on.

    The registered FQN is the ROS truth and wins when the model knows it. A
    node the launch file did not name registers a FQN the model cannot know, so
    the model key is the only join there.
    """
    if fqn in model_nodes:
        return fqn
    if member in model_nodes:
        return member
    return fqn


def flip(direction):
    return "sub" if direction == "pub" else "pub"


def grade(observed, claimed, model_nodes):
    observed_known = {e for e in observed if e[0] in model_nodes}
    confirmed = sorted(claimed & observed_known)
    contradicted = sorted(
        e for e in claimed
        if e not in observed_known and (e[0], flip(e[1]), e[2]) in observed_known
    )
    missing = sorted(observed_known - claimed)
    unexercised = sorted(
        e for e in claimed
        if e not in observed_known
        and (e[0], flip(e[1]), e[2]) not in observed_known
    )
    ungraded = sorted({e[0] for e in observed} - model_nodes)
    # A claim on a node that never appeared in the run is not evidence about
    # the inference at all -- the node did not start, or started and wired
    # nothing. Only a claim on a node that DID wire endpoints, none of them
    # this one, says the inferred name was wrong.
    ran = {e[0] for e in observed_known}
    absent = sorted(e for e in unexercised if e[0] not in ran)
    wrong_name = sorted(e for e in unexercised if e[0] in ran)
    return (observed_known, confirmed, contradicted, missing, unexercised,
            ungraded, absent, wrong_name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", help="a play_log/<ts> directory")
    ap.add_argument("--model", required=True)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--include-infra", action="store_true")
    ap.add_argument(
        "--traffic",
        action="store_true",
        help="grade against events.jsonl (messages) instead of endpoints.tsv",
    )
    args = ap.parse_args()

    run = args.run
    if os.path.isfile(run):
        run = os.path.dirname(os.path.dirname(run))
    idir = os.path.join(run, "interception")
    ep_path = os.path.join(idir, "endpoints.tsv")
    ev_path = os.path.join(idir, "events.jsonl")

    claimed, model_topics, inferred_topics, model_nodes = load_model(args.model)

    source = None
    unresolved = 0
    if os.path.exists(ep_path) and not args.traffic:
        rows = load_endpoints(ep_path)
        observed = {
            (resolve_node(m, f, model_nodes), d, t) for m, f, d, t in rows
        }
        source = ep_path
    elif os.path.exists(ev_path):
        observed, unresolved = load_traffic(ev_path)
        source = ev_path
        if not args.traffic:
            print(
                "note: no endpoints.tsv in this run -- grading against message\n"
                "      traffic instead, which reports an endpoint that never\n"
                "      carried a message as missing from the RUN, not the model.\n"
            )
    else:
        sys.exit(f"no endpoints.tsv or events.jsonl under {idir}")

    if not args.include_infra:
        observed = {e for e in observed if not is_infra(e[2])}
        claimed = {e for e in claimed if not is_infra(e[2])}

    (known, confirmed, contradicted, missing, unexercised, ungraded,
     absent, wrong_name) = grade(observed, claimed, model_nodes)

    missing_absent = [e for e in missing if e[2] not in model_topics]
    missing_side = [e for e in missing if e[2] in model_topics]
    inferred_claimed = [e for e in claimed if e[2] in inferred_topics]
    inferred_confirmed = [e for e in confirmed if e[2] in inferred_topics]
    inferred_contradicted = [e for e in contradicted if e[2] in inferred_topics]

    # When both sources are present, the traffic set is reported as the subset
    # of endpoints the run actually exercised -- coverage, not a verdict.
    exercised = None
    if source == ep_path and os.path.exists(ev_path):
        traffic, unresolved = load_traffic(ev_path)
        if not args.include_infra:
            traffic = {e for e in traffic if not is_infra(e[2])}
        exercised = len(traffic & known)

    result = {
        "source": source,
        "model": args.model,
        "observed_endpoints": len(known),
        "model_endpoints": len(claimed),
        "model_endpoints_inferred": len(inferred_claimed),
        "confirmed": len(confirmed),
        "contradicted": len(contradicted),
        "missing": len(missing),
        "missing_topic_absent_from_model": len(missing_absent),
        "missing_side_only": len(missing_side),
        "unexercised": len(unexercised),
        "unexercised_node_never_ran": len(absent),
        "unexercised_node_ran_endpoint_absent": len(wrong_name),
        "inferred_confirmed": len(inferred_confirmed),
        "inferred_contradicted": len(inferred_contradicted),
        "ungraded_nodes": len(ungraded),
        "unresolved_events": unresolved,
        "exercised_by_traffic": exercised,
    }

    if args.json:
        result["detail"] = {
            "contradicted": [list(e) for e in contradicted],
            "missing_topic_absent": [list(e) for e in missing_absent],
            "missing_side_only": [list(e) for e in missing_side],
            "unexercised_node_ran": [list(e) for e in wrong_name],
            "unexercised_node_never_ran": [list(e) for e in absent],
            "ungraded_nodes": ungraded,
        }
        print(json.dumps(result, indent=2))
        return 1 if contradicted else 0

    print(f"run   {source}")
    print(f"model {args.model}")
    print()
    print(f"  observed endpoints (on nodes the model knows) : {len(known)}")
    print(f"  model endpoints                               : {len(claimed)}"
          f"  ({len(inferred_claimed)} inferred, no contract)")
    print()
    print(f"  confirmed    {len(confirmed)}")
    print(f"  CONTRADICTED {len(contradicted)}")
    print(f"  missing      {len(missing)}"
          f"  ({len(missing_absent)} topic absent from the model,"
          f" {len(missing_side)} one side only)")
    print(f"  unexercised  {len(unexercised)}"
          f"  ({len(absent)} on nodes that never wired anything,"
          f" {len(wrong_name)} on nodes that did)")
    if inferred_claimed:
        print()
        print(f"  inferred edges: {len(inferred_confirmed)} confirmed,"
              f" {len(inferred_contradicted)} CONTRADICTED")
    if exercised is not None:
        print(f"\n  {exercised} of {len(known)} observed endpoints carried a"
              f" message during the run")
    if ungraded:
        print(f"\n  {len(ungraded)} node(s) wired topics but are absent from"
              f" the model (not graded):")
        for n in ungraded[:10]:
            print(f"    {n}")
        if len(ungraded) > 10:
            print(f"    ... and {len(ungraded) - 10} more")

    for title, items in (
        ("CONTRADICTED (the model has the direction backwards)", contradicted),
        ("missing: the topic is absent from the model", missing_absent),
        ("missing: the model has the topic, not this side", missing_side),
        ("unexercised, on a node that DID wire endpoints -- the strongest "
         "evidence the inferred name is wrong", wrong_name),
        ("unexercised, on a node that wired nothing at all -- says nothing "
         "about the inference", absent),
    ):
        if not items:
            continue
        print(f"\n{title}:")
        for node, direction, topic in items[:40]:
            mark = " [inferred]" if topic in inferred_topics else ""
            print(f"  {node}  {direction}  {topic}{mark}")
        if len(items) > 40:
            print(f"  ... and {len(items) - 40} more")

    return 1 if contradicted else 0


if __name__ == "__main__":
    sys.exit(main())
