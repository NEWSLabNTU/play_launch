---
id: 44
title: "the manifest spec documents a capture mode (`--save-manifest-dir`) that no binary accepts"
status: resolved
type: correctness
severity: low
---

# 0044 - a documented workflow with no flag behind it

**Repo:** `play_launch` at `af0aeab1`, and the `ros-launch-manifest` spec.
**Affects:** `docs/launch-manifest.md` (capture-mode section);
`src/play_launch/src/cli/options.rs`; `src/ros-launch-resolve/cli/src/options.rs`

## Symptom

The specification describes a capture workflow in which a run writes the
manifest it observed, so an author can start from a real system rather than
from a blank file. The flag it names is `--save-manifest-dir`.

```
$ grep -rn "save-manifest-dir\|save_manifest_dir" --include="*.rs" src/
$
```

No binary accepts it, and no code writes a manifest from a run.

## Impact

Small but specific: the capture workflow is the answer to "how do I get a
first contract for a system that has none", which is the question a new user
asks first. The documented answer does not exist, and the nearest real thing
(`play_launch check --emit diagnostics-params`, which prints
`diagnostic_updater` parameters from declared bounds) solves a different
problem.

`play_launch measure` is the closest shipped relative — it turns a recorded
run into a platform-file `overrides:` fragment, and phase 70 W5 extended it to
print `min_latency` floors as contract-shaped comments. So the machinery for
"observe a run, emit contract text" exists; it just does not emit a manifest.

## Fix direction

One of:

- **Implement it** on the back of `measure`: the interceptor already records
  every publisher and subscription created (`interception/endpoints.tsv`,
  phase 77) plus per-message events, which is enough to emit `nodes:`,
  `topics:` with wiring, and observed rates. The honest output marks every
  number as observed rather than required, since a measurement is a fact and
  a contract states requirements — the same distinction `measure` already
  draws by printing costs as comments under a header saying where they belong.
- **Delete the section**, and point instead at `measure` plus the worked
  example in the spec.

Either way the spec must stop naming a flag that does not exist. Phase 77's
`scripts/verify_graph.py` is prior art for the first option: it already grades
a model against `endpoints.tsv`, so the inverse direction is a short step.

## Provenance

Found 2026-09-23 during the v0.1.39 documentation pass, by two independent
agents reading the spec against the CLI.

---

## Resolution (2026-09-24)

Both halves, taken in the order the issue offered them: the capture was
implemented as a script, and the spec's remaining claim was pinned down. The
second half turned out to be mostly already fixed by the v0.1.39–41 tags, which
landed after this issue was filed — so what follows is what is *left*, checked
against the pinned tag rather than against the report.

### What was built

`scripts/capture_manifest.py <run-dir> --model <model.yaml>` prints a contract
file to stdout. Phase 77's `scripts/verify_graph.py` is the prior art and the
inverse walk over the same data; the two share their argument handling, their
infra-topic exclusion set and their node-join rule. Guide:
`docs/guide/first-contract.md`.

It reads, under `play_log/<ts>/interception/`:

| file | for |
|---|---|
| `endpoints.tsv` | the wiring. Every publisher and subscription CREATED, remap-resolved. Complete |
| `node_identity.tsv` | model key → real ROS node name, for the #0017 comment |
| `stats_summary.json`, `frontier_summary.json` | the message TYPE, and the observed rate |

**It emits** `version: 1`, `nodes:` with `pub:`/`sub:` endpoint maps, and
`topics:` with `type:` and both sides of the wiring.

**It emits no requirement of any kind** — no `rate_hz`, `min_rate_hz`,
`max_latency`, `max_age`, `max_jitter`, `paths:` or `criticality`. Observed
counts and rates go in as comments on the topic they describe, under a header
saying a measurement is a fact about one run and a contract states
requirements. Same discipline `measure` already follows.

**It emits no conditions**, and the header says why: a run is ONE branch of the
launch file with every `if:`/`unless:` already resolved, nothing on disk records
which way each went, and a contract that should mirror the launch file's
structure therefore cannot be recovered whole from a run. Stated so that nobody
diffs a capture against a hand-written contract and reads the missing
conditions as a disagreement.

**It refuses rather than guesses**, listing each refusal with its reason at the
end of the file: a topic whose type is not on disk, endpoints on a node the
model does not carry, and subscribers cut to break a causal cycle. A topic with
one side empty IS emitted, with a comment — `check` then reports the
`dangling-entity` warning whose own wording ("may be published by an external
system") is the correct verdict, since a run cannot tell an external publisher
from a node that failed to start.

### The gate

Run end to end against four real bundles produced in this session
(`tests/fixtures/simple_test`, interception on, `play_launch` from
`install/`), capture → `play_launch check <launch> --contracts <overlay>`:

| launch file | exercises | check verdict |
|---|---|---|
| `remapped_topic.launch.xml` | a remap | 1 clean, **0 errors, 0 warnings**, exit 0 |
| `unnamed_node.launch.xml` | #0017: model key ≠ ROS name | 1 clean, **0 errors, 0 warnings**, exit 0 |
| `all.launch.xml` | container + 2 composables, 7 nodes, a topic with no subscriber, a topic with no traffic | 1 clean, **0 errors**, 1 warning, exit 0 |
| a synthetic 3-node cycle | `causal-dag` | 1 clean, **0 errors**, 1 warning, exit 0 |

The one warning in each of the last two is a true statement about the run
(`/group_test/chatter` genuinely had no subscriber; `/ab` lost its subscriber to
the cycle cut).

### Three findings worth keeping

**1. The message type is on disk, but only for topics that carried a message.**
`endpoints.tsv` has **no type column**. `find_type_identity`
(`src/play_launch_interception/src/introspection.rs:95`) resolves
`pkg/msg/Name` and it reaches disk in two places: `stats_summary.json` and
`frontier_summary.json` carry `msg_type` per topic (phase 42.0), and
`discovered_topic_types.tsv` is written by the RuleEngine — which only exists
when contracts are already in play, so it is circular for this purpose and
unusable here. Since the summaries are keyed by traffic, **a created-but-never-
exercised endpoint has no type**, and `type:` is mandatory in the grammar (a
topic without one is a parse error that drops the WHOLE file). Phase 77
measured 982 endpoints created and 63 carrying a message on one Autoware run,
so on a real system the capture emits the exercised fraction and refuses the
rest by name. **Adding a type column to `endpoints.tsv` would close this**: the
init hooks already hold the type support, which is where `find_type_identity`
reads from. That is the one code change that would materially improve this
script, and it belongs to whoever owns `src/`.

**2. A bare node key can silently name a node that does not exist.** Absolute
FQN keys pass through the checker verbatim; a bare name is reconciled through
the launch dump's `node_identity`, and for a node the launch file did not name
that lookup MISSES and falls back to naive `scope.ns + name` qualification.
Measured: a contract keyed `talker-1` against `unnamed_node.launch.xml`
resolved to **`/talker-1`** — not `/identity_test/talker-1` —
and `check` reported `1 clean, 0 errors`. `--export-graph` showed the wrong
FQN. The script therefore always emits absolute keys. The cost is narrow and
documented: `is_state_endpoint` splits an endpoint ref on its FIRST slash, so
`state: true` has no effect on an absolute ref; vertex lookup and every other
consumer match the ref exactly.

**3. A confounded A/B nearly shipped a wrong claim, again.** The first arm
testing whether `causal-dag` fires on absolute keys appeared to show it inert,
and the header was written to say so. It was wrong: the control arm's
`topics:` referenced an endpoint the `nodes:` block no longer declared, so no
vertex resolved and no edge existed to be cyclic — the cycle was not what
differed between the arms. Re-run properly, `causal-dag` errors on a cycle with
absolute keys, and `state: true` does *not* silence it (it does silence
`causal-dag-global`). Same shape as the stale-submodule misreading in
CLAUDE.md: a comparison is only as good as the thing held constant.

### The doc half

Checked against the pinned tag (**v0.1.41**, `aa6752c`), not against the
report. Most of it was already fixed:

- `docs/launch-manifest.md` — the "Generating Manifests from a Running System"
  section, and with it the `--save-manifest-dir` flag, was **removed** in
  `2fcb4d5`. **Nothing to do.**
- `docs/contract-theory.md` Appendix C — already carries an accurate
  *Implementation status* paragraph saying capture mode is "designed but not
  implemented; there is no CLI flag for it" and pointing at `measure`.
- `docs/slides.md:372` — already lists capture mode as "not implemented".

What the spec must now be changed to say, all of it in the
`ros-launch-manifest` repo, which this repo does not own:

1. **`docs/design-issues.md` #32 is stale and is the last place the dead flag
   is named.** Its resolution text still reads "Added 'Generating Manifests
   from a Running System' section to launch-manifest.md with
   `--save-manifest-dir` usage", describing a section that no longer exists.
   It should say instead: the section was removed; the flag was never
   implemented and never will be under that name; what exists is
   `play_launch measure` for costs and `scripts/capture_manifest.py` in the
   `play_launch` repo for structure.

2. **`docs/contract-theory.md` Appendix C should be narrowed, not deleted.**
   Its statement is now half wrong in the useful direction: the STRUCTURE half
   of capture is implemented, and the statistical half (§ "Capture mode derives
   contracts from observed traces", the `α` margin and the confidence bound) is
   not — and on the ruling this script follows, should not be. Deriving
   `max_latency` and `min_rate_hz` from observed values times a safety margin
   *manufactures a requirement out of a measurement*, which is the one thing a
   contract must not contain. The appendix should keep the mathematics as the
   description of a technique, and say plainly that the tool does not apply it:
   observed values are emitted as comments for a human to promote.

3. **`docs/slides.md`** can move capture mode from "open items" to shipped-in-
   part, with the same split: structure captured, requirements deliberately
   not.

### Should it be a verb?

Not yet, and the reason is the type gap (finding 1). A verb implies the output
is complete; today the output is complete only for the traffic a run happened
to exercise. The order is: add a type column to `endpoints.tsv` from the init
hooks, confirm a capture of a real Autoware run emits the whole graph rather
than the exercised fraction, then promote it — most plausibly as
`play_launch check --emit contract-skeleton`, which sits beside the existing
`--emit diagnostics-params` and needs no new verb at all. Until then a script
is the honest shape: `scripts/verify_graph.py` is the same kind of tool at the
same maturity, and they are a pair.

### Files

- `scripts/capture_manifest.py` — new
- `docs/guide/first-contract.md` — new

## Promoted 2026-09-25 — `play_launch contract capture`

The resolution above said "not yet a verb: the type gap means the output is
complete only for traffic the run exercised, and a verb implies completeness".
That gap closed the same week (#0047 put the message type on `endpoints.tsv`,
where it had been in scope at the hook and discarded), so the condition was
met and the script is now `play_launch contract capture <run-dir> --model
<m.yaml>`, implemented in layer 2 so both CLIs get one body.

**Under `contract`, not as a thirteenth verb, and not the `check --emit` this
issue proposed.** `check` takes a launch file and contracts and has no run
bundle, so `--emit` would have needed a `--run-dir` the verb does not
otherwise take — and `check`'s exit code is a CI gate that a bundle-shaped
mode muddies. `measure --emit` fits the INPUTS exactly and was still rejected:
`measure` bails when the model declares no `paths:`, which is precisely the
state of a system with no contract, so the flag would have to bypass its
host's own precondition; its artifact is a platform-file fragment with a
different destination; and a structure capture MEASURES NOTHING, so emitting a
contract under that name implies the one thing this tool exists to refuse.
`contract` is the verb whose output is a contract file to edit — `eject` gets
a package's own, `capture` writes a first one. Group by the artifact produced,
not by the input read.

`scripts/capture_manifest.py` is deleted rather than kept as a wrapper: a
second implementation of the same output reading the same files drifts
silently, which is this repository's most frequently relearned lesson.
`scripts/verify_graph.py` stays and the guide now says why rather than leaving
the pair silently split — it grades a derivation during development, where the
capture answers a question a user asks on day one. It is the remaining
promotion candidate, not an oversight.

Gate: five real bundles captured and fed back through `check`, all exit 0 —
including the one that proves #0047's capability rather than asserting it, a
node whose endpoints are created and never published on, whose topics appear
in `endpoints.tsv` with types and in no summary at all. The cycle-cut refusal
has a negative control: re-adding the cut subscriber by hand turns `check`
into `1 with errors`, `causal-dag-global`, the cycle named hop by hop.

One thing the port ADDED, forced by a real bundle: an `<executable>`'s model
key is its whole command line, so endpoint refs are quoted inside `pub:`/`sub:`
flow lists. The script emitted them bare, and a key containing a comma would
have split one ref into two rather than failing.
