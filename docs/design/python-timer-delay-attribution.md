# Attributing a Python `TimerAction`'s delay

## The problem

`<timer period="N">` on the XML and YAML frontends is easy: the traverser
walks the timer's body itself, so everything appended to the member lists
while that body was being walked is, by construction, inside the timer. That
is all `traverser::delay` does — take a mark, walk, stamp everything past the
mark.

The Python frontend cannot make that argument. A `.launch.py` is *executed*,
and the mock classes capture on construction. Python evaluates

```python
TimerAction(period=3.0, actions=[Node(package='p', executable='e')])
```

inside-out: the `Node` is built, and therefore captured, **before**
`TimerAction.__new__` is ever called. By the time the timer exists its
children are already sitting in the capture lists, indistinguishable from
every other node the file built.

So the nodes were never the casualty — only the delay was. The model came
back with the right members and they all started at once, which is worse than
a visible omission: `dump` and `check` both looked clean.

## What was rejected, and why

The first pass at this deliberately did **not** try to recover the delay. Its
reasoning: the only apparent handle is *capture order* — assume a timer's
children are the last N captures pushed. That is wrong the moment a node is
built outside the argument list and passed in by name:

```python
delayed   = Node(..., name='delayed')     # captured FIRST
immediate = Node(..., name='immediate')   # captured SECOND
return LaunchDescription([immediate, TimerAction(period=3.0, actions=[delayed])])
```

Here the last capture is the node that must **not** be delayed. An
order-based rule attaches the 3 s to `immediate` and starts `delayed` at t=0 —
both nodes wrong, silently. A delay on the wrong node is worse than one
reported missing, so that pass instead carried the loss across the dlopen
boundary (`ExecCaptures.unsupported`, loader ABI 6) and made `check` refuse
such a file.

That conclusion was right about capture order. It was wrong that capture order
is the only handle.

## What is sound: object identity

`TimerAction.__new__` is handed the **actual mock objects**, not copies and
not names. If each capturing mock records which captures its own constructor
appended, a timer can walk its `actions` and union exactly those. Nothing
about where the node was built, what it was called, or what order the file
built things in can affect the answer.

Concretely (`crates/pyexec/src/api/delay.rs`):

- `Node`, `LifecycleNode`, `ComposableNodeContainer` and
  `LoadComposableNodes` each take a mark around their own capture call and
  store the resulting `CaptureSpan` — the index range they appended to the
  node / container / load-node lists. This is the traverser's own argument,
  scoped to one constructor call: within a single constructor nothing else
  runs, so everything appended is that object's.
- `TimerAction` (and `launch_ros`'s `RosTimer`) walks `actions` by identity,
  recursing through `GroupAction`, `LaunchDescription`, nested timers, and
  plain lists/tuples, unions the spans, deduplicates, and adds the period.
- Nested timers **add**, as they do in ROS 2 and in the XML path: the inner
  timer has already stamped its children, and the outer takes the inner's set
  whole and adds onto it.

The capture indices are stable within one execution: the lists are only
appended to, and `process_launch_arguments` re-resolves substitutions in
place.

## What is still not attributable — and is reported, not guessed

Four shapes remain. None of them is guessed at; each produces a
`dropped_actions` entry naming the affected members, which is what `check`
refuses on.

### 1. The same action object in two unrelated timers

```python
shared = Node(..., name='shared')
LaunchDescription([TimerAction(period=2.0, actions=[shared]),
                   TimerAction(period=5.0, actions=[shared])])
```

`ros2 launch` executes that action once per timer, so the process starts
twice, at 2 s and at 5 s. This model holds the node **once**. Summing to 7 s
is nonsense and silently picking one is the failure this whole change is
about, so the first timer's delay stands and the second is refused by name.
Detection is by ownership: each capture records the timer that claimed it,
and a claim from a timer outside that one's nesting family is the conflict.

### 2. The same object both inside a timer and started directly

```python
both = Node(...)
LaunchDescription([both, TimerAction(period=4.0, actions=[both])])
```

Same root cause: two starts, one modelled member. The delayed start is kept
and the double start is reported. This is detected in the executor's walk,
which knows whether it reached a node through a timer or not.

### 3. A `period` that is not a number while the file is read

`period` may be a substitution, and it is resolved the way `<timer
period="$(var d)">` is — eagerly, through the substitution machinery. If it
does not come out as a number, there is no delay to attribute and the nodes
are named as starting immediately in the model.

Note this is a *different* problem from the one in
`docs/guide/parser-features.md` ("A second case: a lazily-evaluated `<timer
period>`"): ROS 2 resolves `period` when the timer *fires*, so a period this
parser resolves happily may still raise `SubstitutionFailure` at run time and
drop the children entirely. Eager resolution is the same choice the XML
frontend makes; it is not a claim that the timer will fire.

### 4. A child this parser cannot see into

An `IncludeLaunchDescription` under a timer is replayed by the traverser from
its own context, outside any timer, so its nodes do not inherit the delay.
Likewise a class this parser has no mock for. Both are named in the
diagnostic along with the period that was lost.

`OpaqueFunction` is deliberately **not** in this list: its nodes do not exist
when the timer is constructed, so identity cannot reach them — but the
executor's walk can. `visit_entity` now descends into a timer, takes a mark,
visits the body (running any `OpaqueFunction`), and stamps whatever the visit
appended. That is the traverser's argument again, and it is sound for the same
reason. Before this the walk did not enter a timer at all, so those nodes were
missing from the model outright. A timer holding an `OpaqueFunction` that the
walk never reaches is reported once the walk is over.

## What this does not change

- **A condition on a timer.** `TimerAction(condition=UnlessCondition(...))`
  that evaluates false should contribute nothing, but its children captured
  themselves on construction and are in the model regardless. That is a
  general property of capture-on-construction — the same is true of a
  conditioned `GroupAction` — not a timer problem, and it is unchanged here.
- **The loader ABI.** It stays at 6. The new field on the wire
  (`start_delay_secs` on the captures) predates ABI 6 and is serde-defaulted,
  and the pairing that would matter — a new loader with an old object — is
  not silent: an ABI-6 object reports every timer in `unsupported`, so `check`
  still refuses. A bump would only break a working installed pair.
- **A composable under a timer.** The delay is carried, but a composable is
  loaded with its container rather than on a deferred load call; the model's
  `meta.diagnostics` says so by name. Unchanged.

## Where the tests are

`crates/play_launch_parser/tests/python_tests.rs`, the block headed
"`TimerAction` on the Python frontend". They are written so that anything
order-based fails them: in every positive case the timer's children are
captured *before* the node that must not be delayed.
