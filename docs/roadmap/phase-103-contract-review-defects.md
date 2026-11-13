# Phase 103 - defects the grammar review found

**Status (2026-11-13): planned.** The review of the contract grammar (rlm
`docs/design/grammar-revision/`, ten topics) found defects in what ships
today. None needs a grammar change, and two are silent, one of them
fail-unsafe, so they are fixed before the revision itself (phase 104).

## Who does what

rlm is the runtime-agnostic specification: the grammar, the SystemModel, and
what each key and rule means. play_launch is the Linux realizer: it resolves
launch trees, checks them, and observes the running system. nano-ros is the
RTOS realizer: it builds the model into a board image. Each unit names its
owner; where a meaning is involved, rlm states it first and both realizers
follow, because three of these defects exist precisely because each side
implemented a meaning rlm never wrote down.

The static resolver (play_launch's layer 2, built by nano-ros too) is
realizer-neutral; a fix there reaches both realizers.

## Units

| Unit | Defect | rlm | play_launch | nano-ros | Size |
|---|---|---|---|---|---|
| **D1** | A predicate naming a message constant (`not_equals: MANUAL`) is never judged at run time, silently: the model carries the name, not the value. A value function reads as held and `safe-state-violated` never fires. 22 fixtures and the island's `driver_took_over` (T8). | Model: `Constant { constant, value }`, value optional with a serde default. | Resolver fills the value from the `.msg`; the observer compares it; an unresolved constant is a warning, not silence. A runtime test where a constant predicate fires. | Tag bump (no reader of predicates today). | S |
| **D2** | A predicate's value is never checked against the field's type; a mismatch is silently unjudged at run time (T8). | Rule text. | `value_rules.rs`: a predicate value whose kind does not fit the field's type is an error, beside `commands-field-unknown`. | - | S |
| **D3** | Criticality derivation reads only the chain form of a hazard's reaction, so a hazard that enters a mode credits no integrity to its reacting nodes: 15 of 18 resolvable models differ, the island and rt_av_demo among them (T3). | - | Resolver: derive through the same ladder resolution the reaction walk uses. Every changed `node_criticality` is reviewed, and the ranked-plan snapshot is updated only for those. | Picks it up through the resolver. | M |
| **D4** | "Entering a mode" means three things: the checker walks `fallback`, the Linux observer and `measure` use the last level, `supervise.enters` and nano-ros's `ftti_budget::reaction_path` use the mode's own `reaction` (T3). | **Decision and normative text first** (see Decisions). | Checker, observer, `measure` and `supervise.enters` follow it. | `ftti_budget::reaction_path` and its observer rows follow it. | M |
| **D5** | The default of an omitted `on:` has three definitions: rlm's doc says every class, play_launch's message says omission, late and loss, nano-ros differs again (T7). | **Decision**, the doc comment and the field table row. | The message and the runtime observer. | The image's default. | S |
| **D6** | An `independence` entry naming a node absent from this model passes clean; an absent absolute node key is accepted with no diagnostic (T10). | Rule text. | `independence.rs`: an absent node is `independence-unchecked`, naming it. | - | S |
| **D7** | A reaction route through an action is refused (`reaction-unreachable`) and the fallback then drops the settle: 200 ms reported against a true 307 ms. An action client written under `cli:` parses (T9). | Parser: refuse an action type under `cli:`/`srv:`, naming `actions:`. | Fallback keeps the settle and names the action as the reason the route stops. | - | S |
| **D8** | Repetition and resequencing are judged on the sender's publish only, and a monitored topic without a stamp is never judged for them, with no warning (T7). | Doc: what the classes detect on each realizer. | A static warning when `on:` asks for them on an unstamped type; the guide says they detect an application re-sending, not a channel fault. | Its own statement of what the image observes. | S |
| **D9** | Messages and text: a literal `100Hz` under a `_hz` key is refused as "a quoted string -- remove the quotes"; a timer rate error names `paths.<p>.rate_hz`; `paths.<p>.trigger` still lists `spontaneous`; `key-inventory.md` lists 16 island contracts where the tree has 1 (T4, T6). | All four, plus `check-terms` over the field table's live rows (not its `removed` rows). | - | - | S |

D1, D2 and the rlm half of D7 and D9 go in one rlm release; nano-ros bumps
once for it.

## Decisions needed

- **D4, entering a mode.** The candidates: (a) the checker's meaning:
  entering mode M walks M's `fallback` from its first level, and the time
  charged is each level's route; (b) the mode's own `reaction` is what runs
  on entry, and `fallback` is consulted only when M itself is lost. Only the
  shape `{ fallback: [self], reaction: X }` agrees with all three readings
  today. The topic file `T3.md` has the evidence.
- **D5, the omitted `on:`.** Every class (strictest, rlm's current text)
  or the timing classes only (omission, late, loss; what play_launch says).
  Every class makes `repetition` fire on a state topic that re-sends a
  stamp, which T7 found on Autoware's gear and operation-mode topics.

## Acceptance

- D1: a value function whose predicate names a constant is lost at run time
  when the constant matches, and the island's `driver_took_over` is judged.
- D3: the island's and rt_av_demo's reacting nodes carry the integrity of
  the hazard that enters their mode.
- D4, D5: one sentence in rlm per meaning, cited by both realizers' code.
- Every unit: a test that fails before the fix (a negative control).
