# Phase 54: Parameter source ordering — one ordered list, ROS-faithful

**Status:** ✅ Complete (2026-07-27) — 54.1–54.5 landed; nano-ros pin bump follows in that repo.
**Fixes:** issue 0007 — inline `<param>` always wins over a later
`<param from=>`, diverging from ROS 2.
**Design:** issue 0007 §"Verified against ROS 2" + §"A vs B, decided"
(option A: one ordered `param_sources` vec; `params`/`params_files` retained
as derived views).

## The invariant this phase installs

ROS 2 has ONE ordered parameter-source list per node
(`launch_ros/actions/node.py`: `parse_nested_parameters` builds it in document
order; `execute` materializes each dict into a temp file and emits
`--params-file`/`-p` in list order; rcl applies them left-to-right, later
wins). Global params precede the node's own.

After this phase the same is true here, end to end: parse → record → model →
spawn, with no step able to represent "which kind wins" because kinds no
longer have precedence — only position does.

## Work items

- **54.1 — parser: stop forking.** `actions/node.rs` currently splits
  `<param>` children into `parameters` / `param_files` inside a walk that is
  already in document order. Replace with a single ordered
  `Vec<ParamSource>`; `ParamSource::{Inline{name,value}, File(path)}`.
  Resolution of substitutions happens per element, unchanged.
  **Done when:** a launch with inline→file→inline yields three ordered
  elements; existing param tests still pass. ✅ DONE —
  `param_sources_preserve_document_order`.

- **54.2 — record: `param_sources`.** `NodeRecord` gains the ordered vec.
  `params` / `params_files` stay as SERDE-DERIVED views (computed on
  serialize) so every existing reader and every committed record fixture keeps
  working. Global params land at the HEAD of the list (mirroring ROS's
  `params_container` loop running before `parameters`).
  **Done when:** record round-trips; `compare_models.py` parity unchanged.
  ✅ DONE — generator resolves inline values, stores file CONTENT (matching
  `params_files`), expands launch temp params files into Inline entries
  (Python-dumper parity), globals at the head.

- **54.3 — spawn: emit in list order.** `execution/node_cmdline.rs` walks
  `param_sources` and emits one `--params-file` per file element; consecutive
  INLINE runs coalesce into one temp YAML chunk (a run has no internal
  ordering conflict) and are emitted at the run's position — not forced last.
  The current `overrides.yaml`-always-last behavior goes away.
  **Done when:** the regression fixture below produces `a == 2`. ✅ DONE —
  `ordered_params_files` built by walking `param_sources` (one chunk file per
  source, consecutive inline runs coalesced); rendered in list order;
  `overrides.yaml`-last gated to the legacy path so older records are
  unchanged. Also fixed a latent bug: `params_files` was a `HashSet`, so even
  file-vs-file order was nondeterministic — the ordered path is a `Vec`.
  NOTE: play_launch mirrors the record in `ros/launch_dump.rs` (the same layer
  that dropped `machine` in nano-ros #236) — the field had to be added there
  too; the exhaustive destructure in `from_node_record` caught it.

- **54.4 — model: `NodeInstance.param_sources`.** Shared `model` crate gains
  the ordered vec; `params`/`params_files` become derived views for one
  release so nano-ros keeps building against either pin. Document in the type
  that a dict is a FILE in ROS's execution model — a bake-time consumer must
  fold it with file semantics (`ros__parameters` sections, wildcards), never
  as a bare key/value overlay.
  **Done when:** `resolved_params` (nano-ros side) can fold the list in order;
  model golden round-trip green. ✅ DONE — rlm `c4683d4`:
  `NodeInstance.param_sources` + `ParamSource::{Inline,File}`; when the list is
  non-empty `resolved_params` folds it IN ORDER and ignores the legacy split
  entirely (folding both would restore "inline always wins"). Pre-54 models,
  which carry only the split view, resolve exactly as before. The per-file
  merge was factored into `merge_param_file` so both paths share one matcher.
  Wired on the play_launch side in BOTH directions: `model_builder` lowers the
  record's list into the model, and `node_record_from_instance` carries it back
  onto the record — without the second half a spawn driven from a MODEL rather
  than a fresh dump silently kept the old bug.

- **54.5 — regression fixture + tests.** A launch with
  `<param name="a" value="1"/>` then `<param from="a_is_2.yaml"/>`, asserted
  on BOTH paths: the resolved model (later file wins → 2) and the spawned
  cmdline (the file appears after the inline chunk). Plus the mirror case
  (file then inline → inline wins) so the fix is not just an inversion.
  ✅ DONE, in three places:
  - cmdline (`node_cmdline`): `ordered_params_files_render_in_list_order_and_suppress_overrides`
    + `legacy_path_still_renders_overrides_last`.
  - model fold (rlm `params_projection`): `inline_then_file_lets_the_file_win`,
    `file_then_inline_lets_the_inline_win` (the mirror case),
    `ordered_list_shadows_the_legacy_split_views`,
    `empty_ordered_list_falls_back_to_the_legacy_split`,
    `file_sources_fold_left_to_right`.
  - lowering (`model_builder`): `param_sources_lower_in_order_and_the_later_file_wins`
    — record → model → `resolved_params` end to end.

## Non-goals

- `ParameterFile(allow_substs=…)` — ROS re-resolves substitutions inside a
  param file on request; our model carries content verbatim. Recorded as a
  related gap in issue 0007, not fixed here.
- Within-file section precedence (`/**` before more specific) is rcl's job on
  the spawn path and already implemented in nano-ros's bake matcher; this
  phase must not regress it, but does not change it.

## Rollout

Additive first: 54.1–54.2 land the ordered vec while the derived views keep
old readers green; 54.3 flips the spawn path to consume it; 54.4 propagates to
the shared model crate (pin bump on the nano-ros side follows). The two
`params`/`params_files` views are removed in a later release once no consumer
reads them.

## Follow-on (other repo)

nano-ros vendors the same `model` crate: bump its
`packages/cli/third-party/ros-launch-manifest` pin to `c4683d4` so the
compile-time bake folds in ROS order. No nano-ros code change is needed —
`resolved_params` is the seam, and it now prefers the ordered list itself.

## Known-unrelated breakage seen while testing (fixed)

`src/play_launch/src/member_actor/mod.rs`'s module doctest failed to compile:
it used `MemberCoordinator`, a type the builder/runner split replaced.
Pre-existing on main, untouched by this phase; rewritten against the real API
(`MemberCoordinatorBuilder::new().spawn(None)` → `(MemberHandle, MemberRunner)`,
events read off the runner).
