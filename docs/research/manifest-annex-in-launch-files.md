# Annexing the Manifest into Standard ROS 2 Launch Files

**Date:** 2026-07-15 · **Status:** research note (feasibility answered empirically)

## Question

Can manifest content (the per-launch-file contract YAML, today a sidecar at
`<manifest-dir>/<pkg>/<file>.yaml`) be embedded *inside* a standard ROS 2
launch file **without breaking the stock `ros2 launch` parser**?

## Experiment

Four embedding strategies were fed to the stock launch frontend
(`get_launch_description_from_any_launch_file`, ROS 2 Humble):

| # | strategy | example | stock parser result |
|---|---|---|---|
| 1 | **XML comment payload** | `<!-- play_launch:manifest\nversion: 1\nnodes: … -->` | ✅ **PARSED OK** (1 entity) |
| 2 | **XML processing instruction** | `<?play_launch-manifest version: 1 ?>` | ✅ **PARSED OK** (1 entity) |
| 3 | unknown element | `<manifest><topic …/></manifest>` | ❌ `RuntimeError: Unrecognized entity of the type: manifest` |
| 4 | unknown attribute on a known tag | `<node … manifest_tier="control"/>` | ❌ `ValueError: Unexpected attribute(s) found in `node`: {'manifest_tier'}` |

(Python launch files are trivially safe — arbitrary module-level data is
invisible to the launch system — but the interesting case is XML.)

## Answer

**Yes — via XML comments (or processing instructions). No — via elements or
attributes.** The stock XML frontend validates both tag names and attribute
sets strictly, so any "native-looking" annex breaks `ros2 launch`. Content the
XML parser is *required to ignore* — comments and PIs — passes through
untouched.

## Implications for an implementation

- **Recommended carrier: a fenced XML comment** with a magic prefix, e.g.

  ```xml
  <launch>
    <!-- play_launch:manifest
    version: 1
    nodes:
      talker: { pub: { chatter: { min_rate_hz: 1 } } }
    topics:
      chatter: { type: std_msgs/msg/String, rate_hz: 1 }
    -->
    <node pkg="demo_nodes_cpp" exec="talker" name="talker"/>
  </launch>
  ```

  Comments are human-readable/editable in place, survive every conforming XML
  parser, and diff cleanly next to the entities they describe. PIs work too
  but are more likely to be stripped by formatters and are hostile to
  multi-line YAML.

- **Extraction is a raw-text concern, not a DOM concern.** `xml.etree`
  (used by launch_xml) and most default parser configurations *drop* comments
  and PIs, so the annex must be recovered from the raw file: scan for
  `<!-- play_launch:manifest` … `-->` and hand the payload to the existing
  `parse_manifest_str`. Both our parsers can do this cheaply (the Rust parser
  reads the file anyway; quick-xml can also surface comment events).
- **YAML inside an XML comment: two syntactic hazards.** `--` is forbidden
  inside XML comments (so no `--sched`-style strings, and YAML documents must
  avoid `--- ` document markers), and a literal `-->` would terminate the
  comment. A conservative rule — indent the payload and forbid `--` in it —
  or a PI (which only forbids `?>`) sidesteps both. Worth a validation rule in
  the extractor.
- **Precedence must be defined**: embedded manifest vs sidecar file when both
  exist (suggest: sidecar wins + warn, so a repo-wide `--manifest-dir` stays
  authoritative).
- **Scope mapping stays natural**: the annex in a launch file *is* that file's
  scope manifest — exactly what the sidecar lookup keys on today, so
  `manifest_loader` gains one alternative source, nothing else moves.
- **Not applicable to `system.toml`**: the scheduling spec is deliberately
  system-level (one per deployment), not per-launch-file; embedding it in a
  launch file would re-couple scheduling to launch structure. The annex idea
  fits the *manifest* (per-scope contracts) only.

## Verdict

Feasible and cheap: comment-fenced YAML, extracted by raw-text scan in
`manifest_loader` as an alternative to the sidecar. Stock `ros2 launch`
compatibility is preserved by construction (verified above). Not scheduled —
candidate for a future manifest phase.
