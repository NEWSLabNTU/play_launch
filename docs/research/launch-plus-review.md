# launch-plus Review

**Source**: https://github.com/paulsohn/launch-plus.git (cloned to `external/launch-plus/`)
**Date**: 2026-03-18

---

## What It Is

launch-plus is a **Bazel-like build system for ROS 2** that treats launch
files as build targets. It solves workspace scaling: instead of cloning and
building 200+ packages, it parses launch files statically to determine the
minimal set of packages needed, sparse-checks them out, and builds only
those.

**Not a runtime tool.** It operates entirely pre-execution — fetch, resolve,
build. It does not declare topics, QoS, or services. No runtime auditing.

## Architecture

```
.repos file ──→ Indexer ──→ lockfile (SHA-pinned)
                              │
launch file ──→ Resolver ─────┤──→ dependency graph
                              │     └──→ Fetcher (sparse-checkout)
                              │          └──→ Builder (colcon --packages-select)
                              │
                              └──→ Resolved XML (fully inlined, flattened)
```

Two embedded parsers:
- **XML Resolver** (Rust, `quick-xml`) — handles `.launch.xml`
- **Python Resolver** (Python shim, embedded via `include_str!`) — handles
  `.launch.py` by shimming `launch`/`launch_ros` imports to record
  constructor calls without requiring ROS 2 Python packages

## Key Features

| Feature | How |
|---------|-----|
| Lazy fetching | `git sparse-checkout` on demand; no full clone |
| Minimal builds | Only transitive build deps (excludes `exec_depend`) |
| Static analysis | Resolve without ROS install or building packages |
| SHA-pinned lockfile | Reproducible, VCS-compatible (valid `vcs import` input) |
| Python launch support | Shimmed imports — no ROS 2 Python needed |
| OpaqueFunction handling | Execute with patched filesystem; retry on missing packages |
| Portable output | Resolved XML uses `$(find-pkg-share pkg)/...` not absolute paths |
| Composable nodes | Full support for containers and `<load_composable_node>` |

## Lockfile Format

Dual-indexed: O(1) lookup by repo path OR package name.

```yaml
version: 1

repos:
  core/autoware_msgs:
    url: https://github.com/autowarefoundation/autoware_msgs.git
    version: abc123def    # pinned SHA
    packages: [autoware_msgs, autoware_planning_msgs]

packages:
  autoware_msgs:
    repo: core/autoware_msgs
    path: autoware_msgs   # relative path in repo
```

## Resolved XML Output

All `<include>` expanded, conditionals evaluated, variables substituted.
Portable paths preserved (`$(find-pkg-share)` not expanded).

```xml
<!-- resolved by launch-plus from autoware_launch://launch/autoware.launch.xml -->
<launch>
  <node pkg="topic_tools" exec="relay" name="trajectory_relay">
    <param name="input_topic" value="/planning/trajectory"/>
  </node>
  <node_container pkg="rclcpp_components" exec="component_container_mt"
                  name="pointcloud_container">
    <composable_node pkg="autoware_system_monitor" plugin="CPUMonitor"
                     name="cpu_monitor">
      <param name="usage_warn" value="0.96"/>
    </composable_node>
  </node_container>
</launch>
```

## Interesting Implementation Details

### Format-Agnostic Intermediate Representation

Both XML and YAML parsers emit the same `RawElement` tree, converted to
typed `LaunchElement` by a shared function:

```rust
pub struct RawElement {
    pub tag: String,
    pub attrs: HashMap<String, String>,
    pub children: Vec<RawElement>,
}
// Both XML parser and YAML parser → RawElement → raw_to_launch_elements()
```

This means adding a new launch format (TOML, etc.) only requires a new
parser that emits `RawElement` — validation and resolution are shared.

### Shimmed Python Imports

Python resolver intercepts `import launch_ros.actions` etc. via
`importlib.abc.MetaPathFinder`, replacing them with lightweight stubs that
record constructor calls. This means launch-plus can resolve Python launch
files **without any ROS 2 Python packages installed**.

### Fetch-and-Retry Loop

When the resolver encounters a package that hasn't been fetched yet, it
raises `_PackageNotFetchedError`. The orchestrator catches this, runs
`git sparse-checkout` for the missing package, and retries resolution.
This is dynamic dependency discovery — no static pre-analysis needed.

## Comparison to play_launch

| Aspect               | launch-plus                 | play_launch manifest                     |
|----------------------|-----------------------------|------------------------------------------|
| **Phase**            | Pre-execution (fetch/build) | Parse + runtime (audit)                  |
| **Input**            | `.repos` + launch files     | Launch files + manifest YAML             |
| **Output**           | Lockfile + resolved XML     | Expected graph in `record.json`          |
| **Topics**           | Not declared                | First-class: type + QoS                  |
| **QoS**              | N/A                         | Per-topic, audited at parse and runtime  |
| **Services/Actions** | N/A                         | First-class                              |
| **Timing contracts** | N/A                         | Per-node + per-chain (AUTOSAR TIMEX)     |
| **Reliability**      | N/A                         | Loss ratio, deadline misses              |
| **Sync policies**    | N/A                         | Correlation, tolerance, drop propagation |
| **RCL interception** | N/A                         | Frontier + stats + future graph plugin   |

**Complementary, not competing.** launch-plus ensures the right packages
are fetched and built. play_launch's manifest ensures the running system's
communication graph matches expectations.

## Ideas Worth Adopting

### 1. Dual-Indexed Lookup

launch-plus lockfile is indexed by both repo path and package name.

For our manifest directory, we already index by `<package>/<launch>.yaml`
which is effectively a dual index (package name + launch file name).
No change needed, but the principle is sound.

### 2. Portable Path Preservation

launch-plus preserves `$(find-pkg-share pkg)/...` in resolved output
instead of expanding to absolute paths. This makes output portable.

Our manifests use relative names (namespace-applied at parse time), which
achieves the same portability. Cross-namespace references use absolute
ROS topic names (e.g., `/sensing/pointcloud`), which are portable by
definition (they're the same on every machine).

### 3. Format-Agnostic IR

The `RawElement` intermediate representation lets launch-plus support
XML and YAML with shared validation logic.

Our manifest format is YAML-only. If we ever need to support multiple
manifest formats, this pattern would be useful. For now, YAML is
sufficient.

### 4. Resolved/Flattened Output

launch-plus can produce a fully-resolved, flattened XML with all includes
expanded and conditionals evaluated.

Our parser already does this — `record.json` is the fully-resolved output.
The expected graph (from manifests) extends this with topic information.
Similar philosophy.

## Not Applicable to Our Design

- **Lockfile/fetch/build** — play_launch assumes packages are already
  installed. We don't manage workspace dependencies.
- **Shimmed Python imports** — our parser already uses PyO3 to execute
  Python launch files with the real ROS 2 Python packages.
- **Fetch-and-retry loop** — our manifest-dir is pre-populated. No dynamic
  discovery needed.

## Conclusion

launch-plus confirms that **static analysis of launch files** is valuable
and tractable. Their resolved XML is analogous to our `record.json` — a
fully-evaluated, portable representation of the launch graph.

The gap launch-plus leaves open — no topic, QoS, service, or timing
information — is exactly what play_launch's topic manifest fills. The two
tools address different phases of the ROS 2 development lifecycle and could
complement each other.
