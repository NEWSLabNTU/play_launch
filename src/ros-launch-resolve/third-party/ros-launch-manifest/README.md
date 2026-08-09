# ros-launch-manifest

Static checking and runtime monitoring contracts for ROS 2 launch files.

A **launch manifest** is a sidecar YAML file that describes what a launch
file contributes to the communication graph: nodes, topics, services,
and timing contracts. Where a launch file says *what to run*, the manifest
says *what communicates and at what quality*.

## Documentation

- **[Launch Manifest Specification](docs/launch-manifest.md)** — the
  manifest format: elements, background, worked examples, format
  reference, and validation rules.
- **[Contract Theory](docs/contract-theory.md)** — formal foundations:
  latency composition, drop budgets, burstiness detection, empirical
  contract derivation.
- **[Design Issues](docs/design-issues.md)** — open and resolved design
  questions with rationale.

## Crate Structure

| Crate    | Description                                                         |
|----------|---------------------------------------------------------------------|
| `types/` | Manifest data types, YAML parser, substitution, condition filtering |
| `check/` | Static validation rules (14 rules), diagnostic emitter              |

## Quick Start

```bash
# Check a manifest against a launch tree
play_launch check --manifest-dir ./manifests <pkg> <launch_file>

# Check a single manifest standalone
play_launch check --manifest-dir ./manifests --standalone <manifest.yaml>
```
