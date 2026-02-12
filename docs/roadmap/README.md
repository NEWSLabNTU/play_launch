# play_launch Roadmap

## Overview

This directory contains the implementation roadmap for play_launch, organized by development phase. Each phase has its own detailed documentation file.

## Goals

1. **Familiar UX**: ROS 2 users can use `play_launch` like native ros2 commands
2. **Transparent workflow**: Automatic dump-and-replay in a single command
3. **Flexible**: Support dump-only and replay-only modes for advanced users
4. **Production ready**: Robust error handling and comprehensive testing

---

## Current Status

**Overall Progress**: ~95% complete (16 of 17 phases complete, Phase 17 in progress)

**Timeline**: Started October 2025, ongoing development

---

## Development Phases

### Core Functionality

| Phase | Status | Completion | Documentation |
|-------|--------|------------|---------------|
| **Phase 1**: Core Subcommand Structure | ✅ Complete | 2025-10-20 | [phase-1.md](./phase-1.md) |
| **Phase 2**: dump_launch Integration | ✅ Complete | 2025-10-29 | [phase-2.md](./phase-2.md) |
| **Phase 2.5**: CLI Simplification | ✅ Complete | 2025-10-29 | [phase-2.5.md](./phase-2.5.md) |

### Documentation & Testing

| Phase | Status | Completion | Documentation |
|-------|--------|------------|---------------|
| **Phase 3**: Documentation & Polish | 🔄 Partial | - | [phase-3.md](./phase-3.md) |
| **Phase 4**: Testing & Validation | 🔄 Partial | - | [phase-4.md](./phase-4.md) |
| **Phase 5**: Optional Enhancements | ⏳ Planned | - | [phase-5.md](./phase-5.md) |

### Advanced Features

| Phase | Status | Completion | Documentation |
|-------|--------|------------|---------------|
| **Phase 6**: I/O Helper Integration | ✅ Complete | 2025-11-02 | [phase-6.md](./phase-6.md) |
| **Phase 7**: Logging Improvements | ✅ Complete | 2025-11-04 | [phase-7.md](./phase-7.md) |
| **Phase 8**: Web UI | ✅ Complete | 2025-12-18 | [phase-8.md](./phase-8.md) |
| **Phase 9**: Web UI Status Refactoring | ✅ Complete | 2026-01-17 | [phase-9.md](./phase-9.md) |

### Architecture Improvements

| Phase | Status | Completion | Documentation |
|-------|--------|------------|---------------|
| **Phase 10**: Async Actor Pattern Transformation | ✅ Complete | 2026-01-01 | [phase-10.md](./phase-10.md) |
| **Phase 11**: Web UI Actor Integration | ⏳ Planned | - | [phase-11.md](./phase-11.md) |
| **Phase 12**: Container-Managed Composable Nodes | ✅ Complete | 2026-01-13 | [phase-12.md](./phase-12.md) |

### Performance & Maintainability

| Phase | Status | Completion | Documentation |
|-------|--------|------------|---------------|
| **Phase 13**: Rust Parser Migration | ✅ Complete | 2026-01-27 | [phase-13.md](./phase-13.md) |
| **Phase 14**: Python Launch File Execution | ✅ Complete | 2026-01-31 | [phase-14-python_execution.md](./phase-14-python_execution.md) |
| **Phase 14.5**: Namespace Accumulation Bug Fixes | ✅ Complete | 2026-02-02 | [phase-14_5-namespace_accumulation_fixes.md](./phase-14_5-namespace_accumulation_fixes.md) |
| **Phase 15**: Python API Type Safety Improvements | ✅ Complete | 2026-01-31 | [phase-15-python_api_type_safety.md](./phase-15-python_api_type_safety.md) |
| **Phase 16**: YAML Parameter Loading & Global Parameters | ✅ Complete | 2026-02-06 | In README.md |
| **Phase 17**: Context Unification & Parser Parity | 🔄 In Progress | - | [phase-17-context_unification.md](./phase-17-context_unification.md) |

### Container Isolation

| Phase | Status | Completion | Documentation |
|-------|--------|------------|---------------|
| **Phase 18**: Code Quality Improvements | ✅ Complete | 2026-02-08 | [phase-18-code_quality.md](./phase-18-code_quality.md) |
| **Phase 19**: Clone-Isolated Component Manager | ⏳ Planned | - | [phase-19-isolated_container.md](./phase-19-isolated_container.md) |

---

## Progress Summary

### Completed Features ✅

- ✅ Subcommand-based CLI (`launch`, `run`, `dump`, `replay`)
- ✅ Automatic dump-and-replay workflow
- ✅ Configuration file support (moved fine-grained tuning out of CLI)
- ✅ Service readiness checking (enabled by default)
- ✅ Privileged process I/O monitoring (via CAP_SYS_PTRACE helper)
- ✅ Per-node and system-wide resource monitoring
- ✅ GPU monitoring (NVIDIA via NVML)
- ✅ Interactive Plotly visualization dashboard
- ✅ Logging verbosity control and node error detection
- ✅ Web-based management interface with live log streaming
- ✅ Node control (start/stop/restart) via Web UI

### In Progress 🔄

- 🔄 Context Unification & Parser Parity (Phase 17 - Started 2026-02-06)
  - ✅ Namespace propagation fixed (XML → Python context synchronization)
  - ✅ Root cause identified: Two incompatible context structures
  - ✅ Temporary workaround: Manual bidirectional synchronization
  - ⏳ Unified context design (LaunchContext + ParseContext → UnifiedContext)
  - ⏳ Remaining discrepancies: exec_name, global params, params_files, namespace in cmd
  - See [phase-17-context_unification.md](./phase-17-context_unification.md) for detailed roadmap

### Planned ⏳

- ⏳ Clone-Isolated Component Manager (Phase 19) - Consolidate container into single binary, per-node process isolation via `clone(CLONE_VM)`, crash detection, cgroups CPU control
- ⏳ Web UI Actor Integration (Phase 11) - Remove bridging layer, implement direct actor control
- ⏳ Complete documentation (Phase 3)
- ⏳ Comprehensive testing (Phase 4)
- ⏳ Optional enhancements (Phase 5)

### Recently Completed 🎉

- ✅ YAML Parameter Loading & Global Parameters (Phase 16 - Complete 2026-02-06)
  - ✅ Namespace normalization complete (all composable nodes have leading slashes)
  - ✅ YAML loading infrastructure complete
  - ✅ Global parameter merging complete (all three code paths)
  - ✅ XML composable node YAML loading complete (behavior_path_planner: 15 → 813 params)
  - ✅ Parser comparison test integrated into `tmp/run_all_checks.sh`
  - ✅ Executable path resolution implemented
  - ⚠️ Minor formatting differences remain (array spacing, string quoting)

- ✅ Web UI Status System Refactoring (Phase 9 - Complete 2026-01-17)
  - ✅ Distinct status types: NodeStatus, ComposableNodeStatus, UnifiedStatus
  - ✅ Categorized health metrics with separate counts for nodes, containers, and composable nodes
  - ✅ Container restart with automatic composable node reloading
  - ✅ Improved UX with proper status colors and control buttons per node type

- ✅ Composable Node Namespace Normalization (2026-02-04)
  - ✅ Fixed LoadNode service "Couldn't parse remap rule" errors in Autoware
  - ✅ All composable node namespaces now have leading slashes for RCL compatibility
  - ✅ Modified ComposableNode::capture_as_load_node in launch_ros.rs
  - ✅ Fixed 16 load_node entries (adapi/node → /adapi/node, etc.)
  - ✅ All namespace-related LoadNode errors resolved

- ✅ Parser Improvements & Testing Enhancements (2026-02-04)
  - ✅ Runtime substitution resolution for unresolved Python LaunchConfiguration objects
  - ✅ Fixed Autoware container startup issue (all 15 containers now start successfully)
  - ✅ Added `<let>` statement temporal ordering test validating parse-time resolution semantics
  - ✅ Converted all debug logging to proper log::debug!/trace! (13 statements cleaned up)
  - ✅ Test count increased to 310 (was 308)
  - ✅ 100% Autoware compatibility maintained

- ✅ Namespace Accumulation Bug Fixes (Phase 14.5 - Complete 2026-02-02)
  - ✅ Fixed critical namespace accumulation causing 389-char namespaces instead of 60-char
  - ✅ Implemented save/restore pattern for ROS_NAMESPACE_STACK (replaced push/pop)
  - ✅ Fixed XML include namespace inheritance to prevent double accumulation
  - ✅ Fixed GroupAction namespace leakage in Python list comprehensions
  - ✅ All Autoware nodes now start successfully (traffic_light_occlusion_predictor, simple_planning_simulator)
  - ✅ Rust parser output now matches Python parser 100% for namespaces
  - ✅ All 221 unit tests passing

- ✅ Python API Type Safety Improvements (Phase 15 - Complete 2026-01-31)
  - ✅ SetEnvironmentVariable accepts PyObject for name and value
  - ✅ AppendEnvironmentVariable accepts PyObject for name, separator, and prepend
  - ✅ ExecuteProcess accepts Vec<PyObject> for cmd and PyObject for cwd/name
  - ✅ Node.arguments accepts Vec<PyObject> for flexible argument handling
  - ✅ 5 comprehensive tests covering all fixes (plain strings, substitutions, lists)
  - ✅ All Phase 15 tests passing (test_set_environment_variable, test_append_environment_variable, test_execute_process, test_node_arguments, test_phase_15_combined)

- ✅ Python Launch File Execution (Phase 14 - Complete 2026-01-31)
  - ✅ Python files execute through ROS 2 launch system (not static analysis)
  - ✅ LaunchConfiguration resolution via `perform()` method
  - ✅ Global parameters captured from LAUNCH_CONFIGURATIONS
  - ✅ SetLaunchConfiguration and all launch actions supported
  - ✅ 15 Python tests + 25+ fixtures covering all major features
  - ✅ Autoware comparison passes (Rust vs Python parsers equivalent)
  - ⚠️ Parameter file paths captured (contents optional, not implemented)

- ✅ Rust Parser Migration (Phase 13 - Complete 2026-01-27)
  - ✅ Container record consolidation (only in container[], not node[])
  - ✅ Global parameters capture for XML-parsed nodes and containers
  - ✅ Simple test comparison passes (Rust vs Python parsers)
  - ✅ XML parsing achieves full parity with Python dump visitor

- ✅ Async Actor Pattern Transformation (Phase 10 - Complete 2026-01-01)
  - ✅ Actor infrastructure module created (Phase 10.1)
  - ✅ State machines defined (NodeState, ComposableState, ContainerState)
  - ✅ Event types defined (ControlEvent, StateEvent)
  - ✅ MemberCoordinator implemented
  - ✅ RegularNodeActor implemented (Phase 10.2)
  - ✅ Spawn logic extracted with respawn as direct loop
  - ✅ Integration tests for actor lifecycle and control
  - ✅ Migrated `run` command to actor pattern (Phase 10.3)
  - ✅ Event bridging for web UI compatibility
  - ✅ ContainerActor with supervision pattern (Phase 10.4)
  - ✅ ComposableNodeActor with LoadNode service integration (Phase 10.4)
  - ✅ Migrated `replay` command to actor pattern (Phase 10.5)
  - ✅ Cleanup and optimization - removed EventProcessor and ProcessMonitor (Phase 10.6)

---

## Dependencies

### External
- ROS 2 (Humble or Jazzy)
- `dump_launch` package (Python component, optional fallback after Phase 13)
- `play_launch_parser` (Rust submodule, primary parser after Phase 13)
- System capabilities for privileged monitoring (`CAP_SYS_PTRACE`, `CAP_SYS_NICE`)

### Rust Crates
- `clap` - CLI parsing
- `tokio` - Async runtime
- `eyre` - Error handling
- `sysinfo` - System monitoring
- `nvml-wrapper` - GPU monitoring
- `axum` - Web framework
- `rust-embed` - Asset embedding

---

## Quick Links

- [Main README](../../README.md)
- [CLAUDE.md - Development Guide](../../CLAUDE.md)
- [CLI Interface Documentation](../cli-interface.md)
- [Resource Monitoring Design](../resource-monitoring-design.md)
- [I/O Helper Design](../io-helper-design.md)

---

## Future Phases

- **Phase 19**: Clone-Isolated Component Manager - See [phase-19-isolated_container.md](./phase-19-isolated_container.md)
  - Phase 19.0: Consolidate ST/MT into single `component_container` binary with CLI flags
  - Phase 19.1-19.2: Per-node process isolation via `clone(CLONE_VM)` with pidfd death monitoring
  - Phase 19.3: Integration tests (crash isolation, IPC, PID visibility)
  - Phase 19.4: play_launch Rust integration (config-driven container mode selection)
  - Phase 19.5: Per-node cgroups CPU control via `CLONE_INTO_CGROUP` (optional)
  - Phase 19.6: Intel MPK memory domain protection (optional, experimental)
  - Design docs: `docs/container-isolation-design.md`, `docs/clone-vm-container-design.md`

## Future Considerations

See [future-considerations.md](./future-considerations.md) for potential enhancements and integration opportunities.
