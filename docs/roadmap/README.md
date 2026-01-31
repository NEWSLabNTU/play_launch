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

**Overall Progress**: ~95% complete (12 of 14 phases complete, 0 planned, 2 in planning)

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
| **Phase 9**: Web UI Status Refactoring | 🔄 In Progress | - | [phase-9.md](./phase-9.md) |

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

- 🔄 Web UI status system refactoring (Phase 9)
  - Distinct status types for nodes, containers, and composable nodes
  - Improved health metrics with categorized counts
  - Container restart with composable node reloading

### Planned ⏳

- ⏳ Web UI Actor Integration (Phase 11) - Remove bridging layer, implement direct actor control
- ⏳ Complete documentation (Phase 3)
- ⏳ Comprehensive testing (Phase 4)
- ⏳ Optional enhancements (Phase 5)

### Recently Completed 🎉

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

## Future Considerations

See [future-considerations.md](./future-considerations.md) for potential enhancements and integration opportunities.
