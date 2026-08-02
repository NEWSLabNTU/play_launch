# Phase 1: Project Setup & Architecture Design

**Status**: 🚧 In Progress
**Priority**: High (Foundation)
**Estimated Effort**: 1 week
**Dependencies**: None

---

## Overview

Establish the foundational project structure, tooling, and architecture design for the play_launch_parser project. This phase focuses on setting up the development environment, creating initial documentation, and designing the overall system architecture.

---

## Motivation

### Goals
- Create a well-organized project structure following play_launch conventions
- Set up development tooling (justfile, linters, formatters)
- Document the project vision and technical approach
- Design the parser architecture before implementation

### Success Criteria
- [ ] Project structure matches play_launch conventions
- [ ] Documentation clearly explains project goals and approach
- [ ] Architecture design covers all major components
- [ ] Build system (justfile) works correctly
- [ ] Ready to begin parser implementation

---

## Work Items

### Task 1.1: Project Structure Setup ✅

**Objective**: Create directory structure and initial files

**Subtasks**:
- [x] 1.1.1: Create src/ directory for source packages
- [x] 1.1.2: Create docs/ and docs/roadmap/ directories
- [x] 1.1.3: Create .gitignore file
- [x] 1.1.4: Initialize git repository

**Files Created**:
- `/src/` - Source packages directory
- `/docs/` - Documentation directory
- `/docs/roadmap/` - Phase roadmap files
- `/.gitignore` - Git ignore patterns

**Status**: ✅ Complete

---

### Task 1.2: Development Tooling ✅

**Objective**: Set up build system and development tools

**Subtasks**:
- [x] 1.2.1: Create justfile with build/test/check recipes
- [x] 1.2.2: Add install-deps recipe for dependency installation
- [x] 1.2.3: Add clean/clean-all recipes
- [x] 1.2.4: Add download-demos recipe for test packages

**Files Created**:
- `/justfile` - Build and development recipes

**Recipes**:
- `just build` - Build all packages with colcon
- `just test` - Run tests
- `just check` - Run linters and formatters
- `just format` - Format code
- `just clean` - Clean build artifacts
- `just install-deps` - Install dependencies
- `just download-demos` - Download test packages

**Status**: ✅ Complete

---

### Task 1.3: Documentation ✅

**Objective**: Create comprehensive project documentation

**Subtasks**:
- [x] 1.3.1: Create CLAUDE.md with project overview and guidelines
- [x] 1.3.2: Create this roadmap file (phase-1-project_setup.md)
- [x] 1.3.3: Create README.md with user-facing documentation
- [x] 1.3.4: Create docs/ros2_launch_architecture.md analyzing ROS 2 launch system
- [x] 1.3.5: Create docs/dump_launch_analysis.md analyzing dump_launch implementation
- [x] 1.3.6: Create docs/record_json_format.md specifying record.json format

**Files Created**:
- `/CLAUDE.md` - Guide for Claude Code (5,489 bytes) ✅
- `/docs/roadmap/phase-1-project_setup.md` - This file ✅
- `/README.md` - User-facing documentation (5,552 bytes) ✅
- `/docs/ros2_launch_architecture.md` - ROS 2 launch architecture (880 lines) ✅
- `/docs/dump_launch_analysis.md` - dump_launch analysis (776 lines) ✅
- `/docs/record_json_format.md` - record.json specification (731 lines) ✅

**Documentation Summary**:
- Total: 2,387 lines of comprehensive technical documentation
- External repos analyzed: launch, launch_ros, demos, play_launch/dump
- Complete architecture understanding documented
- record.json format fully specified

**Status**: ✅ Complete

---

### Task 1.4: Architecture Design 📋

**Objective**: Design the parser architecture and component interactions

**Subtasks**:
- [ ] 1.4.1: Analyze ROS 2 XML launch file format and semantics
- [ ] 1.4.2: Analyze ROS 2 YAML launch file format and semantics
- [ ] 1.4.3: Study dump_launch implementation and behavior
- [ ] 1.4.4: Design XML parser component
- [ ] 1.4.5: Design YAML parser component
- [ ] 1.4.6: Design launch tree builder component
- [ ] 1.4.7: Design record.json generator component
- [ ] 1.4.8: Document data structures and interfaces

**Design Documents**:
- `/docs/ARCHITECTURE.md` - Overall architecture
- `/docs/xml_format.md` - XML launch file format analysis
- `/docs/yaml_format.md` - YAML launch file format analysis
- `/docs/record_format.md` - record.json format specification

**Status**: 📋 Not Started

---

### Task 1.5: Initial Package Setup 📋

**Objective**: Create initial Rust package structure

**Subtasks**:
- [ ] 1.5.1: Create play_launch_parser ROS 2 package
- [ ] 1.5.2: Set up Cargo.toml with dependencies
- [ ] 1.5.3: Create package.xml for ROS 2 integration
- [ ] 1.5.4: Set up basic module structure
- [ ] 1.5.5: Create initial CLI skeleton

**Package Structure**:
```
src/play_launch_parser/
├── Cargo.toml
├── package.xml
└── src/
    ├── main.rs
    ├── xml_parser/
    │   └── mod.rs
    ├── yaml_parser/
    │   └── mod.rs
    ├── tree_builder/
    │   └── mod.rs
    └── record_generator/
        └── mod.rs
```

**Status**: 📋 Not Started

---

### Task 1.6: Testing Infrastructure 📋

**Objective**: Set up testing framework and initial test cases

**Subtasks**:
- [ ] 1.6.1: Download ROS 2 demo packages for testing
- [ ] 1.6.2: Set up integration test framework
- [ ] 1.6.3: Create test fixture generator
- [ ] 1.6.4: Document testing workflow

**Test Strategy**:
- Unit tests: Parser components, tree builder logic
- Integration tests: Real ROS 2 launch files
- Comparison tests: Parser output vs dump_launch output
- Performance tests: Parse time benchmarks

**Status**: 📋 Not Started

---

## Technical Decisions

### Build System
- **Choice**: colcon + cargo (via colcon-cargo-ros2)
- **Rationale**: Native ROS 2 integration, familiar to ROS developers
- **Alternative Considered**: Pure cargo workspace (rejected: harder ROS 2 integration)

### Language
- **Choice**: Rust
- **Rationale**: Performance, safety, matches play_launch runtime
- **Alternative Considered**: C++ (rejected: safety concerns, less modern tooling)

### Initial Scope
- **Choice**: XML + YAML only (Python deferred)
- **Rationale**: Python launch files require Python interpreter, complex to parse statically
- **Future Work**: Python launch support in later phase

---

## Risks and Mitigations

| Risk                                  | Impact | Mitigation                                                    |
|---------------------------------------|--------|---------------------------------------------------------------|
| ROS 2 launch semantics too complex    | High   | Start with simple XML/YAML cases, iterate                     |
| record.json format undocumented       | Medium | Reverse-engineer from dump_launch, test extensively           |
| Performance goals unmet               | Medium | Profile early, optimize hot paths, parallelize where possible |
| Compatibility issues with play_launch | High   | Extensive integration testing, version pinning                |

---

## Next Steps

After Phase 1 completion:
1. **Phase 2**: XML Parser Implementation
   - Implement XML parsing with `roxmltree` or similar
   - Handle basic node, container, composable node definitions
   - Implement substitution resolution

2. **Phase 3**: YAML Parser Implementation
   - Implement YAML parsing with `serde_yaml`
   - Support YAML-specific features
   - Reuse substitution logic from XML parser

3. **Phase 4**: Launch Tree Builder
   - Build unified launch tree from parsed files
   - Handle includes and parameter passing
   - Resolve all substitutions

4. **Phase 5**: Record Generator & Integration
   - Generate record.json compatible with play_launch
   - Integration testing with play_launch runtime
   - Performance benchmarking

---

## References

- [ROS 2 Launch Documentation](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Launch-Main.html)
- [play_launch Project](https://github.com/tier4/play_launch) (assumed path)
- [dump_launch Python Package](https://github.com/tier4/play_launch) (assumed path)
- [colcon-cargo-ros2](https://github.com/colcon/colcon-cargo)

---

## Notes

- Keep architecture flexible for future Python launch support
- Document all semantic differences from dump_launch
- Prioritize correctness over performance initially
- Maintain compatibility with play_launch runtime versions
