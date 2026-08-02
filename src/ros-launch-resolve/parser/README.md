# play_launch_parser

Fast ROS 2 launch file parser in Rust. Generates `record.json` for [play_launch](https://github.com/tier4/play_launch).

## Features

- XML, Python, and YAML launch files
- 5-10x faster than dump_launch
- Compatible output format
- 260 tests, 100% Autoware compatible

## Install

```bash
git clone https://github.com/tier4/play_launch_parser.git
cd play_launch_parser
just install-deps
just build
source install/setup.bash
```

Requires ROS 2 (Humble/Jazzy), Rust 1.70+, and [just](https://github.com/casey/just).

## Usage

```bash
# Basic
play_launch_parser launch <package> <file.xml>

# With arguments
play_launch_parser launch <package> <file.xml> -- use_sim:=true

# Autoware example
play_launch_parser launch autoware_launch planning_simulator.launch.xml
```

Output: `record.json` in current directory.

## Supported Elements

**Actions**: `<node>`, `<include>`, `<group>`, `<let>`, `<arg>`, `<set_parameter>`
**Containers**: `<node_container>`, `<composable_node>`, `<load_composable_node>`
**Substitutions**: `$(var)`, `$(env)`, `$(find-pkg-share)`, `$(eval)`, nested lists
**Python**: OpaqueFunction, DeclareLaunchArgument, containers, all standard substitutions

## Performance

| Workload | dump_launch | play_launch_parser |
|----------|-------------|-------------------|
| Simple | ~100ms | <10ms |
| Autoware | ~40s | <5s |

## Development

```bash
just test          # All tests
just test-rust     # Rust tests only
just quality       # Linters + tests
just format        # Format code
```

## Documentation

- [Feature List](docs/feature_list.md) - Complete feature documentation
- [CLAUDE.md](CLAUDE.md) - Development guidelines

## License

[License TBD - should match play_launch]
