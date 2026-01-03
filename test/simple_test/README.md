# Simple Test Suite

This directory contains simple test cases for play_launch functionality.

## Directory Structure

```
simple_test/
├── launch/
│   ├── pure_nodes.launch.xml      # Simple talker/listener nodes
│   ├── composition.launch.xml     # Composable nodes in container
│   ├── set_parameter.launch.xml   # Set_parameter functionality test
│   └── all.launch.xml             # All tests combined
├── justfile                       # Test automation recipes
└── README.md
```

## Test Cases

### Pure Nodes Test
Tests basic node execution with `demo_nodes_cpp` talker and listener.

**Run:**
```bash
just run-pure-nodes
```

### Composition Test
Tests composable nodes loaded into a component container using the composition package.

**Run:**
```bash
just run-composition
```

### Set Parameter Test
Tests global parameter setting with `<set_parameter>` and parameter inheritance.

**Run:**
```bash
just run-set-parameter
```

### All Tests Combined
Runs all three test cases in a single launch file.

**Run:**
```bash
just run-all
```

## Automated Testing

Run all tests with timeout (for CI):

```bash
just test-all
```

Individual tests:
```bash
just test-pure-nodes
just test-composition
just test-set-parameter
```

## Analysis

Generate resource usage plots from the latest run:

```bash
just plot
```

Clean generated files:

```bash
just clean
```

## Requirements

- ROS 2 Humble (or adjust `ros_distro` variable in justfile)
- `demo_nodes_cpp` package
- `composition` package
- `rclcpp_components` package
- GNU `just` command runner
