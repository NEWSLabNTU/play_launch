# Test Fixtures

This directory contains test fixtures for the play_launch_parser.

## Directory Structure

```
tests/fixtures/
├── launch/              # Main test launch files
│   ├── python/         # Python launch files
│   └── *.launch.xml    # XML launch files
└── includes/           # Launch files to be included by other files
```

## Fixture Categories

### Basic Features
- `test_args.launch.xml` - Argument declaration and usage
- `test_params.launch.xml` - Parameter handling
- `test_remaps.launch.xml` - Topic remapping
- `test_conditions.launch.xml` - Conditional execution

### Namespacing
- `test_namespace.launch.xml` - Basic namespace handling
- `test_namespace_group.launch.xml` - Group-based namespace scoping
- `test_push_pop_namespace.launch.xml` - Explicit namespace push/pop

### Include Mechanism
- `test_include.launch.xml` - Basic file inclusion
- `test_include_with_args.launch.xml` - Passing arguments to includes

### Containers & Composable Nodes
- `test_container.launch.xml` - Container nodes
- `test_composable_node.launch.xml` - Loading composable nodes
- `composable_node_simple.launch.xml` - Simple composable node template

### Python Launch Files
- `python/test_simple.launch.py` - Basic Python launch file
- `python/test_substitutions.launch.py` - Substitution usage
- `python/test_parameters.launch.py` - Parameter files
- `python/test_conditions.launch.py` - Conditional logic
- `python/test_container.launch.py` - Container creation

## Autoware-Inspired Edge Cases

These fixtures test edge cases discovered from Autoware 1.5.0 integration:

### Namespace Synchronization
- **`test_namespace_sync_xml_python.launch.xml`**
  - Tests XML `<push-ros-namespace>` visibility in Python `OpaqueFunction`
  - Pattern: XML pushes namespace → Python file uses it via context
  - Replicates: Autoware's `system.launch.xml` + `component_state_monitor.launch.py`

- **`python/namespace_aware.launch.py`**
  - Python file that reads `ros_namespace` from context
  - Creates nodes with namespace from XML parent

### List Concatenation
- **`python/list_namespace_concatenation.launch.py`**
  - Tests list concatenation in namespace field
  - Pattern: `namespace=["/", LaunchConfiguration("name")]`
  - Common Autoware pattern for dynamic namespace construction

### OpaqueFunction with XML Includes
- **`test_opaque_xml_namespace.launch.xml`**
  - Complete integration test for namespace preservation
  - Pattern: XML namespace → Python OpaqueFunction → XML includes
  - Ensures includes from OpaqueFunction get parent namespace

- **`python/opaque_with_xml_includes.launch.py`**
  - OpaqueFunction that returns `IncludeLaunchDescription` actions
  - Uses `launch_ros.utilities` functions
  - Creates container and loads nodes via XML includes

### Utility Functions
- **`python/test_utilities.launch.py`**
  - Tests `launch_ros.utilities.make_namespace_absolute()`
  - Tests `launch_ros.utilities.prefix_namespace()`
  - These are required by Autoware launch files

### Combined Integration
- **`test_autoware_patterns.launch.xml`**
  - Combines multiple edge cases in one test
  - Tests interaction between features
  - Comprehensive validation of Autoware compatibility

## Usage in Tests

Test fixtures are used in:
- `src/play_launch_parser/tests/python_tests.rs` - Python-related tests
- `src/play_launch_parser/tests/xml_tests.rs` - XML-related tests
- `src/play_launch_parser/tests/integration_tests.rs` - Integration tests

Example:
```rust
fn get_fixture_path(filename: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/launch")
        .join(filename)
}

#[test]
fn test_my_feature() {
    let fixture = get_fixture_path("test_feature.launch.xml");
    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(result.is_ok());
}
```

## Adding New Fixtures

When adding new fixtures:

1. **Choose the right location:**
   - Main test files → `tests/fixtures/launch/`
   - Python files → `tests/fixtures/launch/python/`
   - Files to be included → `tests/fixtures/includes/`

2. **Use descriptive names:**
   - Start with `test_` for main test files
   - Describe the feature being tested
   - Example: `test_opaque_xml_namespace.launch.xml`

3. **Add documentation:**
   - Comment at the top explaining what's being tested
   - Reference any ROS 2 patterns or edge cases
   - Include expected behavior

4. **Create corresponding test:**
   - Add test case in appropriate `*_tests.rs` file
   - Verify all expected behaviors
   - Check error cases if applicable

5. **Update this README:**
   - Add entry in appropriate category
   - Describe the pattern being tested
   - Note any special requirements

## Testing Edge Cases

When creating fixtures for edge cases:

1. **Isolate the pattern:** Create minimal reproducible example
2. **Document the source:** Note which real-world system uses this pattern
3. **Test boundaries:** Include both valid and invalid variations
4. **Combine features:** Test interactions between multiple features

## ROS 2 Compatibility

These fixtures aim to match ROS 2 launch system behavior:
- Python API compatibility (launch, launch_ros)
- XML tag support (standard and hyphenated variants)
- Substitution syntax and resolution
- Namespace scoping rules
- Parameter handling
- Condition evaluation

Verified against:
- ROS 2 Humble
- ROS 2 Jazzy
- Autoware 1.5.0 (binary distribution)
- Autoware 2025.02 (source build)
