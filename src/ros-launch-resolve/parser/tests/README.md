# Test Directory

This directory contains testing utilities and auxiliary test scripts for the play_launch_parser project.

**Note**: The actual test fixtures used by Rust unit/integration tests are located at `src/play_launch_parser/tests/fixtures/`, not here. This directory contains auxiliary testing tools.

## Directory Structure

```
tests/
├── autoware_test/      # Autoware-specific testing
│   ├── scripts/        # Test and comparison scripts
│   │   ├── activate_autoware.sh          # User-specific workspace activation (git-ignored)
│   │   ├── activate_autoware.sh.template # Template for activation script
│   │   └── compare_rust_python.py        # Autoware comparison script
│   └── output/         # Test output files
├── comparison_tests/   # Rust vs Python comparison tests
│   ├── compare_rust_python.py  # Main comparison script
│   ├── test_simple.launch.xml  # Simple test case
│   ├── test_group_namespace.launch.xml  # Namespace test
│   └── run_tests.sh    # Run all comparison tests
└── README.md           # This file

src/play_launch_parser/tests/
├── fixtures/           # ACTUAL TEST FIXTURES (used by Rust tests)
│   ├── launch/         # Main test launch files
│   │   ├── python/     # Python launch files
│   │   └── *.xml       # XML launch files
│   └── includes/       # Launch files to be included by other launch files
├── edge_cases.rs       # Edge case tests
├── integration_tests.rs # Integration tests
├── python_tests.rs     # Python launch file tests
└── xml_tests.rs        # XML launch file tests
```

## Test Launch Files

The actual test launch files are located at `src/play_launch_parser/tests/fixtures/launch/`.

For documentation on test fixtures, see the inline comments in the test files or refer to:
- `src/play_launch_parser/tests/xml_tests.rs` - XML test documentation
- `src/play_launch_parser/tests/python_tests.rs` - Python test documentation
- `src/play_launch_parser/tests/edge_cases.rs` - Edge case documentation

### Usage

Test fixtures are accessed via the helper functions in each test file:

```rust
use std::path::PathBuf;

// In xml_tests.rs and python_tests.rs:
fn get_fixture_path(filename: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/launch")
        .join(filename)
}
```

### Adding New Fixtures

When adding new test fixtures:
1. Place main launch files in `src/play_launch_parser/tests/fixtures/launch/`
2. For Python files: `src/play_launch_parser/tests/fixtures/launch/python/`
3. Place included launch files in `src/play_launch_parser/tests/fixtures/includes/`
4. Use descriptive names like `test_<feature>.launch.xml` or `test_<feature>.launch.py`
5. Add corresponding test cases in the appropriate test file
6. Update any include paths to use relative paths (e.g., `../includes/included.launch.xml`)

## Comparison Tests

The `comparison_tests/` directory contains tools for comparing Rust and Python parser outputs.

### Quick Start

```bash
# Run all comparison tests
cd tests/comparison_tests
./run_tests.sh

# Run individual test
python3 compare_rust_python.py test_simple.launch.xml
```

### Test Files

- **test_simple.launch.xml** - Basic namespace scoping test
- **test_group_namespace.launch.xml** - Nested namespace inheritance test
- **compare_rust_python.py** - Comprehensive comparison script
- **run_tests.sh** - Run all comparison tests

See `comparison_tests/README.md` for detailed documentation.

## Autoware Tests

The `autoware_test/` directory contains Autoware-specific test scripts and output.

### Setup

Create an activation script to point to your Autoware workspace:

```bash
cd tests/autoware_test/scripts
cp activate_autoware.sh.template activate_autoware.sh
# Edit activate_autoware.sh to point to your Autoware workspace
```

Example `activate_autoware.sh` content:
```bash
#!/usr/bin/env bash
source /path/to/your/autoware/workspace/install/setup.bash
```

**Note:** The `activate_autoware.sh` file is git-ignored since it contains user-specific paths.

### Scripts

- **scripts/compare_rust_python.py** - Comparison tool that validates Rust parser against Python dump_launch
- **scripts/activate_autoware.sh.template** - Template for workspace activation script
- **scripts/benchmark.sh** - Performance benchmarking

### Usage

```bash
# Run default test (planning_simulator.launch.xml)
just test-autoware

# Or run comparison directly
cd tests/autoware_test/scripts
python3 compare_rust_python.py

# Test specific launch file
python3 compare_rust_python.py /path/to/launch/file.launch.xml
```

See `autoware_test/README.md` for more details.
