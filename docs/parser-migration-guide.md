# Parser Migration Guide

**Date**: 2026-01-27
**Version**: 0.6.0+

---

## Overview

Starting from version 0.6.0, `play_launch` uses a **Rust-based parser** as the default for launch file parsing, replacing the previous Python-based parser. This change provides significant performance improvements (3-12x faster) while maintaining full compatibility with ROS 2 launch files.

---

## What Changed

### Before (< v0.6.0)

```bash
# Python parser was the only option
play_launch launch demo_nodes_cpp talker_listener.launch.py
```

**Behavior**: Always used Python parser (~40s for large launch files)

### After (≥ v0.6.0)

```bash
# Rust parser is now the default
play_launch launch demo_nodes_cpp talker_listener.launch.py

# Or explicitly specify Rust parser
play_launch launch demo_nodes_cpp talker_listener.launch.py --parser rust

# Use Python parser for maximum compatibility
play_launch launch demo_nodes_cpp talker_listener.launch.py --parser python
```

**Behavior**:
- **Default**: Rust parser (fast, fails immediately on error)
- **Option**: Python parser (maximum compatibility)
- **No automatic fallback**: User explicitly chooses parser

---

## Performance Improvements

| Test Case | Rust Parser | Python Parser | Speedup |
|-----------|-------------|---------------|---------|
| Simple nodes (pure_nodes.launch.xml) | 0.133s | 0.390s | **2.93x** |
| Composable nodes (simple_test.launch.xml) | 0.136s | 1.570s | **11.54x** |
| Autoware planning_simulator | ~4s | ~40s | **~10x** |

---

## Migration Steps

### For Most Users

**No action required.** The Rust parser is the default and works with all standard ROS 2 launch files.

```bash
# This just works (uses Rust parser)
play_launch launch <package> <launch_file>
```

### If You Encounter Parsing Errors

If the Rust parser fails, you'll see a clear error message:

```
Error: Rust parser error: XML parsing error: invalid name token at 6:3

Hint: If you encounter parsing issues, try the Python parser:
  play_launch launch demo_nodes_cpp talker_listener.launch.py --parser python
```

**Solution**: Use the Python parser as suggested:

```bash
play_launch launch <package> <launch_file> --parser python
```

### For Scripts and Automation

Update any scripts that previously relied on specific parser behavior:

```bash
# Before (no flag needed, Python was default)
play_launch launch autoware_launch planning_simulator.launch.xml

# After (Rust is default, usually no change needed)
play_launch launch autoware_launch planning_simulator.launch.xml

# If you need Python parser for compatibility
play_launch launch autoware_launch planning_simulator.launch.xml --parser python
```

---

## When to Use Each Parser

### Use Rust Parser (Default)

✅ **Recommended for**:
- Normal day-to-day usage
- Performance-critical workflows
- CI/CD pipelines (faster builds)
- All standard ROS 2 launch files

✅ **Benefits**:
- 3-12x faster parsing
- Lower memory usage
- Immediate error feedback
- Predictable behavior

### Use Python Parser

⚙️ **Use when**:
- Rust parser fails with a parsing error
- Testing compatibility between parsers
- Edge cases or non-standard launch file features
- Maximum compatibility is required

⚙️ **How to use**:
```bash
play_launch launch <package> <file> --parser python
play_launch dump launch <package> <file> --parser python
```

---

## Compatibility

### Supported Launch File Formats

Both parsers support:
- ✅ XML launch files (`.launch.xml`)
- ✅ Python launch files (`.launch.py`)
- ✅ YAML launch files (`.yaml`)
- ✅ Launch arguments (`KEY:=VALUE`)
- ✅ Composable nodes
- ✅ Nested includes
- ✅ Parameter files

### Tested Configurations

The Rust parser has been validated with:
- ✅ **218 unit tests** in parser library
- ✅ **Autoware planning_simulator** (52 composable nodes, 15 containers)
- ✅ **demo_nodes_cpp** (simple and composable nodes)
- ✅ All launch file formats (XML, Python, YAML)

---

## Breaking Changes

### Removed: Auto Mode

Previously, there was an `Auto` parser mode that tried Rust first and fell back to Python on error. This has been removed for clarity and predictability.

**Before** (< v0.6.0):
```bash
# Auto mode (hidden fallback behavior)
play_launch launch <package> <file> --parser auto
```

**After** (≥ v0.6.0):
```bash
# Explicit choice required
play_launch launch <package> <file>                # Rust (default)
play_launch launch <package> <file> --parser python  # Python (explicit)
```

**Why removed?**
- Hidden behavior: Users didn't know which parser was used
- Inconsistent: Same command might use different parsers
- Slower error detection: Rust parser errors were masked
- Debugging difficulty: Hard to reproduce issues

---

## Troubleshooting

### Error: "Rust parser error: ..."

**Problem**: The Rust parser encountered a parsing error.

**Solution**: Try the Python parser:
```bash
play_launch launch <package> <file> --parser python
```

If the Python parser also fails, the launch file likely has a syntax error.

### Performance Degradation After Upgrade

**Problem**: Launch parsing seems slower than before.

**Check**: Ensure you're not accidentally using the Python parser:
```bash
# Verify Rust parser is being used (should see "Using Rust parser")
play_launch launch <package> <file>
```

**Solution**: Remove any `--parser python` flags from scripts.

### Script Automation Issues

**Problem**: Scripts that used `--parser auto` now fail.

**Solution**: Remove the `--parser auto` flag (Rust is now default):
```bash
# Before
play_launch launch <pkg> <file> --parser auto

# After (remove flag)
play_launch launch <pkg> <file>
```

---

## Feedback and Bug Reports

If you encounter issues with the Rust parser:

1. **Try Python parser** to verify the launch file is valid
2. **Report the issue** at https://github.com/tnagyzambo/play_launch_parser/issues
3. **Include**:
   - Launch file (or minimal reproduction)
   - Error message
   - ROS 2 distribution (Humble/Jazzy)
   - Expected behavior

---

## Additional Resources

- **Parser Library**: [play_launch_parser](https://github.com/tnagyzambo/play_launch_parser)
- **Benchmark Scripts**: `scripts/benchmark_parsers.sh`, `scripts/compare_parsers.sh`
- **Test Cases**: `test/simple_test/`, `test/autoware_planning_simulation/`
- **Phase 13 Documentation**: `docs/roadmap/phase-13.md`

---

**Last Updated**: 2026-01-27
**Applies to**: play_launch v0.6.0+
