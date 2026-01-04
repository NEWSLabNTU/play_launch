# Sequential Loading Test

Tests container-managed loading with multiple composable nodes in a single container.

## Test Scenario

- **1 container** (`sequential_container`)
- **5 composable nodes** loaded into the same container:
  - talker1, talker2, listener1, listener2, talker3

## Expected Behavior

1. **Sequential loading (FIFO)**: Nodes load one-by-one in the order they appear in the launch file
2. **Increasing queue wait**: Later nodes wait longer in the container's queue
3. **No parallel loading**: Container serializes all LoadNode requests

## Verification

### Load Timing Metrics

Check `play_log/latest/load_node/*/load_timing.csv`:

| Node | queue_wait_ms | service_call_ms | total_duration_ms |
|------|---------------|-----------------|-------------------|
| talker1 | ~0 | ~300 | ~300 |
| talker2 | ~300 | ~300 | ~600 |
| listener1 | ~600 | ~300 | ~900 |
| listener2 | ~900 | ~300 | ~1200 |
| talker3 | ~1200 | ~300 | ~1500 |

**Pattern**: Queue wait increases by ~service_call_ms for each subsequent node.

### Debug Logs

```bash
just run-debug 2>&1 | grep "Starting load"
```

Look for sequential timestamps (not overlapping).

## Usage

```bash
# Run test
just run

# Check timing metrics
just check-timing

# Clean up
just clean
```

## Success Criteria

- ✅ All 5 nodes load successfully
- ✅ Queue wait times increase linearly
- ✅ Service call times are similar (~100-500ms each)
- ✅ No load failures or timeouts
