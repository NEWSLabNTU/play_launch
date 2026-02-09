# Concurrent Loading Test

Tests container-managed loading with multiple containers loading in parallel.

## Test Scenario

- **4 containers** (`container1`, `container2`, `container3`, `container4`)
- **4 composable nodes** (one per container):
  - talker_c1 → container1
  - talker_c2 → container2
  - listener_c3 → container3
  - talker_c4 → container4

## Expected Behavior

1. **Concurrent loading**: All nodes load in parallel (different containers)
2. **Minimal queue wait**: Each container has only one node (~0ms queue wait)
3. **Overlapping service calls**: Debug logs show concurrent LoadNode execution

## Verification

### Load Timing Metrics

Check `play_log/latest/load_node/*/load_timing.csv`:

| Node | queue_wait_ms | service_call_ms | total_duration_ms |
|------|---------------|-----------------|-------------------|
| talker_c1 | ~0 | ~300 | ~300 |
| talker_c2 | ~0 | ~300 | ~300 |
| listener_c3 | ~0 | ~300 | ~300 |
| talker_c4 | ~0 | ~300 | ~300 |

**Pattern**: All nodes have similar total_duration_ms (started at the same time).

### Debug Logs

```bash
just run-debug 2>&1 | grep "Starting load"
```

Look for overlapping timestamps (concurrent execution).

## Performance Expectations

With centralized ComponentLoader (old):
- Sequential loading: ~1200ms total (4 × 300ms)

With container-managed loading (new):
- Concurrent loading: ~300ms total (all in parallel)

**Expected speedup**: ~4x

## Usage

```bash
# Run test
just run

# Run with debug logs
just run-debug

# Check timing metrics
just check-timing

# Clean up
just clean
```

## Success Criteria

- ✅ All 4 nodes load successfully
- ✅ Queue wait times near zero (~0-10ms)
- ✅ Total duration similar for all nodes
- ✅ Debug logs show overlapping LoadNode calls
