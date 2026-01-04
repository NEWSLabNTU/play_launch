# Mixed Loading Test

Tests container-managed loading with both sequential (within container) and concurrent (across containers) patterns.

## Test Scenario

- **2 containers** (`container_a`, `container_b`)
- **5 composable nodes**:
  - **Container A**: talker1, talker2, listener1 (sequential)
  - **Container B**: talker3, listener2 (sequential)
  - **Across containers**: Concurrent execution

## Expected Behavior

1. **Sequential within container**: Nodes in the same container load one-by-one (FIFO)
2. **Concurrent across containers**: Both containers process loads in parallel
3. **Mixed timing pattern**: Combination of queue wait (sequential) and concurrency

## Verification

### Load Timing Metrics

Check `play_log/latest/load_node/*/load_timing.csv`:

**Container A (3 nodes, sequential):**
| Node | queue_wait_ms | service_call_ms | total_duration_ms |
|------|---------------|-----------------|-------------------|
| talker1 | ~0 | ~300 | ~300 |
| talker2 | ~300 | ~300 | ~600 |
| listener1 | ~600 | ~300 | ~900 |

**Container B (2 nodes, sequential):**
| Node | queue_wait_ms | service_call_ms | total_duration_ms |
|------|---------------|-----------------|-------------------|
| talker3 | ~0 | ~300 | ~300 |
| listener2 | ~300 | ~300 | ~600 |

**Pattern**:
- Within each container: Increasing queue_wait_ms
- Across containers: Similar start times (talker1 and talker3 both start ~0ms)

### Debug Logs

```bash
just run-debug 2>&1 | grep -E "Starting load|container_[ab]"
```

Look for:
- Overlapping timestamps for container_a and container_b
- Sequential execution within each container

## Performance Expectations

**Total load time**:
- Container A: ~900ms (3 nodes × 300ms)
- Container B: ~600ms (2 nodes × 300ms)
- **Overall**: ~900ms (limited by slowest container)

With centralized loader:
- Sequential: 5 × 300ms = ~1500ms

**Expected speedup**: ~1.7x

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

- ✅ All 5 nodes load successfully
- ✅ Container A shows increasing queue_wait (talker1 < talker2 < listener1)
- ✅ Container B shows increasing queue_wait (talker3 < listener2)
- ✅ talker1 and talker3 have similar start times (concurrent containers)
- ✅ Overall load time < 1000ms (faster than sequential)
