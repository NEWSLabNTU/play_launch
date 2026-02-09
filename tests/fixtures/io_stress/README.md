# I/O Stress Test

ROS2 package with custom C++ nodes for high-throughput disk and network I/O stress testing.

## Architecture

**Pipeline**: /dev/urandom → talker → ROS2 topic → listener → /dev/null

- **Talker nodes**: Read random data from `/dev/urandom` using background I/O thread, publish blocks to `/io_stress` topic via ROS2
- **Listener nodes**: Subscribe to `/io_stress` topic, write received blocks to `/dev/null` using background I/O thread
- **Threading**: Producer-consumer pattern with thread-safe queues for maximum I/O throughput
- **Configurable**: Block size parameter adjusts I/O intensity

## Test Setup

- **10 nodes total**: 5 talkers + 5 listeners
- **Default block size**: 8192 bytes (8 KB)
- **Message type**: std_msgs::msg::UInt8MultiArray
- **I/O operations**:
  - Disk reads: /dev/urandom (talkers)
  - Network: ROS2 pub/sub over DDS
  - Disk writes: /dev/null (listeners)

## Usage

```bash
# Build the C++ package
make build

# Run stress test with I/O monitoring
make run

# Generate resource usage plots
make plot

# Clean artifacts
make clean
```

## Implementation Details

### Talker Node (src/talker.cpp)

**Pipeline architecture:**
1. Background thread reads blocks from `/dev/urandom`
2. Pushes blocks to thread-safe queue
3. ROS2 timer (1ms) pops from queue and publishes to topic

**Parameters:**
- `block_size`: Size of data blocks in bytes (default: 4096)

**Statistics**: Logs throughput every 5 seconds

### Listener Node (src/listener.cpp)

**Pipeline architecture:**
1. ROS2 subscription callback receives messages
2. Pushes blocks to thread-safe queue
3. Background thread pops from queue and writes to `/dev/null`

**Statistics**: Logs throughput every 5 seconds

## Expected Output

```
play_log/YYYY-MM-DD_HH-MM-SS/
├── node/
│   ├── talker_1/
│   │   ├── metrics.csv       # All I/O fields populated
│   │   └── metadata.json
│   ├── talker_2/
│   ⋮
│   ├── listener_1/
│   ⋮
└── system_stats.csv            # Aggregate I/O stats
```

## Verification

After `make plot`, check:
1. **10 node directories** in `play_log/TIMESTAMP/node/`
2. **metrics.csv** with I/O columns showing high throughput:
   - `io_syscr` / `io_syscw`: System call counts
   - `io_storage_read_bytes` / `io_storage_write_bytes`: Storage I/O
   - `total_read_bytes` / `total_write_bytes`: Total I/O including cache
3. **Interactive HTML plots** showing I/O timelines and distributions
4. **statistics.txt** with I/O rankings

## Key Features

- ✅ Real disk I/O (/dev/urandom, /dev/null)
- ✅ High-frequency ROS2 messaging (1ms publish interval)
- ✅ Pipeline concurrency (non-blocking I/O)
- ✅ Configurable load via block_size parameter
- ✅ Thread-safe producer-consumer queues
- ✅ Graceful shutdown handling

## Dependencies

- ROS2 Humble
- rclcpp
- std_msgs
- C++17 (for threading support)

## Package Structure

```
test/io_stress/
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS2 package manifest
├── Makefile               # Build/run/plot/clean targets
├── README.md              # This file
├── src/
│   ├── talker.cpp         # /dev/urandom → ROS topic
│   └── listener.cpp       # ROS topic → /dev/null
└── launch/
    └── io_stress.launch.xml  # Launch 5 talkers + 5 listeners
```

## Notes

- Uses standard ROS2 C++ nodes (no containers, no CAP_SYS_PTRACE required)
- For comparison with containerized workloads, see `test/autoware_planning_simulation/`
- Integrates with play_launch for I/O monitoring and resource tracking
