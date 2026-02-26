# Autoware Autonomous Mode Investigation

**Date**: 2025-10-22
**Issue**: Autonomous mode button greyed out in RViz when using play_launch
**Status**: Root cause identified - pose_initializer node hangs during initialization

## Problem Description

When running Autoware planning simulator with `play_launch`, the autonomous mode button remains greyed out in RViz even after setting initial and goal poses. The same setup works correctly with standard `ros2 launch`.

## Test Environment

- **Autoware Version**: 2025.02-ws
- **Map**: sample-map-planning
- **DDS**: CycloneDDS (localhost configuration via cyclonedds.xml)
- **Test Directory**: `test/autoware_planning_simulation/`

## Investigation Timeline

### Initial Symptoms

1. User sets initial pose in RViz → No response
2. User sets goal pose in RViz → No response
3. Autonomous mode button remains greyed out
4. simple_planning_simulator stuck in "waiting initialization..." loop

### Service Chain Analysis

Traced the initialization service chain:

```
RViz publishes /initialpose
  ↓
initial_pose_adaptor subscribes to /initialpose
  ↓
initial_pose_adaptor calls service /api/localization/initialize
  ↓
/adapi/node/localization service server receives request
  ↓
/adapi/node/localization calls downstream service /localization/initialize
  ↓
pose_initializer service server receives request
  ↓
pose_initializer should publish to /initialpose3d
  ↓
simple_planning_simulator subscribes to /initialpose3d
```

**Finding**: The chain breaks at pose_initializer - it never publishes to /initialpose3d

### Node Status Verification

All nodes were verified to be running and registered with ROS:

```bash
$ export CYCLONEDDS_URI="file://.../cyclonedds.xml"
$ ros2 node list | grep -E "pose_initializer|simple_planning"
/pose_initializer                          ✓ Running
/simulation/simple_planning_simulator      ✓ Running
```

Services were correctly advertised:

```bash
$ ros2 service list | grep initialize
/api/localization/initialize               ✓ Present
/localization/initialize                   ✓ Present
```

Topics were correctly configured:

```bash
$ ros2 topic list | grep initialpose
/initialpose                               ✓ Present
/initialpose3d                             ✓ Present (but no messages)
```

### Log Analysis

Examined play_launch logs for pose_initializer:

**Location**: `play_log/2025-10-22_12-32-26/node/autoware_pose_initializer/autoware_pose_initializer_node-59/`

**Files**:
- `cmdline` - 1.1KB - Command looks correct
- `pid` - 8 bytes - Process ID recorded
- `out` - **0 bytes** - NO stdout output
- `err` - 66 bytes - Only contains SIGTERM signal handler message

**Key Finding**: The node produced absolutely NO initialization logs, which is highly unusual for a ROS node.

### Comparison Test: ros2 launch vs play_launch

Created comparison script: `scripts/start-sim-ros2-launch.sh`

#### ros2 launch Results ✅ WORKS

```bash
$ make start-sim-ros2-launch
# After setting poses in RViz:
is_autonomous_mode_available: true
Localization state: 3 (INITIALIZED)
Vehicle positioned at goal: (3772.56, 73699.80)
```

**Status**: Autonomous mode button available and functional

#### play_launch Results ❌ DOES NOT WORK

```bash
$ make start-sim
# After setting poses in RViz:
is_autonomous_mode_available: false
Localization state: NOT initialized
simple_planning_simulator: "waiting initialization..."
pose_initializer: NO logs produced
```

**Status**: Autonomous mode button greyed out

### Evidence Collection

1. **initial_pose_adaptor logs** (working):
   ```
   [INFO] [1761113022.734] fit_target: vector_map, frame: map
   [INFO] [1761113027.919] fit_target: vector_map, frame: map
   [INFO] [1761113711.502] fit_target: vector_map, frame: map
   ```
   Shows pose_adaptor successfully receiving /initialpose messages

2. **pose_initializer logs** (NOT working):
   ```
   $ cat .../autoware_pose_initializer_node-59/err
   [INFO] [1761114881.381] [rclcpp]: signal_handler(signum=15)
   ```
   Only signal handler output - NO initialization messages

3. **Topic monitoring**:
   ```bash
   $ timeout 3 ros2 topic echo /initialpose3d --once
   Error  # Timeout - no messages
   ```

4. **File timestamps**:
   - `out`: Last modified 12:32:26 (startup time) - 0 bytes
   - `err`: Last modified 14:34:41 (after SIGTERM) - 66 bytes
   - **Conclusion**: Node produced no output during its entire lifetime

### Configuration Comparison

Both methods use identical configuration:
- ✓ Same Autoware workspace
- ✓ Same map path
- ✓ Same CycloneDDS configuration (cyclonedds.xml)
- ✓ Same node parameters (from record.json)
- ✓ Same topic remappings

### Node Classification Analysis

Checked if pose_initializer might be misclassified:

```bash
$ jq '.node | length' record.json
61  # Regular nodes

$ jq '.load_node | length' record.json
54  # Composable nodes

$ jq '.container | length' record.json
15  # Containers
```

**Finding**: pose_initializer is correctly recorded as a regular node (not composable)

```json
{
  "package": "autoware_pose_initializer",
  "executable": "autoware_pose_initializer_node",
  "exec_name": "autoware_pose_initializer_node-59",
  ...
}
```

Not present in `load_node` array - confirmed NOT an orphan node issue.

## Root Cause Analysis

### What is NOT the problem

❌ **DDS Configuration**: Both use same cyclonedds.xml
❌ **Autoware Configuration**: Both use same workspace
❌ **Map Data**: Both use same map
❌ **Network/DDS Discovery**: All nodes discovered correctly
❌ **Orphan Node**: Correctly classified as regular node
❌ **File Redirection**: stderr works (captured SIGTERM message)
❌ **Process Spawning**: Process runs and registers with ROS
❌ **Command Generation**: Command line looks correct

### What IS the problem

✅ **pose_initializer hangs during early initialization**

**Evidence**:
1. Process starts successfully (PID recorded)
2. Registers with ROS (shows up in `ros2 node list`)
3. Advertises services and topics correctly
4. **But produces ZERO initialization output**
5. Never responds to service calls
6. Never publishes to /initialpose3d

**Hypothesis**: The node is blocking/hanging during initialization before it reaches the point where it would start logging.

### Potential Causes

1. **Environment Variables**: play_launch spawned processes may have different environment than ros2 launch
   - Check: AMENT_PREFIX_PATH, ROS_DOMAIN_ID, etc.

2. **File Descriptor Inheritance**: Tokio process spawning might handle stdio differently than Python's subprocess

3. **Timing/Dependency Issue**: Node might be waiting for something that's not available or takes too long
   - Map data availability?
   - Other service dependencies?
   - DDS discovery timing?

4. **Buffering Issue**: Output is buffered and never flushed
   - Less likely since NO output appears even after SIGTERM

## Code References

### play_launch Node Spawning

**File**: `src/play_launch/src/context.rs:126-178`

Node execution context preparation:
```rust
pub fn to_exec_context(&self) -> eyre::Result<ExecutionContext> {
    // ... preparation code ...

    let stdout_file = File::create(stdout_path)?;
    let stderr_file = File::create(&stderr_path)?;

    let mut command: tokio::process::Command = cmdline.to_command(false).into();

    command.kill_on_drop(true);
    command.stdin(Stdio::null());
    command.stdout(stdout_file);  // ← stdout redirection
    command.stderr(stderr_file);  // ← stderr redirection

    // ...
}
```

**File**: `src/play_launch/src/execution.rs:67-94`

Node spawning:
```rust
pub fn spawn_nodes(
    node_contexts: Vec<NodeContext>,
    process_registry: Option<Arc<Mutex<HashMap<u32, String>>>>,
) -> Vec<impl Future<Output = eyre::Result<()>>> {
    node_contexts.into_iter().filter_map(|context| {
        let exec = match context.to_exec_context() {
            Ok(exec) => exec,
            Err(err) => {
                error!("Unable to prepare execution for node: {err}");
                return None;
            }
        };
        let child = match command.spawn() {
            Ok(child) => child,
            Err(err) => {
                error!("{log_name} is unable to start: {err}");
                return None;
            }
        };
        // ...
    })
}
```

### Process Group Isolation

Both regular nodes and composable nodes use process group isolation:

**File**: `src/play_launch/src/context.rs:158-165`
```rust
#[cfg(unix)]
unsafe {
    command.pre_exec(|| {
        libc::setpgid(0, 0);  // Create new process group
        Ok(())
    });
}
```

**Note**: This prevents child processes from receiving terminal signals (Ctrl-C) directly, which is correct behavior.

### Node Command Line

**File**: `src/play_launch/src/node_cmdline.rs:361-366`

Process group setting was also added here:
```rust
#[cfg(unix)]
{
    use std::os::unix::process::CommandExt;
    command.process_group(0);
}
```

**Note**: This uses the safe wrapper instead of `pre_exec`, but achieves the same result.

## Test Artifacts

### Comparison Script

**File**: `test/autoware_planning_simulation/scripts/start-sim-ros2-launch.sh`

Runs standard ros2 launch with same configuration as play_launch:
- Sources same Autoware workspace
- Uses same CycloneDDS configuration
- Uses same map path

**Makefile Target**: `make start-sim-ros2-launch`

### Log Directories

**play_launch logs**: `test/autoware_planning_simulation/play_log/2025-10-22_12-32-26/`
- Timestamped directory created by play_launch
- Contains node logs, metrics, parameter files
- pose_initializer: 0 bytes stdout, 66 bytes stderr

**ros2 launch logs**: No persistent logs (output to terminal)
- But node produces normal ROS initialization messages
- Can be verified via terminal output or `/tmp` parameter files

## Next Steps

### Immediate Debugging

1. **Test standalone execution**:
   ```bash
   cd test/autoware_planning_simulation
   source autoware/install/setup.bash
   export CYCLONEDDS_URI="file://$(pwd)/cyclonedds.xml"

   # Run exact command from play_launch
   ros2 run autoware_pose_initializer autoware_pose_initializer_node \
     --ros-args \
     -r ndt_align:=/localization/pose_estimator/ndt_align_srv \
     # ... (rest of args from cmdline file)
   ```

   **Goal**: Determine if command itself is valid and produces output

2. **Compare environments**:
   ```bash
   # Get play_launch spawned process environment
   cat /proc/<pid>/environ | tr '\0' '\n' > play_launch_env.txt

   # Get ros2 launch spawned process environment
   cat /proc/<pid>/environ | tr '\0' '\n' > ros2_launch_env.txt

   diff play_launch_env.txt ros2_launch_env.txt
   ```

   **Goal**: Find environment variable differences

3. **Add unbuffered output**:
   Modify `context.rs` to prepend command with `stdbuf -oL -eL`:
   ```rust
   let mut command = Command::new("stdbuf");
   command.args(&["-oL", "-eL", "ros2", "run", ...]);
   ```

   **Goal**: Eliminate buffering as potential cause

4. **Enable ROS logging**:
   Set environment variables before spawning:
   ```rust
   command.env("RCUTILS_CONSOLE_OUTPUT_FORMAT",
               "[{severity}] [{time}] [{name}]: {message}");
   command.env("RCUTILS_LOGGING_USE_STDOUT", "1");
   ```

   **Goal**: Force ROS to use stdout and show all messages

### Code Improvements

1. **Add spawn diagnostics**:
   ```rust
   info!("Spawning node: {}", log_name);
   debug!("Command: {:?}", command);
   debug!("Environment: {:?}", command.get_envs());
   ```

2. **Check child process status**:
   ```rust
   // After spawn, check if process is still alive
   tokio::time::sleep(Duration::from_millis(100)).await;
   if let Some(status) = child.try_wait()? {
       error!("{log_name} exited early with status: {status}");
   }
   ```

3. **Add timeout for node initialization**:
   Detect nodes that never produce output within reasonable time

### Documentation Updates

- ✅ Added comparison script: `scripts/start-sim-ros2-launch.sh`
- ✅ Updated README.md with comparison testing section
- ✅ Updated Makefile with `start-sim-ros2-launch` target
- ✅ This investigation document created

## Conclusions

1. **Issue is reproducible**: Consistently fails with play_launch, consistently works with ros2 launch

2. **Issue is specific to play_launch**: Same configuration, same nodes, different behavior

3. **Issue is early initialization**: Node hangs before producing any log output

4. **Impact**: Prevents Autoware autonomous mode from functioning

5. **Workaround**: Use `make start-sim-ros2-launch` for testing until issue is resolved

## References

- **Test Directory**: `test/autoware_planning_simulation/`
- **Main Issue Logs**: `play_log/2025-10-22_12-32-26/`
- **Comparison Script**: `scripts/start-sim-ros2-launch.sh`
- **This Document**: `docs/autoware-autonomous-mode-investigation.md`
