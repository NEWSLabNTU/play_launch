# I/O Helper Daemon Design

## Overview

A self-contained Rust daemon (`play_launch_io_helper`) that reads `/proc/[pid]/io` files with `CAP_SYS_PTRACE` capability, communicating with `play_launch` via Unix domain sockets with type-safe, declarative messages.

## Problem Statement

- `/proc/[pid]/io` requires ptrace permissions when child processes have capabilities or `dumpable=0`
- `play_launch` cannot have `CAP_SYS_PTRACE` set due to ROS dependency LD_LIBRARY_PATH incompatibility
- Need separate privileged helper that has no ROS dependencies

## Architecture

### 1. Workspace Structure

```
play_launch/
├── Cargo.toml                          # Workspace root
├── src/
│   ├── play_launch/                    # Main binary (existing)
│   │   ├── Cargo.toml
│   │   └── src/
│   ├── play_launch_io_helper/          # Helper daemon (NEW)
│   │   ├── Cargo.toml                  # Statically linked, minimal deps
│   │   └── src/
│   │       └── main.rs
│   └── play_launch_ipc/                # Shared protocol library (NEW)
│       ├── Cargo.toml
│       └── src/
│           ├── lib.rs
│           ├── protocol.rs             # Message definitions
│           └── error.rs                # Error types
```

### 2. Crate Responsibilities

#### `play_launch_ipc` (shared library)
- **Purpose:** Protocol definitions shared between main binary and helper
- **Dependencies:** `serde`, `serde_json` or `bincode` (minimal)
- **Contents:**
  - Request/Response message types
  - Error enums
  - Serialization/deserialization helpers
  - No I/O code, pure data structures

#### `play_launch_io_helper` (binary)
- **Purpose:** Privileged daemon that reads `/proc/[pid]/io`
- **Dependencies:** `play_launch_ipc`, `tokio` (optional features), minimal stdlib
- **Build target:** `x86_64-unknown-linux-musl` (static linking)
- **Capabilities required:** `CAP_SYS_PTRACE`
- **No ROS dependencies**

#### `play_launch` (binary)
- **Purpose:** Main application (existing)
- **New dependency:** `play_launch_ipc` for helper communication
- **Changes:** Spawn helper, send requests, integrate responses

### 3. IPC Protocol Design

#### Communication Mechanism

**Option A: Unix Domain Socket (Recommended)**
```rust
// Socket path: /tmp/play_launch_io_helper_{parent_pid}.sock
// Permissions: 0600 (owner only)
// Protocol: Length-prefixed bincode messages
```

**Advantages:**
- Type-safe with serde
- Bidirectional
- Low overhead
- Built-in tokio support
- Socket permissions provide security

#### Message Protocol (Declarative, Type-Safe)

```rust
// src/play_launch_ipc/src/protocol.rs

use serde::{Deserialize, Serialize};

/// Request from play_launch to helper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Request {
    /// Read I/O stats for a single PID
    ReadProcIo { pid: u32 },

    /// Read I/O stats for multiple PIDs (batch request)
    ReadProcIoBatch { pids: Vec<u32> },

    /// Health check
    Ping,

    /// Graceful shutdown
    Shutdown,
}

/// Response from helper to play_launch
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Response {
    /// I/O statistics for a PID
    ProcIo(ProcIoResult),

    /// Batch response
    ProcIoBatch(Vec<ProcIoResult>),

    /// Pong response
    Pong,

    /// Shutdown acknowledged
    ShutdownAck,

    /// Error occurred
    Error(HelperError),
}

/// Result for a single PID I/O query
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcIoResult {
    pub pid: u32,
    pub result: Result<ProcIoStats, ProcIoError>,
}

/// I/O statistics (mirrors /proc/[pid]/io fields)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcIoStats {
    pub rchar: u64,          // bytes read (total I/O)
    pub wchar: u64,          // bytes written (total I/O)
    pub syscr: u64,          // read syscalls
    pub syscw: u64,          // write syscalls
    pub read_bytes: u64,     // actual disk reads
    pub write_bytes: u64,    // actual disk writes
    pub cancelled_write_bytes: u64,
}

/// Per-PID error types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ProcIoError {
    /// Process not found (exited or invalid PID)
    ProcessNotFound,

    /// Permission denied (even with CAP_SYS_PTRACE)
    PermissionDenied,

    /// /proc filesystem not available
    ProcNotAvailable,

    /// Parse error in /proc/[pid]/io
    ParseError(String),
}

/// Helper daemon errors
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HelperError {
    /// IPC error
    IpcError(String),

    /// Internal error
    InternalError(String),
}
```

#### Wire Format

**Option 1: Bincode (Recommended for performance)**
```rust
// Compact binary format
// Length-prefixed: [u32 length][bincode payload]
```

**Option 2: JSON (Easier debugging)**
```rust
// Human-readable but larger
// Newline-delimited: [json message]\n
```

**Recommendation:** Bincode for production, JSON debug mode via feature flag

### 4. Communication Flow

```
┌─────────────────┐                    ┌──────────────────────┐
│   play_launch   │                    │ io_helper (daemon)   │
│                 │                    │ [CAP_SYS_PTRACE]     │
└────────┬────────┘                    └──────────┬───────────┘
         │                                        │
         │ 1. Spawn helper on startup             │
         │────────────────────────────────────────>│
         │                                        │
         │ 2. Unix socket connection              │
         │<──────────────────────────────────────>│
         │                                        │
         │ 3. Request: ReadProcIoBatch([12, 34])  │
         │────────────────────────────────────────>│
         │                                        │
         │                           4. Read /proc/12/io
         │                           5. Read /proc/34/io
         │                                        │
         │ 6. Response: ProcIoBatch([...])        │
         │<────────────────────────────────────────│
         │                                        │
         │ 7. On shutdown: Shutdown               │
         │────────────────────────────────────────>│
         │                                        │
         │ 8. ShutdownAck                         │
         │<────────────────────────────────────────│
         │                                        │
         X                                        X
```

### 5. Lifecycle Management

#### Helper Startup
```rust
// In play_launch/src/main.rs or resource_monitor.rs

fn spawn_io_helper() -> Result<IoHelperClient> {
    // 1. Find helper binary (same directory as play_launch)
    let helper_path = get_helper_binary_path()?;

    // 2. Check if helper has CAP_SYS_PTRACE
    check_helper_capabilities(&helper_path)?;

    // 3. Spawn helper process
    let socket_path = format!("/tmp/play_launch_io_helper_{}.sock", std::process::id());
    let child = Command::new(helper_path)
        .arg("--socket")
        .arg(&socket_path)
        .spawn()?;

    // 4. Connect to socket (with retry/timeout)
    let client = IoHelperClient::connect(&socket_path, Duration::from_secs(5))?;

    // 5. Send ping to verify
    client.ping()?;

    Ok(client)
}
```

#### Helper Shutdown
```rust
impl Drop for IoHelperClient {
    fn drop(&mut self) {
        // Send shutdown message
        let _ = self.send(Request::Shutdown);

        // Wait for graceful shutdown (with timeout)
        let _ = self.child.wait_timeout(Duration::from_secs(2));

        // Force kill if still alive
        let _ = self.child.kill();
    }
}
```

### 6. Error Handling Strategy

#### Fallback Behavior
```rust
// In ResourceMonitor::parse_proc_io()

fn parse_proc_io(&mut self, pid: u32) -> Result<(u64, u64)> {
    // Try direct read first (works for normal processes)
    match self.try_direct_read(pid) {
        Ok(stats) => return Ok(stats),
        Err(e) if e.kind() == ErrorKind::PermissionDenied => {
            // Permission denied - try helper if available
        }
        Err(e) => return Err(e), // Other errors (not found, etc.)
    }

    // Try helper if available
    if let Some(ref mut helper) = self.io_helper {
        match helper.read_proc_io(pid) {
            Ok(stats) => return Ok(stats),
            Err(HelperError::ProcessNotFound) => return Err(...),
            Err(e) => {
                warn!("Helper failed: {}, disabling helper", e);
                self.io_helper = None; // Disable on error
            }
        }
    }

    // Fallback: warn once and return zeros
    if !self.proc_io_warned {
        warn!("I/O monitoring unavailable for PID {}", pid);
        self.proc_io_warned = true;
    }
    Ok((0, 0))
}
```

### 7. Security Considerations

#### Socket Permissions
```rust
// Helper daemon ensures socket is only accessible by parent process
fn create_socket(path: &Path) -> Result<UnixListener> {
    let listener = UnixListener::bind(path)?;

    // Set permissions: owner only
    std::fs::set_permissions(path, Permissions::from_mode(0o600))?;

    Ok(listener)
}
```

#### Credential Verification
```rust
// Helper verifies connection is from spawning play_launch process
use std::os::unix::net::UnixStream;

fn verify_peer_credentials(stream: &UnixStream, expected_pid: u32) -> Result<()> {
    let cred = stream.peer_cred()?;

    // Check UID matches
    if cred.uid() != unsafe { libc::getuid() } {
        return Err("UID mismatch");
    }

    // Optional: verify parent PID
    let parent_pid = get_parent_pid()?;
    if cred.pid().map(|p| p as u32) != Some(expected_pid)
       && Some(cred.pid()) != parent_pid {
        return Err("PID verification failed");
    }

    Ok(())
}
```

### 8. Build Configuration

#### Root Cargo.toml
```toml
[workspace]
resolver = "3"
members = [
    "src/play_launch",
    "src/play_launch_io_helper",
    "src/play_launch_ipc",
]

[workspace.dependencies]
# Shared version pins
serde = { version = "1.0", features = ["derive"] }
bincode = "1.3"
```

#### io_helper Cargo.toml
```toml
[package]
name = "play_launch_io_helper"
version = "0.1.0"
edition = "2021"

[dependencies]
play_launch_ipc = { path = "../play_launch_ipc" }
serde = { workspace = true }
bincode = { workspace = true }
# Minimal async runtime (optional)
tokio = { version = "1", features = ["rt", "net", "io-util"], optional = true }

[profile.release]
lto = true           # Link-time optimization
codegen-units = 1    # Better optimization
strip = true         # Strip symbols
opt-level = "z"      # Optimize for size
panic = "abort"      # Smaller binary

# Build as static binary
[build]
target = "x86_64-unknown-linux-musl"
```

#### Build Instructions
```makefile
# In Makefile

.PHONY: build-io-helper
build-io-helper:
	@echo "Building I/O helper daemon (static binary)..."
	cargo build --release --target x86_64-unknown-linux-musl -p play_launch_io_helper
	@echo "Applying CAP_SYS_PTRACE capability..."
	sudo setcap cap_sys_ptrace+ep target/x86_64-unknown-linux-musl/release/play_launch_io_helper

.PHONY: install-io-helper
install-io-helper: build-io-helper
	install -m 755 target/x86_64-unknown-linux-musl/release/play_launch_io_helper \
	    install/play_launch/lib/play_launch/
	sudo setcap cap_sys_ptrace+ep install/play_launch/lib/play_launch/play_launch_io_helper
```

### 9. Performance Considerations

#### Batch Requests
```rust
// Instead of many small requests:
for pid in pids {
    helper.read_proc_io(pid)?; // N round-trips
}

// Use batch requests:
let results = helper.read_proc_io_batch(&pids)?; // 1 round-trip
```

#### Connection Pooling
```rust
// Reuse connection across monitoring loop
// No reconnection overhead per sample
```

#### Optional: Async I/O
```rust
// Helper can process multiple requests concurrently
// Useful if monitoring hundreds of PIDs
async fn handle_request(req: Request) -> Response {
    match req {
        Request::ReadProcIoBatch(pids) => {
            // Read all PIDs concurrently
            let futures = pids.into_iter()
                .map(|pid| tokio::fs::read_to_string(format!("/proc/{}/io", pid)));
            let results = futures::future::join_all(futures).await;
            Response::ProcIoBatch(parse_results(results))
        }
        // ...
    }
}
```

### 10. Testing Strategy

#### Unit Tests
- Protocol serialization/deserialization
- Error handling
- Message validation

#### Integration Tests
```rust
#[test]
fn test_helper_reads_privileged_process() {
    // Spawn test process with dumpable=0
    let child = spawn_privileged_test_process();

    // Start helper
    let helper = spawn_io_helper().unwrap();

    // Read I/O stats
    let stats = helper.read_proc_io(child.pid()).unwrap();
    assert!(stats.rchar > 0);
}
```

#### Capability Tests
```bash
# Verify helper has CAP_SYS_PTRACE
getcap target/x86_64-unknown-linux-musl/release/play_launch_io_helper
# Expected: cap_sys_ptrace+ep

# Verify helper can run without LD_LIBRARY_PATH
env -i ./target/x86_64-unknown-linux-musl/release/play_launch_io_helper --help
# Should work (no ROS deps)

# Verify helper can read privileged processes
./test_capability_check.sh
```

### 11. Future Enhancements

- [ ] Support multiple concurrent clients (shared daemon)
- [ ] Caching layer to reduce /proc reads
- [ ] Metrics aggregation in helper (reduce IPC overhead)
- [ ] Systemd socket activation
- [ ] Configuration file for helper behavior
- [ ] Optional TLS/authentication for remote monitoring

## Implementation Phases

### Phase 1: Core Protocol
1. Create `play_launch_ipc` crate with message types
2. Write serialization tests
3. Document protocol

### Phase 2: Helper Daemon
1. Create `play_launch_io_helper` binary
2. Implement socket server
3. Implement /proc/[pid]/io reading
4. Add error handling
5. Test with CAP_SYS_PTRACE

### Phase 3: Integration
1. Add helper client to `play_launch`
2. Modify `ResourceMonitor::parse_proc_io()` with fallback logic
3. Update spawn/shutdown lifecycle
4. Integration testing

### Phase 4: Build & Documentation
1. Update Makefile with musl build targets
2. Add capability setup instructions
3. Update CLAUDE.md with new architecture
4. Add troubleshooting guide

## Open Questions

1. **Async vs sync helper?**
   - Sync: Simpler, less dependencies
   - Async: Better performance with many PIDs
   - **Decision:** Start sync, add async as opt-in feature

2. **Socket vs pipe?**
   - Socket: Bidirectional, better features
   - Pipe: Simpler, less overhead
   - **Decision:** Unix domain socket (tokio-unix-ipc)

3. **JSON vs Bincode?**
   - JSON: Debuggable, human-readable
   - Bincode: Faster, smaller
   - **Decision:** Bincode default, JSON via feature flag

4. **Shared daemon vs per-instance?**
   - Shared: One helper for multiple play_launch instances
   - Per-instance: Simpler lifecycle
   - **Decision:** Per-instance initially, shared later

5. **Graceful degradation severity?**
   - Error: Fail if helper unavailable
   - Warn: Continue without I/O stats
   - **Decision:** Warn (existing behavior), user can opt-in to require helper
