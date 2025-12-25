# Event-Driven Architecture Refactoring Plan

**Date:** 2025-12-25
**Status:** Design Phase
**Goal:** Refactor play_launch to use distinct member types and event-driven state management

---

## Table of Contents

1. [Current Architecture & Problems](#1-current-architecture--problems)
2. [Proposed Architecture](#2-proposed-architecture)
3. [Member Type Redesign](#3-member-type-redesign)
4. [Event System Design](#4-event-system-design)
5. [State Machines](#5-state-machines)
6. [Event Processing](#6-event-processing)
   - 6.3 [ProcessMonitor Component](#63-processmonitor-component)
     - Monitoring Task State Machine
     - Usage Flow
     - Benefits
7. [Container-Composable Node Interactions](#7-container-composable-node-interactions)
8. [Web UI Integration](#8-web-ui-integration)
9. [Migration Strategy](#9-migration-strategy)
10. [Benefits](#10-benefits)

---

## 1. Current Architecture & Problems

### 1.1 Current Design

```
NodeRegistry
├── HashMap<String, NodeHandle>
│   ├── NodeHandle (unified type for all node types)
│   │   ├── node_type: NodeType (enum: Node | Container | ComposableNode)
│   │   ├── info: NodeInfo (enum: Regular | Composable)
│   │   └── child: Option<Child>
│   └── State is computed on-demand from filesystem
└── No event system - everything is polled
```

### 1.2 Key Problems

**Problem 1: Mixed Concerns**
- Single `NodeHandle` type tries to represent three fundamentally different entities
- Container-specific logic (managing composable nodes) mixed with regular node logic
- Lots of runtime type checking: `if node_type == Container { ... }`

**Problem 2: Filesystem as Source of Truth**
- State is inferred from files (`status`, `pid`, `service_response.*`) on every query
- Race conditions: file might change between read and action
- No in-memory state representation
- Expensive: every web UI refresh reads dozens of files

**Problem 3: Implicit Relationships**
- Container-composable node relationship is string matching
- No direct references or indexes
- Hard to find "what composable nodes are in this container?"

**Problem 4: No Event System**
- Direct manipulation: `registry.start_node()` does everything inline
- No decoupling: web handler must know about containers, composable nodes, etc.
- No reactive updates: can't notify web UI when state changes
- No audit trail: hard to debug "why did this node enter this state?"

**Problem 5: Container Failure Propagation is Hacky**
- Added `get_status_with_registry()` as workaround
- Checks container status every time composable node status is queried
- Not scalable: what if we need multi-level dependencies?

**Problem 6: Inconsistent State Model**
- Regular nodes: Running | Stopped | Failed | Pending
- Composable nodes: Loaded | Failed | Pending
- What does "Failed" mean for a composable node when container is stopped?
- Confusion between "node failed to load" vs "container failed"

---

## 2. Proposed Architecture

### 2.1 High-Level Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                         Web UI / CLI                             │
└─────────────────┬───────────────────────────────────────────────┘
                  │ HTTP Requests / Commands
                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Web Handlers / Main                         │
│  - Publish events (StartRequested, StopRequested, etc.)         │
└─────────────────┬───────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                         Event Bus                                │
│  - mpsc::unbounded_channel                                       │
│  - Broadcast events to all subscribers                           │
└─────────────────┬───────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Event Processor                             │
│  - Receives events from bus                                      │
│  - Applies business logic                                        │
│  - Updates MemberRegistry state                                  │
│  - Coordinates with ProcessMonitor                               │
│  - Publishes new events as side effects                          │
└─────────────────┬───────────────────────────────────────────────┘
                  │
                  ├──────────────────────────────────────┐
                  ▼                                      ▼
┌──────────────────────────────────────┐  ┌─────────────────────────┐
│        Member Registry               │  │    Process Monitor      │
│  - HashMap<String, Member>           │  │  - Spawns monitoring    │
│  - Pure data storage (PID, state)    │  │    tasks per process    │
│  - Indexes: container → composable   │  │  - Owns Child handles   │
│  - State transitions with validation │  │  - Publishes exit events│
└──────────────────────────────────────┘  └─────────────────────────┘
```

### 2.2 Key Principles

1. **Separation of Concerns**: Each type has its own struct with type-specific fields
2. **Event-Driven**: All actions go through events, enabling reactive patterns
3. **Explicit State**: State is stored in memory, not computed from filesystem
4. **Indexed Relationships**: Direct mapping between containers and composable nodes
5. **Type Safety**: Compiler enforces valid operations per member type
6. **Distinct ProcessMonitor**: Process lifecycle monitoring is a separate component, not embedded in members

---

## 3. Member Type Redesign

### 3.1 Distinct Member Types

Create three separate structs instead of one unified `NodeHandle`:

```rust
// File: src/play_launch/src/member.rs

pub struct RegularNode {
    pub name: String,
    pub state: ProcessState,              // NEW: explicit state (includes PID when running)
    pub record: NodeRecord,
    pub cmdline: NodeCommandLine,
    pub output_dir: PathBuf,
    pub log_paths: NodeLogPaths,

    // Runtime configuration
    pub respawn_enabled: bool,
    pub respawn_delay: f64,

    // NOTE: No Child field! ProcessMonitor owns the Child handle.
    // PID is stored in ProcessState::Running { pid }
}

pub struct Container {
    pub name: String,
    pub state: ProcessState,              // NEW: explicit state (includes PID when running)
    pub record: NodeRecord,
    pub cmdline: NodeCommandLine,
    pub output_dir: PathBuf,
    pub log_paths: NodeLogPaths,

    // Runtime state
    pub composable_nodes: Vec<String>,    // NEW: explicit list of loaded nodes
    pub respawn_enabled: bool,
    pub respawn_delay: f64,

    // NOTE: No Child field! ProcessMonitor owns the Child handle.
    // PID is stored in ProcessState::Running { pid }
}

pub struct ComposableNode {
    pub name: String,
    pub state: ComposableState,           // NEW: distinct state type
    pub container_name: String,           // NEW: explicit reference to container
    pub record: ComposableNodeRecord,
    pub output_dir: PathBuf,
    pub log_paths: NodeLogPaths,
}

// Unified enum for storage
pub enum Member {
    Node(RegularNode),
    Container(Container),
    ComposableNode(ComposableNode),
}
```

### 3.2 State Types

**ProcessState** (for Regular Nodes and Containers):

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum ProcessState {
    Running { pid: u32 },
    Stopped,                              // User-initiated stop
    Failed { exit_code: Option<i32> },   // Crashed while running
    Pending,                              // Not yet started
}
```

**ComposableState** (for Composable Nodes):

```rust
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub enum ComposableState {
    // When container is running
    Loaded,          // Successfully loaded via LoadNode service
    Loading,         // LoadNode service call in progress
    Unloaded,        // Not loaded (initial state or load failed)

    // When container is not available
    Blocked { reason: BlockReason },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum BlockReason {
    ContainerStopped,      // Container was stopped by user
    ContainerFailed,       // Container crashed
    ContainerNotStarted,   // Container hasn't started yet (initial state)
}
```

**Key Design Decisions:**

- **Stopped vs Failed**: `Stopped` = intentional shutdown, `Failed` = crashed
- **Blocked State**: Makes container dependency explicit
- **Loading State**: Tracks in-progress operations
- **No ambiguity**: Clear semantics for each state

---

## 4. Event System Design

### 4.1 Event Types

```rust
// File: src/play_launch/src/events.rs

#[derive(Debug, Clone)]
pub enum MemberEvent {
    // ===== Process Lifecycle (Containers & Regular Nodes) =====
    ProcessStartRequested { name: String },
    ProcessStarted { name: String, pid: u32 },
    ProcessExited { name: String, exit_code: Option<i32> },
    ProcessStopped { name: String },      // Confirmed stopped
    ProcessFailed { name: String, exit_code: Option<i32> },  // Confirmed failed

    // ===== Composable Node Lifecycle =====
    LoadRequested { name: String },
    LoadStarted { name: String },         // LoadNode service call started
    LoadSucceeded { name: String },       // LoadNode returned success
    LoadFailed { name: String, error: String },
    Blocked { name: String, reason: BlockReason },
    Unblocked { name: String },           // Container became available again

    // ===== User Actions =====
    StartRequested { name: String },      // Generic start (dispatch by type)
    StopRequested { name: String },
    RestartRequested { name: String },
    RespawnToggled { name: String, enabled: bool },

    // ===== System Events =====
    ShutdownRequested,
    StateChanged { name: String },        // Notify subscribers (web UI)
}
```

### 4.2 Event Bus

```rust
pub struct EventBus {
    tx: mpsc::UnboundedSender<MemberEvent>,
}

impl EventBus {
    pub fn new() -> (Self, mpsc::UnboundedReceiver<MemberEvent>) {
        let (tx, rx) = mpsc::unbounded_channel();
        (Self { tx }, rx)
    }

    pub fn publish(&self, event: MemberEvent) -> Result<()> {
        self.tx.send(event).map_err(|_| eyre!("Event bus closed"))
    }
}
```

**Why unbounded?**
- Events must never be dropped
- Backpressure is not appropriate (losing events = lost state changes)
- If event processing is slow, that's a bug to fix, not throttle

### 4.3 Event Flow Example

**User clicks "Start" on a container:**

```
Web Handler
    │
    └─> publish(StartRequested { name: "container_x" })
            │
            ▼
        Event Processor
            │
            ├─> Check member type (Container)
            ├─> Spawn process → Child
            ├─> ProcessMonitor.register_process(name, child) → PID
            │   └─> ProcessMonitor spawns monitoring task
            ├─> publish(ProcessStarted { name, pid })
            │       │
            │       ▼
            │   Event Processor (reacts to ProcessStarted)
            │       │
            │       ├─> Update Container.state = Running { pid }
            │       ├─> Get list of composable nodes
            │       ├─> publish(LoadRequested { name: "node_a" })
            │       ├─> publish(LoadRequested { name: "node_b" })
            │       └─> publish(StateChanged { name: "container_x" })
            │
            ▼
        Event Processor (reacts to LoadRequested)
            │
            ├─> Update ComposableNode.state = Loading
            ├─> Call LoadNode service
            └─> publish(LoadSucceeded { name: "node_a" })
                    │
                    ▼
                Event Processor
                    │
                    ├─> Update ComposableNode.state = Loaded
                    └─> publish(StateChanged { name: "node_a" })

(Later, when container exits...)

ProcessMonitor Task (monitoring container_x)
    │
    ├─> child.wait().await completes
    └─> publish(ProcessExited { name: "container_x", exit_code })
            │
            ▼
        Event Processor
            │
            ├─> Update Container.state = Failed or Stopped
            ├─> Get composable nodes: ["node_a", "node_b"]
            ├─> Block each composable node
            └─> publish(StateChanged) for container and composable nodes
```

---

## 5. State Machines

### 5.1 Regular Node State Machine

```
         ┌─────────────────────────────────┐
         │         Pending                 │
         │  (initial state, never started) │
         └──────────┬──────────────────────┘
                    │
                    │ StartRequested
                    ▼
         ┌─────────────────────────────────┐
         │      Running { pid }            │◄────┐
         │   (process is alive)            │     │ Restart
         └──┬────────────────────────┬─────┘     │
            │                        │           │
            │ ProcessExited(0)       │ ProcessExited(non-zero)
            │                        │           │
            ▼                        ▼           │
    ┌──────────────┐        ┌─────────────────┐ │
    │   Stopped    │        │ Failed { code } ├─┘ (if respawn enabled)
    │ (clean exit) │        │  (crashed)      │
    └──────────────┘        └─────────────────┘
         │                           │
         └──────────┬────────────────┘
                    │
                    │ StartRequested (manual)
                    ▼
               (back to Running)
```

### 5.2 Composable Node State Machine

```
                        Container Running?
                        /                 \
                      YES                 NO
                      │                   │
                      ▼                   ▼
            ┌─────────────────┐   ┌──────────────────────┐
            │    Unloaded     │   │ Blocked { reason }   │
            │  (not loaded)   │   │ (container N/A)      │
            └────────┬─────────┘  └──────────────────────┘
                     │                      ▲
                     │ LoadRequested        │
                     ▼                      │
            ┌─────────────────┐             │
            │    Loading      │             │
            │ (service call)  │             │
            └────┬──────┬─────┘             │
                 │      │                   │
   LoadSucceeded │      │ LoadFailed        │
                 ▼      ▼                   │
         ┌───────────┐  └──> Unloaded      │
         │  Loaded   │                     │
         └───────────┘                     │
                 │                         │
                 │ Container stops/fails   │
                 └─────────────────────────┘
```

**State Transitions:**

- `Unloaded → Loading`: LoadRequested event received
- `Loading → Loaded`: LoadNode service returned success
- `Loading → Unloaded`: LoadNode service failed
- `Loaded → Blocked`: Container stopped or failed
- `Blocked → Unloaded`: Container started again (auto-triggers load)

### 5.3 Container State Machine

```
         ┌─────────────────────────────────┐
         │         Pending                 │
         │  (initial state)                │
         └──────────┬──────────────────────┘
                    │
                    │ StartRequested
                    ▼
         ┌─────────────────────────────────┐
         │      Running { pid }            │◄────┐
         │   (process alive)               │     │
         │   ├─> Trigger LoadRequested     │     │ Restart
         │   └─> for all composable nodes  │     │
         └──┬────────────────────────┬─────┘     │
            │                        │           │
            │ ProcessExited(0)       │ ProcessExited(non-zero)
            │ or StopRequested       │           │
            ▼                        ▼           │
    ┌──────────────┐        ┌─────────────────┐ │
    │   Stopped    │        │ Failed { code } ├─┘ (if respawn)
    │   ├─> Block │        │  ├─> Block all  │
    │   └─> all    │        │  └─> composable │
    └──────────────┘        └─────────────────┘
         │                           │
         └──────────┬────────────────┘
                    │
                    │ StartRequested
                    ▼
               (back to Running)
               ├─> Unblock all composable nodes
               └─> Trigger LoadRequested for each
```

---

## 6. Event Processing

### 6.1 Event Processor Structure

```rust
// File: src/play_launch/src/event_processor.rs

pub struct EventProcessor {
    registry: Arc<Mutex<MemberRegistry>>,
    event_rx: mpsc::UnboundedReceiver<MemberEvent>,
    event_bus: EventBus,
    process_monitor: Arc<ProcessMonitor>,
    component_loader: Option<ComponentLoaderHandle>,
    shutdown_tx: watch::Sender<bool>,
}

impl EventProcessor {
    pub async fn run(mut self) {
        while let Some(event) = self.event_rx.recv().await {
            if let Err(e) = self.process_event(event).await {
                error!("Error processing event: {}", e);
            }
        }
    }

    async fn process_event(&mut self, event: MemberEvent) -> eyre::Result<()> {
        match event {
            MemberEvent::ProcessStarted { name, pid } => {
                self.handle_process_started(&name, pid).await?;
            }
            MemberEvent::ProcessExited { name, exit_code } => {
                self.handle_process_exited(&name, exit_code).await?;
            }
            // ... other events
        }
        Ok(())
    }
}
```

### 6.2 Key Event Handlers

**Container Started:**

```rust
async fn handle_process_started(&mut self, name: &str, pid: u32) -> eyre::Result<()> {
    let member_type = {
        let registry = self.registry.lock().await;
        registry.get_member_type(name)?
    };

    match member_type {
        MemberType::Container => {
            // Update container state
            {
                let mut registry = self.registry.lock().await;
                registry.transition_to_running(name, pid)?;
            }

            // Get composable nodes for this container
            let composable_nodes = {
                let registry = self.registry.lock().await;
                registry.get_composable_nodes_for_container(name)
            };

            // Unblock all composable nodes
            {
                let mut registry = self.registry.lock().await;
                for node_name in &composable_nodes {
                    registry.unblock_composable_node(node_name)?;
                }
            }

            // Trigger loading for all composable nodes
            for node_name in composable_nodes {
                self.event_bus.publish(MemberEvent::LoadRequested {
                    name: node_name,
                })?;
            }

            // Notify web UI
            self.event_bus.publish(MemberEvent::StateChanged { name: name.to_string() })?;
        }

        MemberType::Node => {
            let mut registry = self.registry.lock().await;
            registry.transition_to_running(name, pid)?;
            drop(registry);
            self.event_bus.publish(MemberEvent::StateChanged { name: name.to_string() })?;
        }

        _ => {}
    }

    Ok(())
}
```

**Container Stopped/Failed:**

```rust
async fn handle_process_exited(&mut self, name: &str, exit_code: Option<i32>) -> eyre::Result<()> {
    let member_type = {
        let registry = self.registry.lock().await;
        registry.get_member_type(name)?
    };

    let is_failure = exit_code.unwrap_or(0) != 0;

    match member_type {
        MemberType::Container => {
            // Update container state
            if is_failure {
                let mut registry = self.registry.lock().await;
                registry.transition_to_failed(name, exit_code)?;
                drop(registry);
                self.event_bus.publish(MemberEvent::ProcessFailed {
                    name: name.to_string(),
                    exit_code
                })?;
            } else {
                let mut registry = self.registry.lock().await;
                registry.transition_to_stopped(name)?;
                drop(registry);
                self.event_bus.publish(MemberEvent::ProcessStopped {
                    name: name.to_string()
                })?;
            }

            // Block all composable nodes
            let composable_nodes = {
                let registry = self.registry.lock().await;
                registry.get_composable_nodes_for_container(name)
            };

            let reason = if is_failure {
                BlockReason::ContainerFailed
            } else {
                BlockReason::ContainerStopped
            };

            {
                let mut registry = self.registry.lock().await;
                for node_name in &composable_nodes {
                    registry.block_composable_node(node_name, reason)?;
                }
            }

            // Publish blocked events
            for node_name in composable_nodes {
                self.event_bus.publish(MemberEvent::Blocked {
                    name: node_name.clone(),
                    reason,
                })?;
                self.event_bus.publish(MemberEvent::StateChanged { name: node_name })?;
            }

            // Notify web UI about container
            self.event_bus.publish(MemberEvent::StateChanged { name: name.to_string() })?;
        }

        MemberType::Node => {
            // Similar logic for regular nodes (without composable node handling)
        }

        _ => {}
    }

    Ok(())
}
```

### 6.3 ProcessMonitor Component

The **ProcessMonitor** is a distinct component responsible for managing process lifecycle monitoring. It owns the `Child` handles and publishes exit events.

#### Design Rationale

**Why not embed `Child` in member structs?**
1. **Ownership conflict**: `child.wait().await` consumes the `Child` - can't keep it in the struct
2. **Separation of concerns**: Members store data (state, PID), ProcessMonitor handles behavior
3. **Testability**: Can unit test monitoring logic independently
4. **Centralized cleanup**: One place to abort all monitoring tasks on shutdown

#### Implementation

```rust
// File: src/play_launch/src/process_monitor.rs

pub struct ProcessMonitor {
    event_bus: EventBus,
    shutdown_rx: watch::Receiver<bool>,
    // Track monitoring tasks for graceful shutdown
    tasks: Arc<Mutex<HashMap<String, JoinHandle<()>>>>,
}

impl ProcessMonitor {
    pub fn new(event_bus: EventBus, shutdown_rx: watch::Receiver<bool>) -> Self {
        Self {
            event_bus,
            shutdown_rx,
            tasks: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Register a process for monitoring. Returns the PID.
    /// Spawns a background task that waits for process exit and publishes ProcessExited event.
    pub async fn register_process(&self, name: String, mut child: Child) -> eyre::Result<u32> {
        let pid = child.id().ok_or_eyre("Failed to get PID")?;

        let event_bus = self.event_bus.clone();
        let mut shutdown_rx = self.shutdown_rx.clone();
        let name_clone = name.clone();

        // Spawn monitoring task (consumes child via .wait())
        let handle = tokio::spawn(async move {
            tokio::select! {
                result = child.wait() => {
                    match result {
                        Ok(exit_status) => {
                            event_bus.publish(MemberEvent::ProcessExited {
                                name: name_clone,
                                exit_code: exit_status.code(),
                            }).ok();
                        }
                        Err(e) => {
                            error!("Failed to wait for process {}: {}", name_clone, e);
                        }
                    }
                }
                _ = shutdown_rx.changed() => {
                    info!("Shutdown requested, stopping monitor for {}", name_clone);
                }
            }
        });

        // Track the task for graceful shutdown
        self.tasks.lock().await.insert(name.clone(), handle);

        Ok(pid)
    }

    /// Kill a process by PID. Does not need Child handle.
    pub async fn kill_process(&self, pid: u32, signal: Signal) -> eyre::Result<()> {
        use nix::sys::signal;
        use nix::unistd::Pid;

        signal::kill(Pid::from_raw(pid as i32), signal)
            .map_err(|e| eyre!("Failed to send signal to PID {}: {}", pid, e))?;

        Ok(())
    }

    /// Graceful shutdown with SIGTERM → wait → SIGKILL pattern
    pub async fn shutdown_process(&self, name: &str, pid: u32) -> eyre::Result<()> {
        use nix::sys::signal::{Signal, SIGTERM, SIGKILL};

        // Send SIGTERM
        self.kill_process(pid, SIGTERM).await?;

        // Wait up to 5 seconds for process to exit gracefully
        let start = tokio::time::Instant::now();
        let timeout = Duration::from_secs(5);

        while start.elapsed() < timeout {
            // Check if process still exists
            if let Err(_) = self.kill_process(pid, Signal::from_c_int(0).unwrap()).await {
                // Process is gone (signal failed)
                info!("Process {} (PID {}) exited gracefully", name, pid);
                return Ok(());
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }

        // Force kill if still alive
        warn!("Process {} (PID {}) did not exit gracefully, sending SIGKILL", name, pid);
        self.kill_process(pid, SIGKILL).await?;

        Ok(())
    }

    /// Abort all monitoring tasks (called on shutdown)
    pub async fn shutdown(&self) {
        let tasks = self.tasks.lock().await;
        for (name, handle) in tasks.iter() {
            info!("Aborting monitoring task for {}", name);
            handle.abort();
        }
    }
}
```

#### Monitoring Task State Machine

Each call to `register_process()` spawns an independent monitoring task. The task's lifecycle:

```
                    register_process(name, child) called
                                  │
                                  ▼
                        ┌──────────────────┐
                        │   Not Monitored  │
                        │  (just spawned)  │
                        └─────────┬────────┘
                                  │
                                  │ Spawn monitoring task
                                  │ Store in tasks HashMap
                                  ▼
                        ┌──────────────────────────┐
                        │      Monitoring          │
                        │                          │
                        │  tokio::select! {        │
                        │    child.wait() => ...   │◄─────┐
                        │    shutdown_rx => ...    │      │
                        │  }                       │      │
                        └──┬──────────────────┬────┘      │
                           │                  │           │
          Process exits    │                  │  Shutdown │
          (child.wait()    │                  │  signal   │
           completes)      │                  │  received │
                           │                  │           │
                           ▼                  ▼           │
                ┌──────────────────┐  ┌─────────────┐    │
                │  Exit Detected   │  │  Shutdown   │    │
                │                  │  │  Requested  │    │
                │ Publish:         │  │             │    │
                │ ProcessExited {  │  │ Break loop, │    │
                │   name,          │  │ task ends   │    │
                │   exit_code      │  │             │    │
                │ }                │  └──────┬──────┘    │
                └─────────┬────────┘         │           │
                          │                  │           │
                          └────────┬─────────┘           │
                                   │                     │
                                   ▼                     │
                          ┌──────────────────┐           │
                          │    Completed     │           │
                          │                  │           │
                          │ Task removed     │           │
                          │ from HashMap     │           │
                          └──────────────────┘           │
                                   │                     │
                                   │                     │
                     If respawn enabled:                 │
                     EventProcessor publishes             │
                     StartRequested after delay ──────────┘
                     (creates new monitoring task)
```

**State Transitions:**

| From State | Event | To State | Action |
|------------|-------|----------|--------|
| Not Monitored | `register_process()` called | Monitoring | Spawn task, insert into HashMap |
| Monitoring | `child.wait()` completes | Exit Detected | Publish `ProcessExited` event |
| Monitoring | `shutdown_rx.changed()` | Shutdown Requested | Break loop, end task |
| Exit Detected | Event published | Completed | Remove from HashMap |
| Shutdown Requested | Task ends | Completed | Remove from HashMap |

**Key Points:**
- Each process has exactly **one** monitoring task
- Tasks are independent (one crashing doesn't affect others)
- Shutdown signal (`shutdown_rx`) allows graceful cleanup
- If respawn enabled, EventProcessor creates a **new** monitoring task (full cycle restart)
- Tasks clean themselves up (removed from HashMap when complete)

**Relationship to Member States:**

The monitoring task state is **separate** from the member's ProcessState:

```
Monitoring Task State          Member ProcessState (in Registry)
─────────────────────          ───────────────────────────────────

Not Monitored                  Pending
       │                              │
       │ register_process()           │
       ▼                              │
  Monitoring  ───────────────────────►│
       │      (ProcessStarted event)  ▼
       │                           Running { pid }
       │                              │
       │ child.wait() completes       │
       ▼                              │
Exit Detected ─────────────────────►  │
       │      (ProcessExited event)   ▼
       │                           Stopped (exit_code == 0)
       ▼                           OR
  Completed                        Failed { exit_code }
```

**Two separate concerns:**
- **ProcessMonitor**: Owns `Child`, detects exits, publishes events
- **MemberRegistry**: Stores `PID` and `ProcessState`, updates on events

#### Usage Flow

**Starting a process:**
```rust
// In EventProcessor::handle_start_requested()
async fn handle_start_requested(&mut self, name: &str) -> eyre::Result<()> {
    // 1. Get command line from registry
    let cmdline = {
        let registry = self.registry.lock().await;
        registry.get_cmdline(name)?
    };

    // 2. Spawn process
    let child = Command::new(&cmdline.program)
        .args(&cmdline.args)
        .spawn()?;

    // 3. Register with ProcessMonitor (returns PID, spawns monitoring task)
    let pid = self.process_monitor.register_process(name.to_string(), child).await?;

    // 4. Publish ProcessStarted event
    self.event_bus.publish(MemberEvent::ProcessStarted {
        name: name.to_string(),
        pid,
    })?;

    Ok(())
}
```

**Stopping a process:**
```rust
// In EventProcessor::handle_stop_requested()
async fn handle_stop_requested(&mut self, name: &str) -> eyre::Result<()> {
    // 1. Get PID from member state
    let pid = {
        let registry = self.registry.lock().await;
        match registry.get_state(name)? {
            ProcessState::Running { pid } => pid,
            _ => return Err(eyre!("Process {} is not running", name)),
        }
    };

    // 2. Use ProcessMonitor to gracefully shutdown
    self.process_monitor.shutdown_process(name, pid).await?;

    // 3. Monitoring task will detect exit and publish ProcessExited event
    // 4. EventProcessor will handle ProcessExited and update state

    Ok(())
}
```

**Process exits naturally:**
```
1. Process terminates
2. Monitoring task's child.wait() completes
3. Monitoring task publishes ProcessExited { name, exit_code } event
4. EventProcessor receives event
5. EventProcessor updates member state (Running → Stopped or Failed)
6. EventProcessor triggers cascading effects (e.g., block composable nodes if container failed)
```

#### Benefits

1. **Clean ownership**: `Child` owned by monitoring task, `PID` stored in member state
2. **No lifetime issues**: Members don't need to track async tasks
3. **Centralized logic**: All process management in one place
4. **Testable**: Can mock ProcessMonitor for testing EventProcessor
5. **Graceful shutdown**: Single place to implement SIGTERM → SIGKILL pattern

---

## 7. Container-Composable Node Interactions

### 7.1 Indexing Structure

```rust
pub struct MemberRegistry {
    members: HashMap<String, Member>,

    // NEW: Efficient bidirectional indexes
    container_to_composable: HashMap<String, Vec<String>>,
    composable_to_container: HashMap<String, String>,
}
```

**Why indexes?**
- O(1) lookup instead of O(n) iteration
- Clear ownership: one composable node belongs to exactly one container
- Easy to implement cascade operations

### 7.2 Interaction Scenarios

**Scenario 1: User Starts Container**

```
1. User clicks "Start" on container_x
2. Web handler publishes: StartRequested { name: "container_x" }
3. Event processor spawns process
4. Process starts, event processor publishes: ProcessStarted { name: "container_x", pid }
5. Event processor handles ProcessStarted:
   a. Updates container state to Running
   b. Looks up composable_nodes via index: ["node_a", "node_b"]
   c. Unblocks each composable node (Blocked → Unloaded)
   d. Publishes LoadRequested for each: LoadRequested { name: "node_a" }, ...
6. Event processor handles LoadRequested:
   a. Updates composable node state to Loading
   b. Calls LoadNode service
   c. On success, publishes: LoadSucceeded { name: "node_a" }
7. Event processor handles LoadSucceeded:
   a. Updates composable node state to Loaded
   b. Publishes StateChanged for web UI
```

**Scenario 2: Container Crashes**

```
1. Container process exits with code 1
2. Process monitor publishes: ProcessExited { name: "container_x", exit_code: 1 }
3. Event processor handles ProcessExited:
   a. Determines this is a failure (exit_code != 0)
   b. Updates container state to Failed { exit_code: 1 }
   c. Publishes ProcessFailed { name: "container_x", exit_code: 1 }
4. Event processor handles ProcessFailed:
   a. Looks up composable nodes: ["node_a", "node_b"]
   b. Blocks each: node_a.state = Blocked { reason: ContainerFailed }
   c. Publishes Blocked { name: "node_a", reason: ContainerFailed }
   d. Publishes StateChanged for each composable node
   e. Publishes StateChanged for container
5. Web UI receives StateChanged events, refreshes display
   - container_x shows red (Failed)
   - node_a, node_b show gray/blocked (Blocked - Container Failed)
```

**Scenario 3: User Restarts Container**

```
1. User clicks "Restart" on container_x (currently Failed)
2. Web handler publishes: RestartRequested { name: "container_x" }
3. Event processor handles RestartRequested:
   a. If running, publishes StopRequested first
   b. Once stopped, publishes StartRequested
4. (Same flow as Scenario 1: container starts, composable nodes load)
```

### 7.3 Filesystem Synchronization

Events trigger filesystem writes for persistence/debugging:

```rust
async fn handle_blocked(&mut self, name: &str, reason: BlockReason) -> eyre::Result<()> {
    // Update in-memory state
    {
        let mut registry = self.registry.lock().await;
        registry.block_composable_node(name, reason)?;
    }

    // Write marker file for debugging/inspection
    let output_dir = {
        let registry = self.registry.lock().await;
        registry.get_output_dir(name)?.to_path_buf()
    };

    let terminated_path = output_dir.join("terminated");
    let reason_str = match reason {
        BlockReason::ContainerStopped => "container_stopped",
        BlockReason::ContainerFailed => "container_failed",
        BlockReason::ContainerNotStarted => "container_not_started",
    };

    tokio::fs::write(&terminated_path, reason_str).await?;

    Ok(())
}
```

**Filesystem role:**
- **Before:** Source of truth (state computed from files)
- **After:** Audit trail (files written after state changes)

---

## 8. Web UI Integration

### 8.1 Handler Changes

**Old (Direct Manipulation):**

```rust
pub async fn start_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    let mut registry = state.registry.lock().await;

    // Lots of logic here: check type, spawn process, update state, etc.
    match registry.start_node(&name).await {
        Ok(pid) => (StatusCode::OK, format!("Started with PID {}", pid)).into_response(),
        Err(e) => (StatusCode::BAD_REQUEST, e.to_string()).into_response(),
    }
}
```

**New (Event-Driven):**

```rust
pub async fn start_node(State(state): State<Arc<WebState>>, Path(name): Path<String>) -> Response {
    // Just publish event and return immediately
    if let Err(e) = state.event_bus.publish(MemberEvent::StartRequested { name: name.clone() }) {
        return (StatusCode::INTERNAL_SERVER_ERROR, e.to_string()).into_response();
    }

    // Success response (state change will be visible on next poll)
    (StatusCode::ACCEPTED, format!("Start requested for '{}'", name)).into_response()
}
```

**Benefits:**
- Handlers become thin wrappers
- No business logic in HTTP layer
- Async by default (don't block HTTP thread)

### 8.2 State Polling vs SSE

**Current: Polling**

```javascript
// index.html - polls every 5 seconds
hx-get="/api/nodes"
hx-trigger="load, every 5s"
```

**Future: SSE (Optional Enhancement)**

```rust
pub async fn stream_state_changes(
    State(state): State<Arc<WebState>>,
) -> Sse<impl Stream<Item = Result<Event, Infallible>>> {
    let rx = state.event_bus.subscribe_state_changes();

    let stream = BroadcastStream::new(rx)
        .filter_map(|event| async move {
            match event {
                Ok(MemberEvent::StateChanged { name }) => {
                    Some(Ok(Event::default().data(name)))
                }
                _ => None,
            }
        });

    Sse::new(stream)
}
```

**Web UI:**

```javascript
const eventSource = new EventSource('/api/state-changes');
eventSource.onmessage = (event) => {
    // Refresh only the changed node card
    htmx.ajax('GET', `/api/nodes/${event.data}`, {target: `#node-${event.data}`});
};
```

**Migration Path:**
1. Keep polling for now (minimal changes)
2. Add SSE endpoint later as enhancement
3. Web UI can use both (SSE for instant updates, polling as fallback)

---

## 9. Migration Strategy

### Phase 1: Foundation (1-2 weeks)

**Goal:** Create new structures without breaking existing code

**Tasks:**
- [ ] Create `src/play_launch/src/member.rs`
  - Define `RegularNode`, `Container`, `ComposableNode`
  - Define `ProcessState`, `ComposableState`, `BlockReason`
  - Define `Member` enum
  - **NOTE:** No `Child` field in structs (ProcessMonitor owns it)

- [ ] Create `src/play_launch/src/events.rs`
  - Define `MemberEvent` enum
  - Implement `EventBus`
  - Add unit tests for event bus

- [ ] Create `src/play_launch/src/process_monitor.rs`
  - Define `ProcessMonitor` struct
  - Implement `register_process()` - spawns monitoring task per process
  - Implement `kill_process()` - sends signal via PID
  - Implement `shutdown_process()` - graceful SIGTERM → SIGKILL
  - Implement `shutdown()` - abort all monitoring tasks
  - Add unit tests with mock processes

- [ ] Create `src/play_launch/src/event_processor.rs`
  - Stub implementation (handlers do nothing)
  - Add `process_monitor: Arc<ProcessMonitor>` field
  - Wire up event loop with shutdown signal
  - Add logging for all events

- [ ] Create `src/play_launch/src/member_registry.rs`
  - Port existing `NodeRegistry` methods
  - Add indexing: `container_to_composable`, `composable_to_container`
  - Add state transition methods with validation
  - Methods accept/return PIDs (not Child handles)

**Success Criteria:**
- Code compiles
- All tests pass
- New code is not yet used (old code still works)

### Phase 2: Event Processor Implementation (2-3 weeks)

**Goal:** Implement event handling logic

**Tasks:**
- [ ] Implement `handle_process_started()`
- [ ] Implement `handle_process_exited()`
- [ ] Implement `handle_process_stopped()`
- [ ] Implement `handle_process_failed()`
- [ ] Implement `handle_load_requested()`
- [ ] Implement `handle_load_succeeded()`
- [ ] Implement `handle_load_failed()`
- [ ] Implement `handle_blocked()`
- [ ] Implement `handle_start_requested()` (dispatch by type)
- [ ] Implement `handle_stop_requested()`
- [ ] Implement `handle_restart_requested()`
- [ ] Add unit tests for each handler

**Success Criteria:**
- Event processor handles all event types
- State transitions are correct
- Tests verify cascading behavior (container stop → composable block)

### Phase 3: Process Monitoring Integration (1 week)

**Goal:** Integrate ProcessMonitor with execution.rs

**Tasks:**
- [ ] Update `spawn_nodes()`, `spawn_containers()` to use ProcessMonitor
  - Spawn process → get `Child`
  - Call `process_monitor.register_process(name, child)` → get PID
  - Publish `ProcessStarted { name, pid }` event
  - **Remove** direct `child.wait()` calls (ProcessMonitor handles this)

- [ ] Handle respawn via events
  - On `ProcessExited`, check if respawn enabled
  - Use `tokio::time::sleep(respawn_delay)`
  - Publish `StartRequested` after delay
  - Respect shutdown signal (don't respawn during shutdown)

- [ ] Remove direct state manipulation from execution.rs
  - All state changes go through events
  - execution.rs only spawns processes and publishes events

**Success Criteria:**
- All node lifecycle events are published via ProcessMonitor
- Event processor receives and handles ProcessExited events
- Monitoring tasks properly clean up on shutdown
- Old code path can be disabled via feature flag

### Phase 4: Web Handlers Integration (1 week)

**Goal:** Make web handlers publish events

**Tasks:**
- [ ] Add `event_bus` field to `WebState`
- [ ] Update `start_node()` handler to publish `StartRequested`
- [ ] Update `stop_node()` handler to publish `StopRequested`
- [ ] Update `restart_node()` handler to publish `RestartRequested`
- [ ] Update `toggle_respawn()` handler to publish `RespawnToggled`
- [ ] Change HTTP status codes to `202 Accepted` (async operation)

**Success Criteria:**
- Web UI still works
- Actions go through event system
- State changes are visible on next poll

### Phase 5: Web UI Enhancements (1 week)

**Goal:** Update frontend for new state types

**Tasks:**
- [ ] Update `render_node_card()` to handle `MemberState` enum
- [ ] Add CSS for `Blocked` state (gray background, chain icon)
- [ ] Show block reason in tooltip ("Container stopped", "Container failed")
- [ ] Update `Loading` state display (spinner icon)
- [ ] Test all state transitions visually

**Success Criteria:**
- Web UI correctly displays all states
- Visual feedback for blocked composable nodes
- Hover tooltips explain why node is blocked

### Phase 6: Cleanup & Documentation (1 week)

**Goal:** Remove old code, update docs

**Tasks:**
- [ ] Remove old `NodeRegistry` (rename `MemberRegistry` → `Registry`)
- [ ] Remove `get_status_with_registry()` hack
- [ ] Remove filesystem-based state inference
- [ ] Update CLAUDE.md with new architecture
- [ ] Update API documentation
- [ ] Add architecture diagram to docs/

**Success Criteria:**
- No dead code
- All tests pass
- Documentation is up to date

### Phase 7: Testing & Validation (1 week)

**Goal:** Comprehensive testing

**Tasks:**
- [ ] Integration test: start container → composable nodes load
- [ ] Integration test: stop container → composable nodes block
- [ ] Integration test: restart container → composable nodes reload
- [ ] Integration test: container crash → composable nodes fail
- [ ] Integration test: respawn with composable nodes
- [ ] Stress test: rapid start/stop cycles
- [ ] Test with Autoware planning simulator (52 nodes)

**Success Criteria:**
- All integration tests pass
- No race conditions
- No state inconsistencies
- Performance is acceptable

---

## 10. Benefits

### 10.1 Separation of Concerns

**Before:**
```rust
// NodeHandle tries to do everything
impl NodeHandle {
    pub fn get_status(&self) -> UnifiedStatus {
        match self.node_type {
            NodeType::Node => { /* ... */ },
            NodeType::Container => { /* ... */ },
            NodeType::ComposableNode => { /* ... */ },
        }
    }
}
```

**After:**
```rust
// Each type has its own impl
impl RegularNode {
    pub fn state(&self) -> ProcessState { self.state }
}

impl Container {
    pub fn state(&self) -> ProcessState { self.state }
    pub fn composable_nodes(&self) -> &[String] { &self.composable_nodes }
}

impl ComposableNode {
    pub fn state(&self) -> ComposableState { self.state }
    pub fn container(&self) -> &str { &self.container_name }
}
```

### 10.2 Explicit State Management

**Before:**
- State inferred from files every time
- Expensive: ~50 file reads per web UI refresh
- Race-prone: file might change during read

**After:**
- State stored in memory
- Fast: O(1) lookup in HashMap
- Consistent: atomic state transitions

### 10.3 Reactive Architecture

**Before:**
- Poll filesystem every 5 seconds
- Miss events between polls
- No audit trail

**After:**
- Events trigger immediate actions
- Every state change is logged
- Can add SSE for instant web UI updates

### 10.4 Testability

**Before:**
```rust
// Hard to test - needs real processes and files
#[test]
fn test_container_stop_blocks_composable() {
    // Need to spawn actual container
    // Need to create filesystem structure
    // Hard to mock
}
```

**After:**
```rust
// Easy to test - pure event processing
#[test]
fn test_container_stop_blocks_composable() {
    let (event_bus, rx) = EventBus::new();
    let registry = Arc::new(Mutex::new(MemberRegistry::new(event_bus.clone())));
    let mut processor = EventProcessor::new(registry.clone(), rx, event_bus, None, shutdown_tx);

    // Publish event
    processor.event_bus.publish(MemberEvent::ProcessExited {
        name: "container_x".to_string(),
        exit_code: Some(1),
    }).unwrap();

    // Process event
    processor.process_event(/* ... */).await.unwrap();

    // Verify state
    let reg = registry.lock().await;
    assert_eq!(reg.get_state("node_a"), ComposableState::Blocked { reason: ContainerFailed });
}
```

### 10.5 Type Safety

**Before:**
```rust
// Runtime type checking
if handle.node_type == NodeType::Container {
    if let NodeInfo::Regular(info) = &handle.info {
        // What if this fails? Panic!
    }
}
```

**After:**
```rust
// Compiler enforces type safety
match member {
    Member::Container(container) => {
        // container.composable_nodes is guaranteed to exist
        for node in &container.composable_nodes { /* ... */ }
    }
    Member::Node(node) => {
        // node doesn't have composable_nodes - compiler error if you try
    }
    Member::ComposableNode(comp) => {
        // comp.container_name is guaranteed to exist
    }
}
```

### 10.6 Extensibility

**Adding new event types:**
```rust
// Just add to enum
pub enum MemberEvent {
    // ... existing events
    ResourceThresholdExceeded { name: String, metric: String, value: f64 },
}

// Implement handler
async fn handle_resource_threshold_exceeded(&mut self, name: &str, metric: &str, value: f64) {
    // Decide: stop node? Send notification? Log warning?
}
```

**Adding new member types:**
```rust
pub struct LifecycleNode {
    pub name: String,
    pub state: LifecycleState,
    // ... lifecycle-specific fields
}

pub enum Member {
    Node(RegularNode),
    Container(Container),
    ComposableNode(ComposableNode),
    LifecycleNode(LifecycleNode),  // NEW
}
```

### 10.7 Debugging

**Before:**
- "Why is this node in this state?" → No idea, state is computed
- "When did this state change?" → No logs
- "What triggered this?" → Unknown

**After:**
- Every event is logged with timestamp
- Every state transition has a reason (event)
- Full audit trail: `grep "node_a" event_log.txt` shows entire lifecycle

---

## 11. Open Questions / Future Work

### 11.1 Event Persistence

**Question:** Should events be persisted to disk?

**Options:**
1. **No persistence** (current plan)
   - Pros: Simple, fast
   - Cons: Lose audit trail on restart

2. **Event log file** (append-only)
   - Pros: Full audit trail, can replay events
   - Cons: Disk I/O overhead, log rotation needed

3. **Event sourcing** (events are source of truth)
   - Pros: Can reconstruct state at any point in time
   - Cons: Complex, requires event replay on startup

**Recommendation:** Start with no persistence, add event log file later if needed for debugging.

### 11.2 Circular Dependencies

**Question:** What if container A loads composable node X, which somehow depends on container B?

**Current Plan:**
- Not supported (single-level hierarchy only)
- Composable node belongs to exactly one container

**Future:**
- Detect circular dependencies during registration
- Reject invalid configurations

### 11.3 Concurrent Operations

**Question:** What if user clicks "Stop" while node is starting?

**Current Plan:**
- Events are processed sequentially in order received
- Later event wins (StopRequested after StartRequested → node starts then stops)

**Improvement:**
- Track pending operations per member
- Reject conflicting operations (return 409 Conflict)

### 11.4 Partial Failures

**Question:** What if 2/3 composable nodes load successfully?

**Current Plan:**
- Each composable node has independent state
- Some can be Loaded while others are Unloaded

**Future Enhancement:**
- Container-level health metric: "2/3 composable nodes loaded"
- Option to mark container as degraded

---

## 12. Summary

This refactoring transforms play_launch from a **filesystem-based, polling architecture** to an **event-driven, state-managed architecture**.

**Key Changes:**
1. **Three distinct types** instead of one unified type
2. **Explicit states** stored in memory instead of inferred from filesystem
3. **Event system** for decoupled, reactive behavior
4. **Indexed relationships** between containers and composable nodes
5. **Type-safe** state transitions
6. **ProcessMonitor component** - distinct component for process lifecycle management

**New Components:**
- **MemberRegistry**: Pure data storage (state, PID, configuration)
- **EventBus**: Async event distribution via mpsc channels
- **EventProcessor**: Business logic and state transition orchestration
- **ProcessMonitor**: Process spawning, monitoring, and graceful shutdown

**Timeline:** ~8-10 weeks for full migration

**Risk Level:** Medium
- Breaking changes to internal architecture
- Can be done incrementally (old code works during migration)
- Extensive testing needed

**Payoff:**
- Much cleaner architecture
- Easier to debug and maintain
- Better performance (no filesystem polling)
- Extensible for future features
- Testable components (can mock ProcessMonitor, EventBus, etc.)

---

**Next Steps:**
1. Review this document and provide feedback
2. Approve plan or request changes
3. Begin Phase 1 implementation
