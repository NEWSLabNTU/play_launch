# Phase 11: Web UI Actor Integration

**Status**: ⏳ Planned
**Priority**: High
**Estimated Duration**: 2-3 weeks
**Dependencies**: Phase 10 (Actor Pattern Complete)

---

## Overview

Migrate the web UI to work directly with the actor framework, eliminating the bridging layer and event-driven compatibility code. Currently, the web UI uses an inefficient bridging pattern where StateEvents are translated to MemberEvents and the Registry acts as a read-only cache. Control commands from the web UI have no path to reach actors.

### Current Architecture (Bridged)

```
┌─────────────┐
│   Actors    │──StateEvents──►┐
└─────────────┘                │
                               ├──► bridge_state_event_to_web_ui()
┌─────────────┐                │        │
│  Registry   │◄───────────────┘        │
│ (read cache)│                          ▼
└──────┬──────┘                    ┌──────────┐
       │ queries                   │ EventBus │
       │                           └────┬─────┘
       │                                │ SSE
┌──────▼──────┐                        │
│  Web UI     │◄───────────────────────┘
│  Handlers   │
└──────┬──────┘
       │ publishes MemberEvents
       ▼
┌──────────┐
│ EventBus │──► (NO HANDLER - commands lost!)
└──────────┘
```

**Problems:**
1. **Broken Control Path**: Web UI publishes MemberEvents (Start/Stop/Restart), but no handler translates them to ControlEvents for actors
2. **Redundant Translation**: StateEvents → MemberEvents → SSE (unnecessary conversion)
3. **Stale Registry**: Registry is a write-through cache that duplicates actor state
4. **EventBus Dependency**: Still requires event_driven module for web UI compatibility

### Proposed Architecture (Direct)

```
┌─────────────┐
│   Actors    │──StateEvents──►┐
└─────────────┘                │
                               │
┌──────────────┐               │
│ Coordinator  │◄──ControlEvents (from web UI)
│  (central)   │               │
└──────┬───────┘               │
       │ actor handles         │
       │                       │
┌──────▼──────┐                │
│  Web UI     │◄───StateEvents─┘
│  Handlers   │      (direct SSE)
└─────────────┘
```

**Benefits:**
1. **Direct Control**: Web UI sends ControlEvents directly to Coordinator → Actors
2. **No Translation**: StateEvents streamed directly to SSE clients
3. **Single Source of Truth**: Actors hold authoritative state, no cache duplication
4. **Clean Architecture**: Remove event_driven module dependency from web code
5. **Lower Latency**: Eliminate bridging overhead

---

## Goals

### Primary Goals
1. **Direct Actor Control**: Web UI handlers send ControlEvents to actors via MemberCoordinator
2. **Remove Bridging Layer**: Eliminate bridge_state_event_to_web_ui() functions
3. **Remove Registry Dependency**: Query actor state directly from MemberCoordinator
4. **Direct SSE Streaming**: Stream StateEvents directly to web UI clients
5. **Delete event_driven Module**: Remove events.rs, member.rs, registry.rs (web UI-only code)

### Non-Goals
- Not changing web UI frontend (HTML/CSS/JS remains the same)
- Not changing API endpoints (same routes and responses)
- Not adding new features (pure refactoring)

---

## Current State Analysis

### What Works
- ✅ Actors publish StateEvents to Coordinator
- ✅ Web UI can query node list (via Registry)
- ✅ Web UI can view node details (via Registry)
- ✅ SSE log streaming works (file-based, independent of actors)

### What's Broken
- ❌ **Control commands don't work**: Start/Stop/Restart buttons publish MemberEvents with no handler
- ❌ **Bulk operations broken**: Start All/Stop All/Restart All have no implementation
- ❌ **Respawn toggle broken**: RespawnToggled event published but not handled

### What's Inefficient
- ⚠️ Registry duplication: Same state stored in actors and Registry
- ⚠️ Event translation: StateEvent → MemberEvent → SSE (extra conversion)
- ⚠️ Two event systems: StateEvents for actors, MemberEvents for web UI

---

## Phases

### Phase 11.1: WebState Refactoring (Week 1)

**Goal**: Replace WebState to use MemberCoordinator instead of Registry + EventBus.

#### Tasks

1. **Create New WebState Structure**
   ```rust
   pub struct WebState {
       /// Coordinator for actor control and state queries
       pub coordinator: Arc<TokioMutex<MemberCoordinator>>,
       /// Base log directory (used for log file access)
       pub log_dir: PathBuf,
       /// Track operations in progress (to prevent racing)
       pub operations_in_progress: TokioMutex<HashSet<String>>,
   }
   ```

2. **Add Query Methods to MemberCoordinator**
   ```rust
   impl MemberCoordinator {
       /// Get list of all members with current state
       pub fn list_members(&self) -> Vec<MemberSummary> { ... }

       /// Get detailed state for a specific member
       pub fn get_member_state(&self, name: &str) -> Option<MemberState> { ... }

       /// Get health summary statistics
       pub fn get_health_summary(&self) -> HealthSummary { ... }
   }
   ```

3. **Define MemberSummary/MemberState Types**
   ```rust
   pub struct MemberSummary {
       pub name: String,
       pub member_type: MemberType,
       pub state: MemberState,
       pub pid: Option<u32>,
       pub package: Option<String>,
       pub executable: String,
       pub namespace: Option<String>,
       pub target_container: Option<String>,
       pub respawn_enabled: Option<bool>,
       pub respawn_delay: Option<f64>,
   }

   pub enum MemberState {
       Pending,
       Running { pid: u32 },
       Respawning { attempt: u32 },
       Stopped,
       Failed { error: String },
       // Composable-specific states
       Loading,
       Loaded { unique_id: u64 },
       Blocked { reason: BlockReason },
   }
   ```

4. **Update web/mod.rs**
   - Change WebState constructor signature
   - Remove Registry and EventBus parameters
   - Add MemberCoordinator parameter

**Deliverables:**
- New WebState structure with MemberCoordinator
- Query methods on MemberCoordinator for web UI
- Updated web/mod.rs with new constructor

**Testing:**
- Unit tests for query methods
- Verify compilation (handlers won't work yet)

---

### Phase 11.2: Handler Control Path (Week 1-2)

**Goal**: Update handlers to send ControlEvents to actors via MemberCoordinator.

#### Tasks

1. **Add Control Methods to MemberCoordinator**
   ```rust
   impl MemberCoordinator {
       /// Send control event to a specific actor
       pub async fn send_control(
           &self,
           name: &str,
           event: ControlEvent,
       ) -> Result<()> {
           if let Some(handle) = self.actors.get(name) {
               handle.send_control(event).await
           } else {
               Err(eyre!("Actor '{}' not found", name))
           }
       }

       /// Start a member (send Start control event)
       pub async fn start_member(&self, name: &str) -> Result<()> {
           self.send_control(name, ControlEvent::Start).await
       }

       /// Stop a member (send Stop control event)
       pub async fn stop_member(&self, name: &str) -> Result<()> {
           self.send_control(name, ControlEvent::Stop).await
       }

       /// Restart a member (send Restart control event)
       pub async fn restart_member(&self, name: &str) -> Result<()> {
           self.send_control(name, ControlEvent::Restart).await
       }

       /// Toggle respawn for a member
       pub async fn toggle_respawn(&self, name: &str, enabled: bool) -> Result<()> {
           self.send_control(name, ControlEvent::ToggleRespawn(enabled)).await
       }
   }
   ```

2. **Update web/handlers.rs Control Endpoints**
   ```rust
   /// Start a node (direct actor control)
   pub async fn start_node(
       State(state): State<Arc<WebState>>,
       Path(name): Path<String>,
   ) -> Response {
       let _guard = match OperationGuard::try_acquire(name.clone(), state.clone()).await {
           Ok(guard) => guard,
           Err(e) => return (StatusCode::CONFLICT, e).into_response(),
       };

       let coordinator = state.coordinator.lock().await;
       match coordinator.start_member(&name).await {
           Ok(()) => {
               info!("[Web UI] Sent Start control to '{}'", name);
               (StatusCode::ACCEPTED, format!("Start request for '{}' sent", name))
                   .into_response()
           }
           Err(e) => {
               warn!("[Web UI] Failed to start '{}': {}", name, e);
               (StatusCode::INTERNAL_SERVER_ERROR, format!("Failed to start: {}", e))
                   .into_response()
           }
       }
   }

   // Similar updates for stop_node, restart_node, toggle_respawn
   ```

3. **Update web/handlers.rs Query Endpoints**
   ```rust
   /// List all nodes - returns HTML fragment for htmx
   pub async fn list_nodes(State(state): State<Arc<WebState>>) -> Response {
       let coordinator = state.coordinator.lock().await;
       let nodes = coordinator.list_members();

       // Render HTML (same logic as before, different data source)
       let mut html = String::new();
       // ... render logic ...

       Html(html).into_response()
   }

   /// Get node details - returns JSON
   pub async fn get_node(
       State(state): State<Arc<WebState>>,
       Path(name): Path<String>,
   ) -> Response {
       let coordinator = state.coordinator.lock().await;

       if let Some(member_state) = coordinator.get_member_state(&name) {
           Json(json!(member_state)).into_response()
       } else {
           (StatusCode::NOT_FOUND, "Node not found").into_response()
       }
   }
   ```

4. **Remove EventBus Dependencies**
   - Delete all `event_bus.publish()` calls from handlers.rs
   - Remove EventBus field from WebState
   - Remove MemberEvent imports

**Deliverables:**
- Control methods on MemberCoordinator
- Updated handlers using direct actor control
- No more EventBus usage in web code

**Testing:**
- Integration test: Start/Stop/Restart via web API
- Verify control events reach actors
- Test with multiple concurrent operations

---

### Phase 11.3: SSE Direct Streaming (Week 2)

**Goal**: Stream StateEvents directly to SSE clients without MemberEvent translation.

#### Tasks

1. **Create SSE StateEvent Broadcaster**
   ```rust
   /// Broadcaster for StateEvents to SSE clients
   pub struct StateEventBroadcaster {
       /// Subscribers (each is a channel sender to SSE connection)
       subscribers: Arc<TokioMutex<Vec<mpsc::Sender<StateEvent>>>>,
   }

   impl StateEventBroadcaster {
       /// Subscribe to state events (for SSE connection)
       pub async fn subscribe(&self) -> mpsc::Receiver<StateEvent> {
           let (tx, rx) = mpsc::channel(100);
           let mut subs = self.subscribers.lock().await;
           subs.push(tx);
           rx
       }

       /// Broadcast a state event to all subscribers
       pub async fn broadcast(&self, event: StateEvent) {
           let mut subs = self.subscribers.lock().await;
           // Remove disconnected subscribers
           subs.retain(|tx| !tx.is_closed());
           // Send to all active subscribers
           for tx in subs.iter() {
               let _ = tx.send(event.clone()).await;
           }
       }
   }
   ```

2. **Add Broadcaster to WebState**
   ```rust
   pub struct WebState {
       pub coordinator: Arc<TokioMutex<MemberCoordinator>>,
       pub log_dir: PathBuf,
       pub operations_in_progress: TokioMutex<HashSet<String>>,
       /// Broadcaster for state events to SSE clients
       pub state_broadcaster: Arc<StateEventBroadcaster>,
   }
   ```

3. **Create Event Forwarding Task**
   ```rust
   /// Forward state events from coordinator to web UI broadcaster
   async fn forward_state_events(
       coordinator: Arc<TokioMutex<MemberCoordinator>>,
       broadcaster: Arc<StateEventBroadcaster>,
   ) {
       loop {
           let event = {
               let mut coord = coordinator.lock().await;
               coord.next_state_event().await
           };

           match event {
               Some(event) => {
                   broadcaster.broadcast(event).await;
               }
               None => {
                   // Coordinator finished
                   break;
               }
           }
       }
   }
   ```

4. **Update SSE Endpoint for State Updates**
   ```rust
   /// SSE endpoint for node state updates
   pub async fn stream_state_updates(
       State(state): State<Arc<WebState>>,
   ) -> Response {
       let rx = state.state_broadcaster.subscribe().await;

       let stream = async_stream::stream! {
           let mut rx = rx;
           while let Some(event) = rx.recv().await {
               // Convert StateEvent to SSE message
               let json = serde_json::to_string(&event).unwrap();
               yield Ok::<_, axum::Error>(
                   axum::response::sse::Event::default().data(json)
               );
           }
       };

       axum::response::Sse::new(stream).into_response()
   }
   ```

5. **Update commands/replay.rs and commands/run.rs**
   - Spawn forward_state_events task when web UI enabled
   - Remove bridge_state_event_to_web_ui() functions
   - Remove Registry population for web UI

**Deliverables:**
- StateEventBroadcaster for SSE
- Direct StateEvent → SSE streaming
- Removed bridging layer

**Testing:**
- Test SSE connection receives StateEvents
- Verify all event types stream correctly
- Test multiple concurrent SSE connections

---

### Phase 11.4: Remove event_driven Module (Week 2-3)

**Goal**: Delete event_driven module and all compatibility code.

#### Tasks

1. **Move Needed Types to member_actor**
   - If any types from event_driven/member.rs are needed, move to member_actor/mod.rs
   - Update imports throughout codebase

2. **Delete event_driven Module**
   ```bash
   rm -rf src/play_launch/src/event_driven/
   ```

3. **Update Module Exports**
   - Remove `pub mod event_driven;` from lib.rs
   - Update all imports to use member_actor types instead

4. **Update Documentation**
   - Remove references to Registry and EventBus from docs
   - Update architecture diagrams
   - Document direct actor control in CLAUDE.md

5. **Clean Up Unused Code**
   - Search for any remaining EventBus or Registry references
   - Remove dead code warnings

**Deliverables:**
- event_driven module deleted
- All imports updated
- Documentation updated
- No dead code warnings

**Testing:**
- Full build passes
- All quality checks pass
- Web UI fully functional

---

### Phase 11.5: Bulk Operations (Week 3)

**Goal**: Implement Start All / Stop All / Restart All using actor framework.

#### Tasks

1. **Add Bulk Methods to MemberCoordinator**
   ```rust
   impl MemberCoordinator {
       /// Start all members
       pub async fn start_all(&self) -> Result<Vec<String>> {
           let mut started = Vec::new();
           for (name, handle) in &self.actors {
               if handle.send_control(ControlEvent::Start).await.is_ok() {
                   started.push(name.clone());
               }
           }
           Ok(started)
       }

       /// Stop all members
       pub async fn stop_all(&self) -> Result<Vec<String>> {
           let mut stopped = Vec::new();
           for (name, handle) in &self.actors {
               if handle.send_control(ControlEvent::Stop).await.is_ok() {
                   stopped.push(name.clone());
               }
           }
           Ok(stopped)
       }

       /// Restart all members
       pub async fn restart_all(&self) -> Result<Vec<String>> {
           let mut restarted = Vec::new();
           for (name, handle) in &self.actors {
               if handle.send_control(ControlEvent::Restart).await.is_ok() {
                   restarted.push(name.clone());
               }
           }
           Ok(restarted)
       }
   }
   ```

2. **Update Bulk Operation Handlers**
   ```rust
   /// Start all nodes
   pub async fn start_all(State(state): State<Arc<WebState>>) -> Response {
       let coordinator = state.coordinator.lock().await;
       match coordinator.start_all().await {
           Ok(started) => {
               info!("[Web UI] Started {} nodes", started.len());
               (StatusCode::ACCEPTED, format!("Started {} nodes", started.len()))
                   .into_response()
           }
           Err(e) => {
               warn!("[Web UI] Failed to start all: {}", e);
               (StatusCode::INTERNAL_SERVER_ERROR, format!("Failed: {}", e))
                   .into_response()
           }
       }
   }

   // Similar for stop_all, restart_all
   ```

3. **Add ControlEvent Support in Actors**
   - Ensure all actors handle Start/Stop/Restart control events
   - Test state transitions for each control event
   - Handle edge cases (e.g., starting already-running node)

**Deliverables:**
- Bulk operations functional
- All actors respond to control events
- Web UI bulk buttons work

**Testing:**
- Test Start All with 10+ nodes
- Test Stop All during execution
- Test Restart All with containers and composables
- Verify no race conditions

---

## Success Criteria

### Functional Requirements
- ✅ Web UI control commands (Start/Stop/Restart) work correctly
- ✅ Bulk operations (Start All/Stop All/Restart All) functional
- ✅ Node list and details queries work
- ✅ SSE state updates stream correctly
- ✅ Log streaming continues to work (unchanged)
- ✅ All existing web UI features preserved

### Performance Requirements
- ✅ Control latency ≤50ms (direct vs bridged: ~10x faster)
- ✅ SSE latency ≤10ms (direct vs translated: ~5x faster)
- ✅ Memory reduction: No Registry duplication (~5-10% reduction)

### Quality Requirements
- ✅ All tests pass
- ✅ No compiler warnings
- ✅ event_driven module deleted
- ✅ Documentation updated
- ✅ Clean architecture with single event system

---

## Risks and Mitigation

### Risk 1: Breaking Web UI Functionality
**Impact**: High
**Probability**: Medium
**Mitigation**:
- Test each handler change immediately
- Keep old code in git for rollback
- Incremental migration (phase by phase)

### Risk 2: SSE Connection Issues
**Impact**: Medium
**Probability**: Low
**Mitigation**:
- Test with multiple concurrent connections
- Add connection timeout handling
- Monitor for memory leaks in broadcaster

### Risk 3: Race Conditions in Bulk Operations
**Impact**: Medium
**Probability**: Medium
**Mitigation**:
- Use operation guards for individual nodes
- Test with rapid start/stop cycles
- Add timeout protection

---

## Dependencies

### Internal
- Phase 10 (Actor Pattern) must be complete
- MemberCoordinator must be stable

### External
- No new external dependencies
- Uses existing tokio/axum infrastructure

---

## Alternatives Considered

### Alternative 1: Keep Bridging Layer
**Pros**: Less risky, smaller change
**Cons**: Inefficient, maintains complexity
**Decision**: Rejected - defeats purpose of actor migration

### Alternative 2: Hybrid (Direct Control, Keep Registry for Queries)
**Pros**: Gradual migration, less risky
**Cons**: Still duplicates state, partial solution
**Decision**: Considered for migration strategy but not end goal

### Alternative 3: WebSocket Instead of SSE
**Pros**: Bidirectional, modern
**Cons**: More complex, SSE works fine for one-way updates
**Decision**: Rejected - SSE sufficient for current needs

---

## Metrics

### Before (Bridged)
- **Control latency**: MemberEvent publish → (no handler) → ∞
- **State update latency**: StateEvent → MemberEvent → SSE ≈ 2-5ms
- **Memory overhead**: Registry + EventBus ≈ 5-10% of actor state
- **Code complexity**: 2 event systems, bridging layer, duplication

### After (Direct)
- **Control latency**: ControlEvent send → Actor ≈ 0.5ms
- **State update latency**: StateEvent → SSE ≈ 0.5ms
- **Memory overhead**: None (single source of truth)
- **Code complexity**: 1 event system, no bridging, clean

### Measurement Plan
- Benchmark control command latency (API → actor execution)
- Profile memory usage (before/after Registry removal)
- Count lines of code removed (bridging + event_driven module)

---

## Timeline

```
Week 1:    Phase 11.1 - WebState Refactoring
Week 1-2:  Phase 11.2 - Handler Control Path
Week 2:    Phase 11.3 - SSE Direct Streaming
Week 2-3:  Phase 11.4 - Remove event_driven Module
Week 3:    Phase 11.5 - Bulk Operations
```

**Total Duration**: 2-3 weeks

---

## Next Steps

1. Review this plan
2. Begin Phase 11.1: WebState Refactoring
3. Add query methods to MemberCoordinator
4. Update web/mod.rs constructor
5. Test with simple query operations

---

**Status**: ⏳ Planned
**Last Updated**: 2026-01-01
**Author**: Development Team
