# Container Observability Design

Analysis of the current container control limitations and design options for
reliable composable node state tracking.

## Current Architecture

play_launch manages composable nodes by calling services on the
`component_container` process. The container exposes exactly three services:

```
/_container/load_node     composition_interfaces/srv/LoadNode
/_container/unload_node   composition_interfaces/srv/UnloadNode
/_container/list_nodes    composition_interfaces/srv/ListNodes
```

No topics. No lifecycle interfaces. No event notifications.

### LoadNode Flow (Current)

```
play_launch                           component_container
    │                                        │
    │  LoadNode Request                      │
    │──────────────────────────────────────►  │
    │  (package, plugin, name, namespace,    │
    │   params, remaps, extra_args)          │
    │                                        │
    │  ... waits indefinitely ...            │
    │                                        │
    │  LoadNode Response                     │
    │  ◄──────────────────────────────────── │
    │  (success, error_message,              │
    │   full_node_name, unique_id)           │
    │                                        │
```

### Where It Breaks

**1. No timeout on service call**

`container_actor.rs:482` -- the `Promise::await()` has no timeout wrapper:

```rust
let response_future = client.call(&ros_request)?;
match response_future.await {   // blocks forever if container doesn't respond
    Ok(response) => { ... }
    Err(e) => { ... }
}
```

The container may receive the request, begin loading the plugin (which can take
seconds for heavy nodes like behavior_path_planner with 800+ parameters), and
not respond until loading completes. If the plugin constructor blocks or is slow,
the response is delayed indefinitely.

**2. ListNodes fallback is one-shot and delayed**

When LoadNode hangs for >30s, `check_loading_timeouts()` fires (polled every
5s). It sends a single `ListNodesRequested` event with a `list_nodes_requested`
flag that prevents retries:

```rust
// container_actor.rs:1022-1032
if elapsed >= timeout && !entry.list_nodes_requested {
    entry.list_nodes_requested = true;   // never retries
    // send ListNodesRequested
}
```

If this single ListNodes query fails (service timeout, rate-limited, network
error), the node stays in `Loading` state indefinitely. There is no periodic
retry.

**3. ListNodes matching is name-based**

`handle_node_discovered()` in `coordinator.rs:1086-1130` matches by constructing
the expected full node name from metadata:

```rust
let expected_full_name = format!("{}/{}", namespace, node_name);
if expected_full_name == full_node_name { ... }
```

If the container remaps the node name, or the namespace doesn't match exactly
(leading slash differences, empty vs `/`), the match fails silently.

**4. DiscoveredLoaded matches first Loading node, ignoring unique_id**

`handle_discovered_loaded()` in `container_actor.rs:958-966` matches the **first
composable node in Loading state**, regardless of which node the unique_id
actually belongs to:

```rust
for (name, entry) in self.composable_nodes.iter_mut() {
    if let ComposableState::Loading { started_at } = &entry.state {
        entry.state = ComposableState::Loaded { unique_id };
        // ...
        break;  // takes the first Loading node, not necessarily the right one
    }
}
```

If two composable nodes are loading simultaneously in the same container, the
wrong one may be marked as Loaded.

**5. 200ms warmup delay is a heuristic**

`container_actor.rs:453-463` adds a hardcoded 200ms sleep after
`service_is_ready()` returns true, because the service can appear in the DDS
graph before the container's executor is actually spinning:

```rust
debug!("Waiting 200ms for container executor to start processing requests");
tokio::time::sleep(Duration::from_millis(200)).await;
```

On slow or loaded systems, 200ms may not be enough. On fast systems, it's
wasted time.

### Failure Mode Summary

| Scenario                                | Current Behavior                        | Consequence                     |
|-----------------------------------------|-----------------------------------------|---------------------------------|
| LoadNode hangs (slow plugin)            | Waits indefinitely; ListNodes after 30s | 30s delay before state resolves |
| LoadNode hangs + ListNodes fails        | Node stuck in Loading forever           | No recovery path                |
| Two nodes loading simultaneously        | First Loading node gets the unique_id   | Wrong node marked as Loaded     |
| Container remaps node name              | ListNodes name match fails              | Node stays Loading forever      |
| Service ready but executor not spinning | 200ms heuristic delay                   | May still hang on slow systems  |

---

## What the Container Does NOT Provide

The `component_container` is a minimal service host. It lacks:

1. **No event topic** -- no way to subscribe to load/unload events
2. **No lifecycle integration** -- no `ChangeState` service or state transitions
3. **No callback mechanism** -- no way to register for notifications
4. **No health/status topic** -- no periodic heartbeat or status publication
5. **No per-node state query** -- ListNodes returns all-or-nothing

The container's `ComponentManager` is a C++ class in rclcpp_components. Its
service callbacks (`on_load_node`, `on_unload_node`, `on_list_nodes`) are
`virtual` and can be overridden in a subclass, but the standard executables
(`component_container`, `component_container_mt`, `component_container_isolated`)
use the base class directly.

---

## Design Options

### Option A: Add Timeout + Retry to LoadNode Call

The simplest fix: wrap the existing `Promise::await()` with
`tokio::time::timeout()` and add retry logic.

**Changes to `container_actor.rs`:**

```rust
// Replace: match response_future.await { ... }
// With:
let call_timeout = Duration::from_secs(self.config.load_node_call_timeout_secs);
match tokio::time::timeout(call_timeout, response_future).await {
    Ok(Ok(response)) => { /* handle response */ }
    Ok(Err(e)) => { /* service error */ }
    Err(_) => {
        // Timeout: service didn't respond
        // Don't mark as Failed -- node may actually be loading
        // Trigger immediate ListNodes verification
        warn!("LoadNode call timed out after {}s, verifying via ListNodes",
              call_timeout.as_secs());
        self.request_list_nodes_verification(&composable_name).await;
    }
}
```

**Changes to timeout retry logic:**

Replace one-shot `list_nodes_requested` flag with a retry counter:

```rust
struct ComposableNodeEntry {
    // ...
    list_nodes_attempts: u32,       // replaces list_nodes_requested: bool
    last_list_nodes_attempt: Option<Instant>,
}

fn check_loading_timeouts(&mut self) {
    for (name, entry) in self.composable_nodes.iter_mut() {
        if let ComposableState::Loading { started_at } = &entry.state {
            let elapsed = now.duration_since(*started_at);
            let can_retry = entry.last_list_nodes_attempt
                .map_or(true, |t| t.elapsed() >= Duration::from_secs(10));

            if elapsed >= timeout && can_retry && entry.list_nodes_attempts < 5 {
                entry.list_nodes_attempts += 1;
                entry.last_list_nodes_attempt = Some(now);
                // send ListNodesRequested
            }
        }
    }
}
```

**Pros:**
- Minimal code change (container_actor.rs only)
- No new packages or infrastructure
- No changes to the container process

**Cons:**
- Still polling-based (ListNodes) for the fallback path
- Still name-based matching
- Doesn't fix the "wrong node matched" bug for concurrent loads
- Doesn't address the executor warmup race

**Fixes:**
- Scenario 1 (LoadNode hangs): timeout + immediate ListNodes
- Scenario 2 (ListNodes fails): retry up to 5 times
- Does NOT fix scenarios 3, 4, 5

---

### Option B: DDS Graph Change Events

Use rclrs graph change notifications to detect when nodes appear in or
disappear from the container's process. This is event-driven, not polling.

**How it works:**

The DDS middleware notifies participants when the graph changes (new nodes,
removed nodes, new topics, etc.). rclpy exposes this via
`Node.get_node_names_and_namespaces()` (which play_launch can already call via
rclrs) and guard conditions that fire on graph changes.

```
play_launch                     DDS middleware
    │                                │
    │  Subscribe to graph events     │
    │◄───────────────────────────────│
    │                                │
    │  [LoadNode call sent]          │
    │                                │
    │  Graph change: new node        │
    │  /perception/filter appeared   │
    │◄───────────────────────────────│
    │                                │
    │  → Match to pending loads      │
    │  → Mark as Loaded              │
```

**Implementation:**

rclrs provides `Node::get_node_names_and_namespaces()` for querying the current
graph, and graph guard conditions can be polled via the executor. The approach:

1. After sending LoadNode, start watching for graph changes
2. On graph change: call `get_node_names_and_namespaces()` and diff against
   previous snapshot
3. Match new nodes to pending loads by (namespace, node_name)
4. If a match is found, mark the composable node as Loaded

```rust
async fn watch_graph_for_node(
    ros_node: Arc<rclrs::Node>,
    expected_namespace: &str,
    expected_name: &str,
    timeout: Duration,
) -> Option<()> {
    let deadline = Instant::now() + timeout;
    let mut known_nodes: HashSet<(String, String)> = HashSet::new();

    // Snapshot current graph
    if let Ok(nodes) = ros_node.get_node_names_and_namespaces() {
        for (name, ns) in &nodes {
            known_nodes.insert((name.clone(), ns.clone()));
        }
    }

    loop {
        tokio::time::sleep(Duration::from_millis(500)).await;
        if Instant::now() > deadline { return None; }

        if let Ok(nodes) = ros_node.get_node_names_and_namespaces() {
            for (name, ns) in &nodes {
                if !known_nodes.contains(&(name.clone(), ns.clone())) {
                    // New node appeared
                    if name == expected_name && ns == expected_namespace {
                        return Some(());
                    }
                    known_nodes.insert((name.clone(), ns.clone()));
                }
            }
        }
    }
}
```

**Pros:**
- Event-driven detection (500ms polling of graph, not 5s polling of ListNodes)
- Doesn't depend on ListNodes service at all
- Works even if LoadNode response is lost
- Detects the actual node appearance in DDS, not just container's internal state

**Cons:**
- Still polling-based (graph query every 500ms), just at a different layer
- Graph query returns ALL nodes, not just container members -- O(total_nodes)
- Same DDS discovery lag problem: node may not appear in graph immediately
- Cannot distinguish between a composable node loaded by play_launch vs one
  loaded by another tool
- No unique_id -- must match by name only
- rclrs graph guard condition API may not be fully exposed

**Fixes:**
- Scenario 1 (LoadNode hangs): detects node appearance via graph
- Scenario 2 (ListNodes fails): doesn't use ListNodes at all
- Does NOT fix scenario 3 (concurrent loads -- still name-based)
- Partially fixes scenario 4 (uses DDS graph names, not container-reported names)
- Does NOT fix scenario 5 (executor warmup)

---

### Option C: Custom Container with Event Topic

Fork or wrap `component_container` to publish a topic whenever a node is loaded
or unloaded. play_launch subscribes to this topic for real-time, event-driven
state tracking.

**Container publishes:**

```
Topic: /<container_name>/component_events
Type:  play_launch_msgs/msg/ComponentEvent  (new message type)
QoS:   reliable, transient_local, keep_last(100)
```

**`msg/ComponentEvent.msg`:**

```
uint8 LOADED=0
uint8 UNLOADED=1
uint8 LOAD_FAILED=2

builtin_interfaces/Time stamp
uint8 event_type
uint64 unique_id               # From LoadNode response
string full_node_name          # Fully qualified name
string package_name            # Plugin package
string plugin_name             # Plugin class
string error_message           # Non-empty when event_type=LOAD_FAILED
```

**Custom container implementation approach:**

Subclass `ComponentManager` and override the service callbacks to publish events:

```cpp
class ObservableComponentManager : public rclcpp_components::ComponentManager {
public:
    ObservableComponentManager(
        std::weak_ptr<rclcpp::Executor> executor,
        const rclcpp::NodeOptions & options)
    : ComponentManager(executor, "component_container", options)
    {
        event_pub_ = create_publisher<play_launch_msgs::msg::ComponentEvent>(
            "~/_container/component_events",
            rclcpp::QoS(100).reliable().transient_local());
    }

protected:
    void on_load_node(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<LoadNode::Request> request,
        std::shared_ptr<LoadNode::Response> response) override
    {
        // Call parent implementation
        ComponentManager::on_load_node(request_header, request, response);

        // Publish event
        auto event = play_launch_msgs::msg::ComponentEvent();
        event.stamp = now();
        event.unique_id = response->unique_id;
        event.full_node_name = response->full_node_name;
        event.package_name = request->package_name;
        event.plugin_name = request->plugin_name;

        if (response->success) {
            event.event_type = ComponentEvent::LOADED;
        } else {
            event.event_type = ComponentEvent::LOAD_FAILED;
            event.error_message = response->error_message;
        }
        event_pub_->publish(event);
    }

    void on_unload_node(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<UnloadNode::Request> request,
        std::shared_ptr<UnloadNode::Response> response) override
    {
        // Capture node name before unload (it won't exist after)
        std::string full_name;
        // ... lookup name from unique_id in node_wrappers_ ...

        ComponentManager::on_unload_node(request_header, request, response);

        if (response->success) {
            auto event = play_launch_msgs::msg::ComponentEvent();
            event.stamp = now();
            event.event_type = ComponentEvent::UNLOADED;
            event.unique_id = request->unique_id;
            event.full_node_name = full_name;
            event_pub_->publish(event);
        }
    }

private:
    rclcpp::Publisher<play_launch_msgs::msg::ComponentEvent>::SharedPtr event_pub_;
};
```

**play_launch subscriber side:**

```rust
// Subscribe to each container's event topic
let sub = ros_node.create_subscription::<play_launch_msgs::msg::ComponentEvent>(
    &format!("{}/_container/component_events", container_name),
    qos,
    move |event: play_launch_msgs::msg::ComponentEvent| {
        match event.event_type {
            ComponentEvent::LOADED => {
                // Match by unique_id (authoritative, not name-based)
                state_tx.send(StateEvent::ComponentLoaded {
                    container_name: container_name.clone(),
                    unique_id: event.unique_id,
                    full_node_name: event.full_node_name,
                });
            }
            ComponentEvent::LOAD_FAILED => { ... }
            ComponentEvent::UNLOADED => { ... }
        }
    },
)?;
```

**Package structure:**

```
src/play_launch_container/
├── package.xml          (depends on rclcpp_components, play_launch_msgs)
├── CMakeLists.txt
├── src/
│   ├── observable_component_manager.hpp
│   ├── observable_component_manager.cpp
│   ├── component_container.cpp           (main, uses ObservableComponentManager)
│   └── component_container_isolated.cpp  (isolated variant)
```

play_launch would use this custom container executable instead of the stock
`rclcpp_components/component_container`. The record.json already specifies the
container executable, so this is a configuration change:

```json
{
  "executable": "component_container",
  "package": "play_launch_container"
}
```

**Pros:**
- **Event-driven**: no polling, immediate notification
- **Authoritative**: event comes from the container itself with the real
  unique_id and full_node_name
- **Fixes all scenarios**: unique_id matching (not name-based), no timeout
  guessing, no executor warmup race
- **Transient local QoS**: late subscribers (play_launch restarts) get event
  history
- **Unload tracking**: also detects unloads (currently not tracked reliably)
- **Standard ROS 2 mechanism**: topic subscription, no special DDS tricks

**Cons:**
- **Requires a custom container binary** -- users must use
  `play_launch_container` instead of `rclcpp_components/component_container`
- **New C++ package** to maintain (CMake, rclcpp_components dependency)
- **Forward compatibility**: must track upstream `ComponentManager` API changes
  across ROS distributions
- **Deployment**: must be installed alongside play_launch

**Fixes:**
- Scenario 1 (LoadNode hangs): event published by container after load completes
- Scenario 2 (ListNodes fails): doesn't use ListNodes at all
- Scenario 3 (concurrent loads): matches by unique_id, not by name
- Scenario 4 (name mismatch): uses container-reported full_node_name
- Scenario 5 (executor warmup): event only fires after executor processes the
  load -- proof that executor is spinning

---

### Option D: Combine A + C (Layered Approach)

Use Option A (timeout + retry) as the baseline for all containers, with Option C
(custom container with events) as an opt-in upgrade for play_launch-managed
containers.

**Rationale:** Option A is a low-risk improvement that works with any container
(including third-party ones). Option C is the proper fix but requires users to
switch container executables.

**Implementation order:**

1. **Phase 1: Option A** -- add timeout to LoadNode call, add ListNodes retry
   logic. Works immediately with stock containers.
2. **Phase 2: Option C** -- build `play_launch_container` package. When detected,
   play_launch subscribes to event topics and disables ListNodes polling for that
   container.
3. **Fallback**: If a container doesn't publish events (stock container or
   third-party), fall back to Phase 1 behavior (timeout + ListNodes retry).

**Detection logic:**

```rust
// After container starts, check if it publishes the event topic
let has_events = ros_node
    .get_topic_names_and_types()
    .iter()
    .any(|(name, types)| {
        name == &format!("{}/_container/component_events", container_name)
            && types.contains(&"play_launch_msgs/msg/ComponentEvent".to_string())
    });

if has_events {
    // Subscribe to event topic (reliable, immediate)
    subscribe_to_container_events(container_name);
} else {
    // Fall back to LoadNode timeout + ListNodes retry
    use_polling_verification(container_name);
}
```

**Pros:**
- Progressive enhancement: immediate improvement (Phase 1), full fix later
  (Phase 2)
- Backward compatible: stock containers still work
- Clean separation: event-driven path and polling path coexist

**Cons:**
- Two code paths to maintain
- Phase 2 still requires a custom container binary

---

## Comparison

| | Option A | Option B | Option C | Option D |
|---|---------|---------|---------|---------|
| **Approach** | Timeout + retry | DDS graph events | Custom container | A + C layered |
| **New packages** | None | None | C++ container | C++ container |
| **Detection** | Polling (ListNodes) | Polling (graph) | Event-driven (topic) | Both |
| **Accuracy** | Name-based | Name-based | unique_id-based | Both |
| **Concurrent loads** | Broken | Broken | Correct | Correct (C path) |
| **Stock containers** | Yes | Yes | No | Yes (A fallback) |
| **Effort** | Low | Medium | High | Medium + High |
| **LoadNode hang** | Timeout → retry | Timeout → graph watch | Event on completion | Both |
| **Unload tracking** | No | Partial (graph diff) | Yes (event) | Yes (C path) |

---

## Recommendation

**Option D (layered)** provides the best balance:

- **Phase 1** (Option A) is a focused fix to `container_actor.rs` that addresses
  the two most common failures (indefinite hang, no retry) with zero new
  infrastructure. Ship this first.

- **Phase 2** (Option C) is the correct long-term architecture. The custom
  container eliminates all polling, fixes the concurrent-load bug, and adds
  unload tracking. It requires a new C++ package but the code is small (~200
  lines) since it delegates to the base `ComponentManager`.

The detection logic in Option D means users who install `play_launch_container`
get the full event-driven experience, while users with stock containers still
get improved timeout/retry behavior.

---

## Source Code Analysis: ComponentManager Internals

Based on study of rclcpp_components source (humble branch, cloned to
`external/rclcpp/`).

### Protected Members (Subclass-Accessible)

From `component_manager.hpp:259-268`:

```cpp
protected:
  std::weak_ptr<rclcpp::Executor> executor_;
  uint64_t unique_id_ {1};
  std::map<std::string, std::unique_ptr<class_loader::ClassLoader>> loaders_;
  std::map<uint64_t, rclcpp_components::NodeInstanceWrapper> node_wrappers_;

  rclcpp::Service<LoadNode>::SharedPtr loadNode_srv_;
  rclcpp::Service<UnloadNode>::SharedPtr unloadNode_srv_;
  rclcpp::Service<ListNodes>::SharedPtr listNodes_srv_;
```

All state needed for observability is protected, not private. A subclass can:
- Read `node_wrappers_` to enumerate loaded components
- Read `unique_id_` to know the next assigned ID
- Access `executor_` to manage the executor relationship

### Virtual Service Callbacks

All three service callbacks are `virtual protected`:

```cpp
virtual void on_load_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response);

virtual void on_unload_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response);

virtual void on_list_nodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response);
```

Also virtual: `add_node_to_executor(uint64_t)`, `remove_node_from_executor(uint64_t)`,
`create_node_options(Request)`, `create_component_factory(ComponentResource)`,
`get_component_resources(package, index)`, `set_executor(weak_ptr)`.

### Constructor (Service Binding)

From `component_manager.cpp:33-64`:

```cpp
ComponentManager::ComponentManager(
  std::weak_ptr<rclcpp::Executor> executor,
  std::string node_name,
  const rclcpp::NodeOptions & node_options)
: Node(std::move(node_name), node_options),
  executor_(executor)
{
  rmw_qos_profile_t service_qos = rmw_qos_profile_services_default;
  service_qos.depth = 200;
  loadNode_srv_ = create_service<LoadNode>(
    "~/_container/load_node",
    std::bind(&ComponentManager::on_load_node, this, _1, _2, _3),
    service_qos);
  // ... similarly for unload and list
}
```

Key: `std::bind(&ComponentManager::on_load_node, this, ...)` binds to the virtual
method. When a subclass overrides `on_load_node()`, service calls dispatch to the
override. No need to re-register services.

### on_load_node() Internals

From `component_manager.cpp:220-286`:

1. Calls `get_component_resources(request->package_name)` to find plugin
2. Iterates resources looking for `request->plugin_name`
3. Creates factory via `create_component_factory(resource)`
4. Calls `create_node_options(request)` to build NodeOptions
5. Assigns `node_id = unique_id_++`
6. Stores: `node_wrappers_[node_id] = factory->create_node_instance(options)`
7. Calls `add_node_to_executor(node_id)`
8. Sets `response->full_node_name`, `response->unique_id`, `response->success`
9. Returns

The entire operation is synchronous. The response fields are set before return,
so a subclass calling `ComponentManager::on_load_node()` can read the response
immediately after.

### on_unload_node() Internals

From `component_manager.cpp:288-309`:

```cpp
auto wrapper = node_wrappers_.find(request->unique_id);
if (wrapper == node_wrappers_.end()) {
    response->success = false;
    // error message...
} else {
    remove_node_from_executor(request->unique_id);
    node_wrappers_.erase(wrapper);
    response->success = true;
}
```

Critical: after `erase()`, the node is gone from `node_wrappers_`. A subclass
that needs the node's name for the unload event must capture it **before** calling
the parent's `on_unload_node()`.

### Proven Subclass Pattern: ComponentManagerIsolated

From `component_manager_isolated.hpp`:

```cpp
template<typename ExecutorT = rclcpp::executors::SingleThreadedExecutor>
class ComponentManagerIsolated : public rclcpp_components::ComponentManager {
  using rclcpp_components::ComponentManager::ComponentManager;  // delegating ctor

protected:
  void add_node_to_executor(uint64_t node_id) override {
    // Creates dedicated executor per node
    auto exec = std::make_shared<ExecutorT>();
    exec->add_node(node_wrappers_[node_id].get_node_base_interface());
    // ...
  }

  void remove_node_from_executor(uint64_t node_id) override {
    // Cancels and joins the dedicated thread
    // ...
  }
};
```

This confirms:
1. Delegating constructor works (`using ComponentManager::ComponentManager`)
2. Protected `node_wrappers_[node_id]` is directly accessible from subclass
3. Virtual `add_node_to_executor` / `remove_node_from_executor` can be overridden

### Container Executables

From `component_container.cpp` and `component_container_mt.cpp`:

```cpp
// Single-threaded (component_container.cpp)
auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
exec->add_node(node);
exec->spin();

// Multi-threaded (component_container_mt.cpp)
auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
auto node = std::make_shared<rclcpp_components::ComponentManager>();
// reads thread_num parameter, recreates executor if needed
node->set_executor(exec);
exec->add_node(node);
exec->spin();
```

Drop-in replacement: substitute `ComponentManager` → `ObservableComponentManager`.

### CMake Pattern

From `rclcpp_components/CMakeLists.txt`:

```cmake
# Shared library
add_library(component_manager SHARED src/component_manager.cpp)
target_link_libraries(...)

# Executables link against the shared library
add_executable(component_container src/component_container.cpp)
target_link_libraries(component_container component_manager)
```

Our custom package links against the upstream `component_manager` library (via
`ament_target_dependencies`) rather than copying source.

---

## Concrete Implementation: ObservableComponentManager

Based on the source analysis, here is the refined implementation.

### Package: play_launch_container

```
src/play_launch_container/
├── package.xml
├── CMakeLists.txt
├── include/play_launch_container/
│   └── observable_component_manager.hpp
├── src/
│   ├── observable_component_manager.cpp
│   ├── component_container.cpp          # ST main
│   └── component_container_mt.cpp       # MT main
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.14)
project(play_launch_container)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(play_launch_msgs REQUIRED)

# Shared library
add_library(observable_component_manager SHARED
  src/observable_component_manager.cpp)
target_include_directories(observable_component_manager PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(observable_component_manager
  rclcpp rclcpp_components composition_interfaces play_launch_msgs)

# Single-threaded container
add_executable(component_container src/component_container.cpp)
target_link_libraries(component_container observable_component_manager)
ament_target_dependencies(component_container rclcpp)

# Multi-threaded container
add_executable(component_container_mt src/component_container_mt.cpp)
target_link_libraries(component_container_mt observable_component_manager)
ament_target_dependencies(component_container_mt rclcpp)

install(TARGETS observable_component_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS component_container component_container_mt
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
```

### observable_component_manager.hpp

```cpp
#ifndef PLAY_LAUNCH_CONTAINER__OBSERVABLE_COMPONENT_MANAGER_HPP_
#define PLAY_LAUNCH_CONTAINER__OBSERVABLE_COMPONENT_MANAGER_HPP_

#include "rclcpp_components/component_manager.hpp"
#include "play_launch_msgs/msg/component_event.hpp"

namespace play_launch_container
{

class ObservableComponentManager : public rclcpp_components::ComponentManager
{
public:
  ObservableComponentManager(
    std::weak_ptr<rclcpp::Executor> executor =
      std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
    std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()
      .start_parameter_services(false)
      .start_parameter_event_publisher(false));

  ~ObservableComponentManager() override = default;

protected:
  void on_load_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response) override;

  void on_unload_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response) override;

private:
  rclcpp::Publisher<play_launch_msgs::msg::ComponentEvent>::SharedPtr event_pub_;
};

}  // namespace play_launch_container

#endif  // PLAY_LAUNCH_CONTAINER__OBSERVABLE_COMPONENT_MANAGER_HPP_
```

### observable_component_manager.cpp

```cpp
#include "play_launch_container/observable_component_manager.hpp"

namespace play_launch_container
{

ObservableComponentManager::ObservableComponentManager(
  std::weak_ptr<rclcpp::Executor> executor,
  std::string node_name,
  const rclcpp::NodeOptions & node_options)
: ComponentManager(executor, std::move(node_name), node_options)
{
  // Transient local: late subscribers (play_launch restarts) get history
  auto qos = rclcpp::QoS(100).reliable().transient_local();
  event_pub_ = create_publisher<play_launch_msgs::msg::ComponentEvent>(
    "~/_container/component_events", qos);
}

void ObservableComponentManager::on_load_node(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<LoadNode::Request> request,
  std::shared_ptr<LoadNode::Response> response)
{
  // Parent does the actual loading (synchronous, response populated on return)
  ComponentManager::on_load_node(request_header, request, response);

  // Publish event with the result
  auto event = play_launch_msgs::msg::ComponentEvent();
  event.stamp = now();
  event.package_name = request->package_name;
  event.plugin_name = request->plugin_name;

  if (response->success) {
    event.event_type = play_launch_msgs::msg::ComponentEvent::LOADED;
    event.unique_id = response->unique_id;
    event.full_node_name = response->full_node_name;
  } else {
    event.event_type = play_launch_msgs::msg::ComponentEvent::LOAD_FAILED;
    event.error_message = response->error_message;
  }
  event_pub_->publish(event);
}

void ObservableComponentManager::on_unload_node(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<UnloadNode::Request> request,
  std::shared_ptr<UnloadNode::Response> response)
{
  // Capture node name BEFORE parent erases it from node_wrappers_
  std::string full_name;
  auto it = node_wrappers_.find(request->unique_id);
  if (it != node_wrappers_.end()) {
    full_name = it->second.get_node_base_interface()->get_fully_qualified_name();
  }

  // Parent does the actual unloading (erases from node_wrappers_)
  ComponentManager::on_unload_node(request_header, request, response);

  if (response->success) {
    auto event = play_launch_msgs::msg::ComponentEvent();
    event.stamp = now();
    event.event_type = play_launch_msgs::msg::ComponentEvent::UNLOADED;
    event.unique_id = request->unique_id;
    event.full_node_name = full_name;
    event_pub_->publish(event);
  }
}

}  // namespace play_launch_container
```

### component_container.cpp (Single-Threaded)

```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "play_launch_container/observable_component_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<play_launch_container::ObservableComponentManager>(exec);
  exec->add_node(node);
  exec->spin();
}
```

### component_container_mt.cpp (Multi-Threaded)

```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "play_launch_container/observable_component_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<play_launch_container::ObservableComponentManager>();
  if (node->has_parameter("thread_num")) {
    const auto thread_num = node->get_parameter("thread_num").as_int();
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, thread_num);
    node->set_executor(exec);
  } else {
    node->set_executor(exec);
  }
  exec->add_node(node);
  exec->spin();
}
```

### Compatibility Notes

1. **Service-level compatible**: The custom container exposes the same three
   services (`load_node`, `unload_node`, `list_nodes`) at the same paths with
   identical behavior. Existing tools (`ros2 component load`, `ros2 component list`)
   work unchanged.

2. **Wire-compatible**: The `composition_interfaces/srv/LoadNode` request/response
   types are unchanged. Only an additional topic is published.

3. **Name-compatible**: The container node can use the same `--ros-args -r __node:=`
   remapping. The event topic is relative (`~/_container/component_events`), so it
   follows the container's namespace automatically.

4. **Drop-in replacement in record.json**: Only the `package` field changes:
   ```json
   // Before
   {"executable": "component_container", "package": "rclcpp_components"}
   // After
   {"executable": "component_container", "package": "play_launch_container"}
   ```
   play_launch can do this substitution automatically when the custom container
   package is detected on the system.

5. **Forward compatibility**: The subclass only calls parent virtual methods and
   reads protected members. The API contract has been stable across Foxy → Galactic
   → Humble → Iron → Jazzy. The `ComponentManagerIsolated` in rclcpp_components
   itself uses the same pattern, so upstream is committed to maintaining it.

---

## Files Summary

### Phase 1 (Option A -- timeout + retry)

| File | Change |
|------|--------|
| `src/play_launch/src/member_actor/container_actor.rs` | Add `tokio::time::timeout()` to LoadNode call; replace one-shot flag with retry counter |
| `src/play_launch/src/cli/config.rs` | Add `load_node_call_timeout_secs` setting (default: 60) |

### Phase 2 (Option C -- custom container)

| File | Action |
|------|--------|
| `src/play_launch_msgs/msg/ComponentEvent.msg` | **NEW** -- event message definition |
| `src/play_launch_container/` | **NEW** -- C++ package with ObservableComponentManager |
| `src/play_launch/src/member_actor/container_actor.rs` | Add event topic subscription path |
| `src/play_launch/src/member_actor/container_actor.rs` | Detection logic: event topic → subscribe, else → polling |
