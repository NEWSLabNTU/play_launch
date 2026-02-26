# ROS 2 Daemon Analysis

Analysis of the `ros2cli` daemon architecture and its failure modes under heavy
node startup load (e.g. Autoware with ~61 concurrent processes).

Source: `external/ros2cli/` (humble branch, `ros2cli 0.18.15`).

## Background

When `play_launch` starts Autoware (~61 processes, 46 nodes, 15 containers, 54
composable nodes), users running `ros2 node list` or `ros2 topic list` observe
stale results and intermittent failures. `play_launch` itself does **not** use
the daemon -- it uses `rclrs` (Rust ROS bindings) directly. The daemon issue
affects the **user experience** when running CLI introspection commands alongside
a large deployment.

---

## Architecture Overview

The daemon is a persistent Python process that caches a single rclpy node for
graph introspection. CLI tools connect to it via XML-RPC instead of creating an
ephemeral rclpy node for each invocation (which costs ~500ms of DDS discovery).

```
┌──────────────┐    XML-RPC     ┌─────────────────────────────────────────────┐
│ ros2 node    │  ──────────►   │  ros2cli daemon (Python process)            │
│     list     │  localhost:    │                                             │
│              │  11511+DID     │  LocalXMLRPCServer                          │
└──────────────┘                │    └─ verify_request() → netifaces          │
                                │    └─ handle_request() [sequential]         │
                                │                                             │
                                │  NetworkAwareNode                           │
                                │    └─ reset_if_addresses_changed()          │
                                │         → netifaces on EVERY method call    │
                                │    └─ DirectNode                            │
                                │         └─ rclpy node (spins 0.5s at init)  │
                                │         └─ get_node_names_and_namespaces()  │
                                │         └─ get_topic_names_and_types()      │
                                │         └─ ... (17 graph functions)         │
                                └─────────────────────────────────────────────┘
```

### Key Components

| File | Role |
|------|------|
| `ros2cli/daemon/__init__.py` | Server entry point, `serve()` loop, function registration |
| `ros2cli/daemon/daemonize.py` | Fork + pickle-based daemonization |
| `ros2cli/node/direct.py` | `DirectNode` -- rclpy node with 0.5s init spin |
| `ros2cli/node/network_aware.py` | `NetworkAwareNode` -- wraps DirectNode, checks netifaces |
| `ros2cli/node/daemon.py` | `DaemonNode` -- XML-RPC client proxy |
| `ros2cli/node/strategy.py` | `NodeStrategy` -- chooses daemon vs direct node |
| `ros2cli/xmlrpc/local_server.py` | `LocalXMLRPCServer` -- SO_LINGER, IP verification |
| `ros2cli/helpers.py` | `pretty_print_call`, `before_invocation` wrappers |

### Request Flow

1. `ros2 node list` creates a `NodeStrategy`
2. `NodeStrategy.__init__` checks `is_daemon_running()` (XML-RPC ping via
   `system.listMethods()`)
3. If running: creates `DaemonNode` (XML-RPC proxy)
4. If not: calls `spawn_daemon()`, falls back to ephemeral `DirectNode`
5. `NodeStrategy.__getattr__` routes method calls to daemon or direct node
6. Daemon processes request sequentially via `server.handle_request()`

### Spawning (TOCTOU-safe)

The daemon uses socket binding as an atomic lock (`daemon.py:104-176`):

1. Parent binds the XML-RPC server socket (if `EADDRINUSE`, daemon is already
   running)
2. Socket is marked inheritable and pickled to the child process via stdin
3. Child unpickles the callable (which holds the server), closes stdin (signals
   readiness)
4. Parent detects stdin closure, closes its copy of the server socket

---

## Root Causes of Failure Under Load

### 1. `netifaces` Called on Every Method Invocation

**File:** `ros2cli/node/network_aware.py:49-61`

```python
def __getattr__(self, name):
    attr = getattr(self.node, name)
    if inspect.ismethod(attr):
        @functools.wraps(attr)
        def wrapper(*args, **kwargs):
            self.reset_if_addresses_changed()          # ← every call
            return getattr(self.node, name)(*args, **kwargs)
        wrapper.__signature__ = inspect.signature(attr)
        return wrapper
    self.reset_if_addresses_changed()
    return attr
```

`reset_if_addresses_changed()` calls `get_interfaces_ip_addresses()` which
iterates all network interfaces via `netifaces.interfaces()` +
`netifaces.ifaddresses()` -- O(interfaces) system calls per XML-RPC request.

**Additionally**, line 31 unconditionally prints the full result to stdout:

```python
def get_interfaces_ip_addresses():
    addresses_by_interfaces = defaultdict(functools.partial(defaultdict, set))
    for interface_name in netifaces.interfaces():
        for kind, info_list in netifaces.ifaddresses(interface_name).items():
            for info in info_list:
                addresses_by_interfaces[kind][interface_name].add(info['addr'])
    print(f'Addresses by interfaces: {addresses_by_interfaces}')  # ← every call
    return addresses_by_interfaces
```

**Worst case**: If addresses happen to change transiently (Docker, WiFi
roaming), the handler destroys the entire rclpy context and recreates it:

```python
def reset_if_addresses_changed(self):
    new_addresses = get_interfaces_ip_addresses()
    if new_addresses != self.addresses_at_start:
        self.addresses_at_start = new_addresses
        self.node.destroy_node()
        rclpy.shutdown()                       # ← destroys ALL DDS state
        self.node = DirectNode(self.args)      # ← 0.5s discovery from scratch
        self.node.__enter__()
```

**Impact**: Every XML-RPC request pays the cost of interface enumeration + debug
print I/O. A transient address change wipes all accumulated DDS discovery state.

### 2. `verify_request()` Also Calls `netifaces` on Every Connection

**File:** `ros2cli/xmlrpc/local_server.py:52-55`

```python
def verify_request(self, request, client_address):
    if client_address[0] not in get_local_ipaddrs():    # ← netifaces again
        return False
    return super(LocalXMLRPCServer, self).verify_request(request, client_address)
```

This is a **second** layer of netifaces overhead, invoked before the request
even reaches the registered function. Combined with `NetworkAwareNode`, each
request triggers netifaces **twice**.

`SO_LINGER(1, 0)` on both the listening and accepted sockets causes immediate
RST on close instead of graceful TCP teardown. Under rapid concurrent queries
this can produce `ConnectionResetError` on the client side:

```python
def server_bind(self):
    self.socket.setsockopt(
        socket.SOL_SOCKET, socket.SO_LINGER, struct.pack('ii', 1, 0))

def get_request(self):
    sock, addr = super().get_request()
    sock.setsockopt(
        socket.SOL_SOCKET, socket.SO_LINGER, struct.pack('ii', 1, 0))
    return sock, addr
```

### 3. No Graph Result Caching

Every `get_node_names_and_namespaces()` call goes directly to the rclpy C
extension layer, which queries DDS. During a burst of rapid-fire queries while
the graph is churning (dozens of nodes starting/stopping), each query returns a
different snapshot. There is no deduplication, debouncing, or caching.

### 4. Server Blocks for 2 Hours Between Requests

**File:** `ros2cli/daemon/__init__.py:113-126`

```python
server.timeout = timeout    # timeout = 2 * 60 * 60 = 7200 seconds

while not shutdown:
    server.handle_request()  # blocks in select() for up to 7200s
```

`handle_request()` calls `select()` with the 2-hour timeout. The `handle_timeout`
callback only fires after the **full timeout expires** and merely sets
`shutdown = True`. Between requests there is no periodic maintenance -- no cache
cleanup, no graph refresh, no spinning.

### 5. `pretty_print_call` on Every Invocation

**File:** `ros2cli/helpers.py:62-73`

```python
def pretty_print_call(func, *args, **kwargs):
    name = func.__name__
    arguments = ', '.join(
        [f'{v!r}' for v in args] +
        [f'{k}={v!r}' for k, v in kwargs.items()]
    )
    print(f'{name}({arguments})')
```

All 17 registered functions are wrapped with this:

```python
for func in functions:
    server.register_function(
        before_invocation(func, pretty_print_call))
```

Every request formats and prints the full call signature to stdout. When the
daemon is run in non-debug mode, stdout is `/dev/null` so the I/O is discarded,
but the string formatting overhead remains.

### 6. DirectNode Never Spins After Initialization

**File:** `ros2cli/node/direct.py:27-60`

```python
class DirectNode:
    def __init__(self, args, *, node_name=None):
        # ...
        timeout = getattr(args, 'spin_time', DEFAULT_TIMEOUT)  # 0.5s
        timer = self.node.create_timer(timeout, timer_callback)
        while not timeout_reached:
            rclpy.spin_once(self.node)
        self.node.destroy_timer(timer)
        # NEVER SPINS AGAIN
```

The underlying rclpy node spins for exactly 0.5 seconds during initialization,
then never spins again. The daemon's `serve()` loop calls
`server.handle_request()` -- it never calls `rclpy.spin_once()`.

Graph queries go directly to the DDS layer via the C extension (bypassing the
executor), so they do return current data without spinning. However, DDS guard
conditions for graph change events accumulate unprocessed, and the middleware's
internal discovery processing has inherent propagation delays when hundreds of
nodes start/stop simultaneously. The daemon has no mechanism to work around
these delays.

---

## Interaction Effects Under Autoware-Scale Load

When ~61 processes start simultaneously:

1. **Client-side**: Multiple `ros2 node list` invocations queue behind sequential
   `handle_request()`. Each waits for the previous to complete.

2. **Per-request overhead**: Each request triggers:
   - `verify_request()` → `get_local_ipaddrs()` → netifaces enumeration
   - `before_invocation()` → `pretty_print_call()` → string format + print
   - `__getattr__` → `reset_if_addresses_changed()` →
     `get_interfaces_ip_addresses()` → netifaces + print
   - Actual DDS graph query

3. **DDS discovery lag**: With ~61 processes joining the graph within seconds,
   DDS discovery propagation may not have completed by the time the query runs.
   The daemon returns whatever DDS currently reports -- potentially stale.

4. **No retry/refresh mechanism**: If the first query returns a partial graph,
   subsequent queries may also return partial results until DDS catches up.
   There is no "wait for graph to stabilize" logic.

---

## Proposed Fixes

### Fix 1: Throttle `netifaces` in `NetworkAwareNode`

Check interfaces at most once every 30 seconds instead of on every method call.
Remove the debug `print()`.

```python
import time

class NetworkAwareNode:
    _IFACE_CHECK_INTERVAL = 30.0  # seconds

    def __init__(self, args):
        self.args = args
        self.node = DirectNode(args)
        self.addresses_at_start = get_interfaces_ip_addresses()
        self._last_iface_check = time.monotonic()

    def _maybe_reset(self):
        now = time.monotonic()
        if now - self._last_iface_check < self._IFACE_CHECK_INTERVAL:
            return
        self._last_iface_check = now
        self.reset_if_addresses_changed()
```

### Fix 2: Cache Local IPs in `LocalXMLRPCServer`

Cache `get_local_ipaddrs()` with a 30-second TTL:

```python
_local_ipaddrs_cache = None
_local_ipaddrs_time = 0.0
_LOCAL_IPADDRS_TTL = 30.0

def get_local_ipaddrs():
    global _local_ipaddrs_cache, _local_ipaddrs_time
    now = time.monotonic()
    if _local_ipaddrs_cache is not None and (now - _local_ipaddrs_time) < _LOCAL_IPADDRS_TTL:
        return _local_ipaddrs_cache
    # ... existing netifaces iteration ...
    _local_ipaddrs_cache = iplist
    _local_ipaddrs_time = now
    return iplist
```

### Fix 3: Graph Cache + Responsive Serve Loop

Replace the 2-hour blocking `handle_request()` with a 200ms timeout heartbeat.
Add a TTL-based graph cache. Remove `pretty_print_call` wrapper.

```python
def serve(server, *, timeout=2 * 60 * 60):
    # ... node setup ...

    cache = {}          # (func_name, args) → (timestamp, result)
    CACHE_TTL = 0.2     # 200ms

    def cached_call(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            key = (func.__name__, args, tuple(sorted(kwargs.items())))
            now = time.monotonic()
            if key in cache:
                ts, result = cache[key]
                if now - ts < CACHE_TTL:
                    return result
            result = func(*args, **kwargs)
            cache[key] = (now, result)
            return result
        wrapper.__signature__ = inspect.signature(func)
        return wrapper

    # Register WITHOUT pretty_print_call, WITH cache
    for func in functions:
        server.register_function(cached_call(func))

    # Responsive loop
    last_activity = time.monotonic()
    server.timeout = 0.2   # 200ms heartbeat

    def timeout_handler():
        nonlocal shutdown
        if time.monotonic() - last_activity > timeout:
            shutdown = True
    server.handle_timeout = timeout_handler
```

### Fix 4: Deployment via `just patch-daemon` / `just unpatch-daemon`

Justfile recipes to copy patched files over the system installation and restore
originals:

```makefile
patch-daemon:
    sudo cp /opt/ros/$ROS_DISTRO/lib/python3.10/site-packages/ros2cli/daemon/__init__.py{,.bak}
    sudo cp external/ros2cli/ros2cli/ros2cli/daemon/__init__.py \
         /opt/ros/$ROS_DISTRO/lib/python3.10/site-packages/ros2cli/daemon/__init__.py
    # ... same for network_aware.py, local_server.py ...
    ros2 daemon stop && ros2 daemon start

unpatch-daemon:
    sudo cp /opt/ros/$ROS_DISTRO/lib/python3.10/site-packages/ros2cli/daemon/__init__.py{.bak,}
    # ... restore other files ...
    ros2 daemon stop && ros2 daemon start
```

---

## Why Not Threading?

Adding `ThreadingMixIn` to the XML-RPC server was considered but rejected:

- rclpy nodes are not thread-safe by default
- The C extension methods likely hold the GIL during execution, serializing
  calls anyway
- The `DirectNode.__getattr__` proxy provides no mutual exclusion for concurrent
  graph queries
- Caching + throttling addresses the root causes without introducing
  concurrency complexity

---

## Verification Plan

1. `ros2 daemon stop && ros2 daemon start` with patched files
2. Start Autoware via `play_launch` (`just run-sim`)
3. While nodes are starting, run `ros2 node list` rapidly -- verify no failures
4. After stabilization, verify `ros2 node list` returns all expected nodes
5. `ros2 daemon stop` -- verify clean shutdown
6. `just test` passes (no changes to play_launch itself)

---

## Summary

| Root Cause | Impact | Fix |
|------------|--------|-----|
| `netifaces` on every method call | O(interfaces) syscalls per request | Throttle to 30s interval |
| `netifaces` in `verify_request()` | Second layer of per-request overhead | Cache with 30s TTL |
| Debug `print()` in `get_interfaces_ip_addresses()` | Stdout I/O on every request | Remove |
| No graph caching | Redundant DDS queries during bursts | 200ms TTL cache |
| 2-hour `select()` timeout | No periodic maintenance possible | 200ms heartbeat + activity tracking |
| `pretty_print_call` wrapper | String format + print per request | Remove from production registration |
| No spinning after init | Guard conditions accumulate | Informational (queries bypass executor) |
