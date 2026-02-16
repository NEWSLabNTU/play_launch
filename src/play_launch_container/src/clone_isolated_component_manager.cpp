// Copyright 2026 play_launch developers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "play_launch_container/clone_isolated_component_manager.hpp"

#include <dlfcn.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <execinfo.h>
#include <locale.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace play_launch_container
{

// ── TLS allocation via glibc internals ──────────────────────────────────
//
// clone(CLONE_VM) shares the address space but NOT thread-local storage.
// Without CLONE_SETTLS, the child shares the parent's TLS pointer, which
// causes glibc's per-thread malloc cache (tcache) to be shared → double-free.
//
// Solution: allocate a fresh TLS block in the parent (via glibc's internal
// _dl_allocate_tls), then pass it to clone() with CLONE_SETTLS so the child
// starts with its own errno, tcache, and other thread-locals.
//
// The child must also set its TID in the TLS block so that glibc operations
// (pthread_create, mutex ownership checks) work correctly.
using dl_allocate_tls_fn = void * (*)(void *);
using dl_deallocate_tls_fn = void (*)(void *, bool);

static dl_allocate_tls_fn get_tls_allocator()
{
  static auto fn = reinterpret_cast<dl_allocate_tls_fn>(dlsym(RTLD_DEFAULT, "_dl_allocate_tls"));
  return fn;
}

static dl_deallocate_tls_fn get_tls_deallocator()
{
  static auto fn =
    reinterpret_cast<dl_deallocate_tls_fn>(dlsym(RTLD_DEFAULT, "_dl_deallocate_tls"));
  return fn;
}

// Find the offset of the `tid` field in glibc's struct pthread (the TLS
// header).  We scan the first 2048 bytes of the current thread's TLS for
// a match with our TID.  This offset is stable for a given glibc build.
static int find_tls_tid_offset()
{
  auto my_tid = static_cast<pid_t>(syscall(SYS_gettid));
  auto * tls_base = reinterpret_cast<char *>(pthread_self());
  for (int offset = 0; offset < 2048; offset += 4) {
    pid_t val;
    std::memcpy(&val, tls_base + offset, sizeof(val));
    if (val == my_tid) {
      return offset;
    }
  }
  return -1;
}

// The TID offset is the same for all threads/children in this process.
static int g_tls_tid_offset = -1;

// ── Child bootstrap ─────────────────────────────────────────────────────
//
// Passed through clone()'s `arg` to the child.  Lives in the shared address
// space (heap-allocated by parent, freed after child exits).

struct ChildBootstrap
{
  rclcpp::Executor * exec;
  int tls_tid_offset;
};

// Thread-local executor pointer for the clone child's SIGTERM handler.
// Each child has its own TLS (via CLONE_SETTLS), so this is per-child
// despite the shared address space.
static thread_local rclcpp::Executor * g_child_executor = nullptr;

static void child_sigterm_handler(int sig)
{
  // cancel() triggers the executor's interrupt guard condition (a pipe write),
  // which is async-signal-safe.  This causes spin() to return.
  if (g_child_executor) {
    g_child_executor->cancel();
  }
}

// Crash handler for clone children: prints a backtrace to stderr before
// re-raising to produce a core dump.  Helps diagnose issues in the isolated
// child process (fresh TLS, no debugger attached).
static void child_crash_handler(int sig)
{
  const char msg[] = "\n=== CLONE CHILD CRASH BACKTRACE ===\n";
  if (write(STDERR_FILENO, msg, sizeof(msg) - 1) < 0) {}
  void * bt[64];
  int n = backtrace(bt, 64);
  backtrace_symbols_fd(bt, n, STDERR_FILENO);
  const char end[] = "=== END BACKTRACE ===\n";
  if (write(STDERR_FILENO, end, sizeof(end) - 1) < 0) {}
  // Re-raise with SIG_DFL to get core dump
  struct sigaction sa{};
  sa.sa_handler = SIG_DFL;
  sigaction(sig, &sa, nullptr);
  raise(sig);
}

// ── Child entry point ───────────────────────────────────────────────────
//
// Runs in a clone'd process with its own stack, PID, and (if CLONE_SETTLS
// succeeded) its own TLS.  Sets the TID in TLS, installs a graceful SIGTERM
// handler, then spins the executor until cancelled.
static int child_executor_fn(void * arg)
{
  auto * boot = static_cast<ChildBootstrap *>(arg);

  // Set our TID in the TLS block so that glibc knows our identity.
  // Without this, pthread_create() and mutex operations fail with EAGAIN.
  auto my_tid = static_cast<pid_t>(syscall(SYS_gettid));
  uint64_t fs_val = 0;
  syscall(SYS_arch_prctl, 0x1003 /* ARCH_GET_FS */, &fs_val);
  auto * tls_base = reinterpret_cast<char *>(fs_val);

  if (boot->tls_tid_offset >= 0) {
    std::memcpy(tls_base + boot->tls_tid_offset, &my_tid, sizeof(my_tid));
  }

  // Ensure tcbhead_t.self and tcbhead_t.tcb point back to the TLS base.
  // _dl_allocate_tls() may not set these; they're needed by THREAD_SELF.
  // Also set multiple_threads = 1 (we're in a clone child alongside other threads).
  {
    auto ** self_field = reinterpret_cast<void **>(tls_base + 0x10);
    if (*self_field == nullptr) {
      // tcbhead_t.tcb = tcbhead_t.self = fs_base
      *reinterpret_cast<void **>(tls_base) = reinterpret_cast<void *>(fs_val);
      *self_field = reinterpret_cast<void *>(fs_val);
    }
    *reinterpret_cast<int *>(tls_base + 0x18) = 1;  // multiple_threads
  }

  // _dl_allocate_tls() creates a fresh TLS block but does NOT initialize the
  // struct pthread fields that pthread_create would normally set up:
  //
  //   1. locale pointer = NULL → __printf_fp_l SIGSEGV on float formatting
  //   2. ctype tables = NULL  → isalpha/isdigit SIGSEGV via __ctype_b_loc()
  //
  // Fix: set locale to global (so _NL_CURRENT falls back to global locale),
  // then call __ctype_init() to populate the per-thread ctype table pointers
  // from the current locale (what glibc's start_thread() does internally).
  uselocale(LC_GLOBAL_LOCALE);
  {
    using ctype_init_fn = void (*)();
    auto ctype_init = reinterpret_cast<ctype_init_fn>(dlsym(RTLD_DEFAULT, "__ctype_init"));
    if (ctype_init) {
      ctype_init();
    }
  }

  // Reset all signal handlers to SIG_DFL first (equivalent to CLONE_CLEAR_SIGHAND).
  // Without CLONE_SIGHAND the child has its own copy of the signal handler
  // table, so this only affects the child.
  struct sigaction sa
  {
  };
  sa.sa_handler = SIG_DFL;
  for (int sig = 1; sig < _NSIG; ++sig) {
    sigaction(sig, &sa, nullptr);
  }

  // Install graceful SIGTERM handler so destructors run (cleans up DDS/SHM).
  // The handler calls executor->cancel(), which wakes up spin() via a pipe write
  // (async-signal-safe).
  g_child_executor = boot->exec;
  sa.sa_handler = child_sigterm_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGTERM, &sa, nullptr);

  // Install crash handlers for diagnosis (backtrace to stderr, then core dump)
  sa.sa_handler = child_crash_handler;
  sa.sa_flags = SA_RESETHAND;  // one-shot, then SIG_DFL for core dump
  sigaction(SIGSEGV, &sa, nullptr);
  sigaction(SIGABRT, &sa, nullptr);
  sigaction(SIGBUS, &sa, nullptr);

  boot->exec->spin();

  // Normal return (not _exit) — C++ destructors run, including FastRTPS
  // cleanup which unlinks /dev/shm/fastrtps_* segments.
  return 0;
}

// ── Constructor / Destructor ────────────────────────────────────────────

CloneIsolatedComponentManager::CloneIsolatedComponentManager(
  std::weak_ptr<rclcpp::Executor> executor, bool use_multi_threaded, std::string node_name,
  const rclcpp::NodeOptions & node_options)
: ObservableComponentManager(executor, std::move(node_name), node_options),
  use_multi_threaded_(use_multi_threaded)
{
  RCLCPP_INFO(get_logger(), "Using clone(CLONE_VM) per-node process isolation (non-blocking load)");

  // Start worker thread pool for async node construction
  workers_running_ = true;
  for (size_t i = 0; i < kWorkerThreadCount; ++i) {
    worker_threads_.emplace_back(&CloneIsolatedComponentManager::worker_loop, this);
  }

  if (!get_tls_allocator()) {
    RCLCPP_WARN(
      get_logger(),
      "_dl_allocate_tls not found; children will share "
      "parent TLS (may cause heap issues)");
  }
  // Find the tid offset once — it's the same for all threads in this process.
  if (g_tls_tid_offset < 0) {
    g_tls_tid_offset = find_tls_tid_offset();
    if (g_tls_tid_offset < 0) {
      RCLCPP_WARN(get_logger(), "Could not find TID offset in glibc TLS; children may fail");
    }
  }

  // Set up child death monitor (epoll on pidfds)
  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd_ < 0) {
    RCLCPP_WARN(
      get_logger(), "epoll_create1 failed: %s — child crash monitoring disabled",
      std::strerror(errno));
    return;
  }

  stop_fd_ = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
  if (stop_fd_ < 0) {
    RCLCPP_WARN(
      get_logger(), "eventfd failed: %s — child crash monitoring disabled", std::strerror(errno));
    close(epoll_fd_);
    epoll_fd_ = -1;
    return;
  }

  // Register stop_fd_ with epoll (data.u64=0 as sentinel — node IDs start at 1)
  epoll_event ev{};
  ev.events = EPOLLIN;
  ev.data.u64 = 0;
  epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, stop_fd_, &ev);

  monitor_running_ = true;
  monitor_thread_ = std::thread(&CloneIsolatedComponentManager::monitor_loop, this);
}

CloneIsolatedComponentManager::~CloneIsolatedComponentManager()
{
  // Stop worker threads first (they may hold load_mutex_)
  {
    std::lock_guard<std::mutex> lock(work_queue_mutex_);
    workers_running_ = false;
  }
  work_queue_cv_.notify_all();
  for (auto & t : worker_threads_) {
    if (t.joinable()) {
      t.join();
    }
  }

  // Stop monitor thread before cleaning up children
  if (monitor_thread_.joinable()) {
    monitor_running_ = false;
    if (stop_fd_ >= 0) {
      uint64_t val = 1;
      if (write(stop_fd_, &val, sizeof(val)) < 0) {
        // best-effort wakeup — ignore errors
      }
    }
    monitor_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(children_mutex_);
    for (auto & [node_id, child] : children_) {
      cleanup_child(child);
    }
    children_.clear();
  }

  if (stop_fd_ >= 0) {
    close(stop_fd_);
  }
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
  }
}

// ── Worker thread pool ──────────────────────────────────────────────────

void CloneIsolatedComponentManager::worker_loop()
{
  while (true) {
    std::function<void()> work;
    {
      std::unique_lock<std::mutex> lock(work_queue_mutex_);
      work_queue_cv_.wait(lock, [this] { return !workers_running_ || !work_queue_.empty(); });
      if (!workers_running_) {
        // Exit immediately on shutdown — don't process remaining items,
        // the rcl context may already be invalid.
        return;
      }
      work = std::move(work_queue_.front());
      work_queue_.pop();
    }
    work();
  }
}

void CloneIsolatedComponentManager::submit_work(std::function<void()> work)
{
  {
    std::lock_guard<std::mutex> lock(work_queue_mutex_);
    work_queue_.push(std::move(work));
  }
  work_queue_cv_.notify_one();
}

// ── Non-blocking on_load_node ───────────────────────────────────────────
//
// Phase 1 (synchronous): validate plugin, pre-assign unique_id, respond.
// Phase 2 (async worker): construct node, add to executor, publish event.

void CloneIsolatedComponentManager::on_load_node(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<LoadNode::Request> request, std::shared_ptr<LoadNode::Response> response)
{
  // ── Phase 1: synchronous validation + pre-assign unique_id ──

  // Look up component resources (ament index, fast)
  auto resources = get_component_resources(request->package_name);

  // Find matching plugin and create factory
  std::shared_ptr<rclcpp_components::NodeFactory> factory;
  for (const auto & resource : resources) {
    if (resource.first != request->plugin_name) {
      continue;
    }
    auto f = create_component_factory(resource);
    if (f) {
      factory = std::move(f);
      break;
    }
  }

  if (!factory) {
    response->success = false;
    response->error_message =
      "Failed to find class with the requested plugin name '" + request->plugin_name + "'";
    RCLCPP_ERROR(get_logger(), "%s", response->error_message.c_str());
    return;
  }

  // Pre-create node options (parses params, fast)
  auto options = create_node_options(request);

  uint64_t node_id;
  {
    std::lock_guard<std::mutex> lock(load_mutex_);
    node_id = unique_id_++;
    if (0 == node_id) {
      throw std::overflow_error("exhausted the unique ids for components in this process");
    }
  }

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_node_ids_.insert(node_id);
  }

  // Approximate full_node_name for the immediate response.
  // The actual name comes from the constructed node (published via
  // ComponentEvent).
  std::string approx_name;
  if (!request->node_namespace.empty() && request->node_namespace != "/") {
    approx_name = request->node_namespace + "/" + request->node_name;
  } else if (!request->node_name.empty()) {
    approx_name = "/" + request->node_name;
  } else {
    approx_name = request->plugin_name;
  }

  // Respond immediately — node is not yet constructed
  response->success = true;
  response->unique_id = node_id;
  response->full_node_name = approx_name;

  RCLCPP_INFO(
    get_logger(),
    "Accepted load request for '%s' (pre-assigned id %lu), "
    "constructing async...",
    request->plugin_name.c_str(), static_cast<uint64_t>(node_id));

  // ── Phase 2: async construction on worker thread ──

  // Capture everything needed by the worker lambda.
  // factory and options are moved; request fields copied for the event.
  auto pkg = request->package_name;
  auto plugin = request->plugin_name;

  submit_work([this, factory = std::move(factory), options = std::move(options), node_id, pkg,
               plugin]() {
    // Guard: if shutdown already started, skip construction entirely.
    // The rcl context may be invalid (ros2/rclcpp#812).
    if (!rclcpp::ok()) {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_node_ids_.erase(node_id);
      return;
    }

    try {
      // Construct the node (SLOW — may take seconds or minutes)
      auto wrapper = factory->create_node_instance(options);

      // Re-check after construction — SIGTERM may have arrived mid-build
      if (!rclcpp::ok()) {
        RCLCPP_DEBUG(
          get_logger(), "Shutdown during construction of '%s' (id %lu), discarding", plugin.c_str(),
          static_cast<uint64_t>(node_id));
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_node_ids_.erase(node_id);
        return;
      }

      std::string actual_name;
      {
        std::lock_guard<std::mutex> lock(load_mutex_);
        node_wrappers_[node_id] = wrapper;
        add_node_to_executor(node_id);
        actual_name = node_wrappers_[node_id].get_node_base_interface()->get_fully_qualified_name();
      }

      {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_node_ids_.erase(node_id);
      }

      RCLCPP_INFO(
        get_logger(), "Component '%s' loaded as '%s' (id %lu)", plugin.c_str(), actual_name.c_str(),
        static_cast<uint64_t>(node_id));

      // Publish LOADED event (skip if context is shutting down)
      if (rclcpp::ok()) {
        auto event = play_launch_msgs::msg::ComponentEvent();
        event.stamp = now();
        event.event_type = play_launch_msgs::msg::ComponentEvent::LOADED;
        event.unique_id = node_id;
        event.full_node_name = actual_name;
        event.package_name = pkg;
        event.plugin_name = plugin;
        event_pub_->publish(event);
      }
    } catch (const std::exception & ex) {
      {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_node_ids_.erase(node_id);
      }

      // During shutdown, rcl context errors are expected — don't publish
      // events (the publisher may also be invalid) or log errors.
      if (!rclcpp::ok()) {
        RCLCPP_DEBUG(
          get_logger(), "Construction interrupted by shutdown for '%s' (id %lu)", plugin.c_str(),
          static_cast<uint64_t>(node_id));
        return;
      }

      RCLCPP_ERROR(
        get_logger(), "Failed to construct component '%s' (id %lu): %s", plugin.c_str(),
        static_cast<uint64_t>(node_id), ex.what());

      // Publish LOAD_FAILED event
      auto event = play_launch_msgs::msg::ComponentEvent();
      event.stamp = now();
      event.event_type = play_launch_msgs::msg::ComponentEvent::LOAD_FAILED;
      event.unique_id = node_id;
      event.package_name = pkg;
      event.plugin_name = plugin;
      event.error_message = ex.what();
      event_pub_->publish(event);
    }
  });
}

// ── on_unload_node override (thread-safe) ───────────────────────────────

void CloneIsolatedComponentManager::on_unload_node(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<UnloadNode::Request> request,
  std::shared_ptr<UnloadNode::Response> response)
{
  // Reject unload if node is still being constructed
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_node_ids_.count(request->unique_id) > 0) {
      response->success = false;
      response->error_message =
        "Node with unique_id " + std::to_string(request->unique_id) + " is still being constructed";
      RCLCPP_WARN(get_logger(), "%s", response->error_message.c_str());
      return;
    }
  }

  // Capture node name before parent erases it
  std::string full_name;
  {
    std::lock_guard<std::mutex> lock(load_mutex_);
    auto it = node_wrappers_.find(request->unique_id);
    if (it != node_wrappers_.end()) {
      full_name = it->second.get_node_base_interface()->get_fully_qualified_name();
    }
  }

  // Delegate to parent (which calls remove_node_from_executor →
  // children_mutex_)
  {
    std::lock_guard<std::mutex> lock(load_mutex_);
    ComponentManager::on_unload_node(request_header, request, response);
  }

  // Publish UNLOADED event on success (skip if context is shutting down)
  if (response->success && rclcpp::ok()) {
    auto event = play_launch_msgs::msg::ComponentEvent();
    event.stamp = now();
    event.event_type = play_launch_msgs::msg::ComponentEvent::UNLOADED;
    event.unique_id = request->unique_id;
    event.full_node_name = full_name;
    event_pub_->publish(event);
  }
}

// ── on_list_nodes override (thread-safe, excludes pending) ──────────────

void CloneIsolatedComponentManager::on_list_nodes(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<ListNodes::Request> /*request*/,
  std::shared_ptr<ListNodes::Response> response)
{
  std::lock_guard<std::mutex> lock1(load_mutex_);
  std::lock_guard<std::mutex> lock2(pending_mutex_);

  for (auto & wrapper : node_wrappers_) {
    // Skip nodes still being constructed
    if (pending_node_ids_.count(wrapper.first) > 0) {
      continue;
    }
    response->unique_ids.push_back(wrapper.first);
    response->full_node_names.push_back(
      wrapper.second.get_node_base_interface()->get_fully_qualified_name());
  }
}

// ── add_node_to_executor ────────────────────────────────────────────────
//
// Called from worker thread after the node is constructed and stored in
// node_wrappers_.  Caller must hold load_mutex_.  We create a dedicated
// executor and spawn a clone'd child process to spin it.

void CloneIsolatedComponentManager::add_node_to_executor(uint64_t node_id)
{
  // Always use SingleThreadedExecutor for clone children.
  // _dl_allocate_tls(nullptr) creates TLS but does NOT fully initialize the
  // struct pthread header (robust_list, thread list, stack info, etc.) that
  // pthread_create needs.  MultiThreadedExecutor::spin() calls pthread_create
  // to spawn worker threads, which crashes because the calling thread's struct
  // pthread is incomplete.  Each node already runs in its own isolated process,
  // so multi-threading within each child is unnecessary.
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node_wrappers_[node_id].get_node_base_interface());

  // Allocate child stack (MAP_STACK hints the kernel for guard pages)
  void * stack = mmap(
    nullptr, kChildStackSize, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK, -1,
    0);
  if (stack == MAP_FAILED) {
    throw std::runtime_error(
      "Failed to mmap stack for clone child: " + std::string(std::strerror(errno)));
  }

  // clone() needs the TOP of the stack (stack grows downward on x86_64/aarch64)
  void * stack_top = static_cast<char *>(stack) + kChildStackSize;

  // Allocate fresh TLS for the child (parent context, so malloc is safe).
  // This gives the child its own errno, tcache, and other glibc thread-locals.
  void * child_tls = nullptr;
  auto tls_alloc = get_tls_allocator();
  if (tls_alloc) {
    child_tls = tls_alloc(nullptr);
  }

  // Bootstrap struct lives on the heap (shared address space).
  // The child reads it; we store the pointer in ChildInfo for cleanup.
  auto * boot = new ChildBootstrap{exec.get(), g_tls_tid_offset};

  // CLONE_VM:      share address space (zero-copy intra-process)
  // CLONE_FILES:   share file descriptor table
  // CLONE_FS:      share cwd/root/umask
  // CLONE_SETTLS:  give child its own TLS (avoids tcache sharing)
  // low byte:      SIGCHLD — notify parent on child exit
  int flags = CLONE_VM | CLONE_FILES | CLONE_FS | SIGCHLD;
  if (child_tls) {
    flags |= CLONE_SETTLS;
  }

  // clone(fn, stack_top, flags, arg, ptid, tls, ctid)
  pid_t child_pid = clone(
    child_executor_fn, stack_top, flags, boot,
    /*ptid=*/nullptr, /*tls=*/child_tls, /*ctid=*/nullptr);
  if (child_pid == -1) {
    int err = errno;
    delete boot;
    munmap(stack, kChildStackSize);
    if (child_tls) {
      auto tls_dealloc = get_tls_deallocator();
      if (tls_dealloc) {
        tls_dealloc(child_tls, true);
      }
    }
    throw std::runtime_error("clone() failed: " + std::string(std::strerror(err)));
  }

  // Get pidfd for race-free monitoring (Linux 5.3+).
  // Can fail if the child already exited; -1 is acceptable.
  int pidfd = static_cast<int>(syscall(SYS_pidfd_open, child_pid, 0));

  // Get node name for logging
  std::string node_name;
  auto it = node_wrappers_.find(node_id);
  if (it != node_wrappers_.end()) {
    node_name = it->second.get_node_base_interface()->get_fully_qualified_name();
  }

  // Store child info (boot pointer kept for lifetime — child may still read it)
  {
    std::lock_guard<std::mutex> lock(children_mutex_);
    children_[node_id] = ChildInfo{child_pid, pidfd,           exec,
                                   stack,     kChildStackSize, node_id,
                                   node_name, child_tls,       static_cast<void *>(boot)};
  }

  // Register pidfd with epoll for crash monitoring
  if (epoll_fd_ >= 0 && pidfd >= 0) {
    epoll_event ev{};
    ev.events = EPOLLIN;
    ev.data.u64 = node_id;
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, pidfd, &ev);
  }

  RCLCPP_INFO(
    get_logger(), "Spawned isolated child PID %d for node '%s' (id %lu)", child_pid,
    node_name.c_str(), static_cast<uint64_t>(node_id));
}

// ── remove_node_from_executor ───────────────────────────────────────────
//
// Called by on_unload_node() before the node is erased from node_wrappers_.
// We cancel the executor, wait for the child to exit, and clean up.

void CloneIsolatedComponentManager::remove_node_from_executor(uint64_t node_id)
{
  std::lock_guard<std::mutex> lock(children_mutex_);
  auto it = children_.find(node_id);
  if (it == children_.end()) {
    // Already cleaned up by monitor (child crashed)
    return;
  }

  auto & child = it->second;

  // Deregister pidfd from epoll BEFORE killing — ensures the monitor only
  // sees unexpected deaths, not graceful unloads.
  if (epoll_fd_ >= 0 && child.pidfd >= 0) {
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, child.pidfd, nullptr);
  }

  // Wait for the executor to start spinning before cancelling, same pattern
  // as upstream ComponentManagerIsolated::cancel_executor
  while (!child.executor->is_spinning()) {
    rclcpp::sleep_for(std::chrono::milliseconds(1));
  }

  // Cancel executor: sets spinning=false, child's spin() returns → _exit(0)
  child.executor->cancel();

  // Wait for graceful exit (child calls _exit after spin returns)
  int status = 0;
  for (int i = 0; i < 200; ++i) {
    if (waitpid(child.pid, &status, WNOHANG) != 0) {
      break;
    }
    usleep(10000);  // 10ms, up to 2 seconds total
  }

  // Force kill if still alive
  if (waitpid(child.pid, &status, WNOHANG) == 0) {
    kill(child.pid, SIGKILL);
    waitpid(child.pid, &status, 0);
  }

  RCLCPP_INFO(
    get_logger(), "Removed isolated child PID %d for node '%s'", child.pid,
    child.node_name.c_str());

  // Free resources
  munmap(child.stack_ptr, child.stack_size);
  if (child.pidfd >= 0) {
    close(child.pidfd);
  }
  auto tls_dealloc = get_tls_deallocator();
  if (tls_dealloc && child.tls) {
    tls_dealloc(child.tls, true);
  }
  delete static_cast<ChildBootstrap *>(child.boot);

  children_.erase(it);
}

// ── monitor_loop — epoll on pidfds ──────────────────────────────────────
//
// Blocks on epoll_wait until a child pidfd becomes readable (child exited)
// or stop_fd_ is signalled (shutdown).  Zero CPU when idle.

void CloneIsolatedComponentManager::monitor_loop()
{
  constexpr int kMaxEvents = 8;
  epoll_event events[kMaxEvents];

  while (monitor_running_) {
    int n = epoll_wait(epoll_fd_, events, kMaxEvents, -1);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    for (int i = 0; i < n; ++i) {
      if (events[i].data.u64 == 0) {
        // Shutdown signal — drain eventfd and exit
        uint64_t val;
        if (read(stop_fd_, &val, sizeof(val)) < 0) {
          // best-effort drain — ignore errors
        }
        continue;
      }
      handle_child_death(events[i].data.u64);
    }
  }
}

// ── handle_child_death — cleanup crashed child + publish event ──────────

void CloneIsolatedComponentManager::handle_child_death(uint64_t node_id)
{
  std::lock_guard<std::mutex> lock(children_mutex_);
  auto it = children_.find(node_id);
  if (it == children_.end()) {
    // Graceful unload already handled this child
    return;
  }

  auto & child = it->second;

  // Reap child and get exit details
  siginfo_t info{};
  int status = 0;
  waitpid(child.pid, &status, WNOHANG);
  if (WIFSIGNALED(status)) {
    info.si_code = CLD_KILLED;
    info.si_status = WTERMSIG(status);
#ifdef WCOREDUMP
    if (WCOREDUMP(status)) {
      info.si_code = CLD_DUMPED;
    }
#endif
  } else if (WIFEXITED(status)) {
    info.si_code = CLD_EXITED;
    info.si_status = WEXITSTATUS(status);
  }

  // Build error message with crash details
  std::string error_msg;
  if (info.si_code == CLD_KILLED || info.si_code == CLD_DUMPED) {
    error_msg = "killed by signal " + std::to_string(info.si_status) + " (" +
                std::string(strsignal(info.si_status)) + ")";
    if (info.si_code == CLD_DUMPED) {
      error_msg += " (core dumped)";
    }
  } else if (info.si_code == CLD_EXITED) {
    error_msg = "exited with status " + std::to_string(info.si_status);
  } else {
    error_msg = "died (unknown cause)";
  }

  RCLCPP_ERROR(
    get_logger(), "Child PID %d crashed for node '%s': %s", child.pid, child.node_name.c_str(),
    error_msg.c_str());

  // Publish CRASHED event. Use try-catch instead of rclcpp::ok() guard here
  // because crash detection must work during normal operation (not just shutdown).
  try {
    auto event = play_launch_msgs::msg::ComponentEvent();
    event.stamp = now();
    event.event_type = play_launch_msgs::msg::ComponentEvent::CRASHED;
    event.unique_id = node_id;
    event.full_node_name = child.node_name;
    event.error_message = error_msg;
    event_pub_->publish(event);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Failed to publish CRASHED event: %s", ex.what());
  }

  // Free resources
  munmap(child.stack_ptr, child.stack_size);
  if (child.pidfd >= 0) {
    close(child.pidfd);
  }
  auto tls_dealloc = get_tls_deallocator();
  if (tls_dealloc && child.tls) {
    tls_dealloc(child.tls, true);
  }
  delete static_cast<ChildBootstrap *>(child.boot);

  children_.erase(it);
}

// ── cleanup_child (destructor helper) ───────────────────────────────────

void CloneIsolatedComponentManager::cleanup_child(ChildInfo & child)
{
  child.executor->cancel();
  kill(child.pid, SIGTERM);
  int status = 0;
  for (int i = 0; i < 50; ++i) {
    if (waitpid(child.pid, &status, WNOHANG) != 0) {
      break;
    }
    usleep(10000);  // 10ms, up to 500ms total
  }
  if (waitpid(child.pid, &status, WNOHANG) == 0) {
    kill(child.pid, SIGKILL);
    waitpid(child.pid, &status, 0);
  }
  munmap(child.stack_ptr, child.stack_size);
  if (child.pidfd >= 0) {
    close(child.pidfd);
  }
  auto tls_dealloc = get_tls_deallocator();
  if (tls_dealloc && child.tls) {
    tls_dealloc(child.tls, true);
  }
  delete static_cast<ChildBootstrap *>(child.boot);
}

}  // namespace play_launch_container
