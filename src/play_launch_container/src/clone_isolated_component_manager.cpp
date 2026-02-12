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

#include <cerrno>
#include <cstring>
#include <stdexcept>

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

// ── Child entry point ───────────────────────────────────────────────────
//
// Runs in a clone'd process with its own stack, PID, and (if CLONE_SETTLS
// succeeded) its own TLS.  Sets the TID in TLS, resets signal handlers,
// then spins the executor.
static int child_executor_fn(void * arg)
{
  auto * boot = static_cast<ChildBootstrap *>(arg);

  // Set our TID in the TLS block so that glibc knows our identity.
  // Without this, pthread_create() and mutex operations fail with EAGAIN.
  if (boot->tls_tid_offset >= 0) {
    auto my_tid = static_cast<pid_t>(syscall(SYS_gettid));
    unsigned long fs_val = 0;
    syscall(SYS_arch_prctl, 0x1003 /* ARCH_GET_FS */, &fs_val);
    auto * tls_base = reinterpret_cast<char *>(fs_val);
    std::memcpy(tls_base + boot->tls_tid_offset, &my_tid, sizeof(my_tid));
  }

  // Reset all signal handlers to SIG_DFL (equivalent to CLONE_CLEAR_SIGHAND).
  // Without CLONE_SIGHAND the child has its own copy of the signal handler
  // table, so this only affects the child.
  struct sigaction sa
  {
  };
  sa.sa_handler = SIG_DFL;
  for (int sig = 1; sig < _NSIG; ++sig) {
    sigaction(sig, &sa, nullptr);
  }

  boot->exec->spin();
  _exit(0);
}

// ── Constructor / Destructor ────────────────────────────────────────────

CloneIsolatedComponentManager::CloneIsolatedComponentManager(
  std::weak_ptr<rclcpp::Executor> executor, std::string node_name,
  const rclcpp::NodeOptions & node_options)
: ObservableComponentManager(executor, std::move(node_name), node_options)
{
  RCLCPP_INFO(get_logger(), "Using clone(CLONE_VM) per-node process isolation");
  if (!get_tls_allocator()) {
    RCLCPP_WARN(
      get_logger(),
      "_dl_allocate_tls not found; children will share parent TLS (may cause heap issues)");
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
  // Stop monitor thread before cleaning up children
  if (monitor_thread_.joinable()) {
    monitor_running_ = false;
    if (stop_fd_ >= 0) {
      uint64_t val = 1;
      if (write(stop_fd_, &val, sizeof(val)) < 0) {
      }  // best-effort wakeup
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

// ── add_node_to_executor ────────────────────────────────────────────────
//
// Called by on_load_node() after the node is constructed and stored in
// node_wrappers_.  We create a dedicated executor and spawn a clone'd
// child process to spin it.

void CloneIsolatedComponentManager::add_node_to_executor(uint64_t node_id)
{
  // Create a dedicated single-threaded executor for this node
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
    node_name.c_str(), static_cast<unsigned long>(node_id));
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
        }  // best-effort drain
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

  // Publish CRASHED event
  auto event = play_launch_msgs::msg::ComponentEvent();
  event.stamp = now();
  event.event_type = play_launch_msgs::msg::ComponentEvent::CRASHED;
  event.unique_id = node_id;
  event.full_node_name = child.node_name;
  event.error_message = error_msg;
  event_pub_->publish(event);

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
