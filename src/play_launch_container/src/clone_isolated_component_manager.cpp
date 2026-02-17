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

#include <poll.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace play_launch_container
{

// ── Helper: resolve component_node binary path ──────────────────────────
//
// component_node is installed next to component_container under
// lib/<project_name>/.  We find our own directory via /proc/self/exe.

static std::string resolve_component_node_path()
{
  char buf[4096];
  ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (len <= 0) {
    return "component_node";  // fallback: hope it's on PATH
  }
  buf[len] = '\0';
  std::string exe_path(buf);
  auto slash = exe_path.rfind('/');
  if (slash == std::string::npos) {
    return "component_node";
  }
  return exe_path.substr(0, slash + 1) + "component_node";
}

// ── Constructor / Destructor ────────────────────────────────────────────

CloneIsolatedComponentManager::CloneIsolatedComponentManager(
  std::weak_ptr<rclcpp::Executor> executor, bool use_multi_threaded, std::string node_name,
  const rclcpp::NodeOptions & node_options)
: ObservableComponentManager(executor, std::move(node_name), node_options),
  use_multi_threaded_(use_multi_threaded),
  component_node_path_(resolve_component_node_path())
{
  RCLCPP_INFO(get_logger(), "Using fork()+exec() per-node process isolation (non-blocking load)");
  RCLCPP_DEBUG(get_logger(), "component_node binary: %s", component_node_path_.c_str());

  // Start worker thread pool for async node spawning
  workers_running_ = true;
  for (size_t i = 0; i < kWorkerThreadCount; ++i) {
    worker_threads_.emplace_back(&CloneIsolatedComponentManager::worker_loop, this);
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

// ── Parameter serialization ─────────────────────────────────────────────
//
// Convert LoadNode::Request parameters (rcl_interfaces/Parameter[])
// to a YAML file that can be passed via --params-file.

std::string CloneIsolatedComponentManager::write_params_file(
  const std::shared_ptr<LoadNode::Request> & request)
{
  if (request->parameters.empty()) {
    return "";
  }

  // Use wildcard namespace — component_node uses use_global_arguments(true)
  // so the YAML namespace must match any node name.
  std::ostringstream yaml;
  yaml << "/**:\n";
  yaml << "  ros__parameters:\n";

  for (const auto & param : request->parameters) {
    yaml << "    " << param.name << ": ";

    const auto & val = param.value;
    switch (val.type) {
      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
        yaml << (val.bool_value ? "true" : "false");
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
        yaml << val.integer_value;
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE: {
        char dbuf[64];
        std::snprintf(dbuf, sizeof(dbuf), "%.17g", val.double_value);
        std::string ds(dbuf);
        // Ensure decimal point for ROS type preservation
        if (ds.find('.') == std::string::npos && ds.find('e') == std::string::npos) {
          ds += ".0";
        }
        yaml << ds;
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
        yaml << "'" << param.value.string_value << "'";
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY: {
        yaml << "[";
        for (size_t i = 0; i < val.byte_array_value.size(); ++i) {
          if (i > 0) yaml << ", ";
          yaml << static_cast<int>(val.byte_array_value[i]);
        }
        yaml << "]";
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY: {
        yaml << "[";
        for (size_t i = 0; i < val.bool_array_value.size(); ++i) {
          if (i > 0) yaml << ", ";
          yaml << (val.bool_array_value[i] ? "true" : "false");
        }
        yaml << "]";
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY: {
        yaml << "[";
        for (size_t i = 0; i < val.integer_array_value.size(); ++i) {
          if (i > 0) yaml << ", ";
          yaml << val.integer_array_value[i];
        }
        yaml << "]";
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY: {
        yaml << "[";
        for (size_t i = 0; i < val.double_array_value.size(); ++i) {
          if (i > 0) yaml << ", ";
          char dbuf[64];
          std::snprintf(dbuf, sizeof(dbuf), "%.17g", val.double_array_value[i]);
          std::string ds(dbuf);
          // Ensure decimal point for ROS type preservation
          if (ds.find('.') == std::string::npos && ds.find('e') == std::string::npos) {
            ds += ".0";
          }
          yaml << ds;
        }
        yaml << "]";
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY: {
        yaml << "[";
        for (size_t i = 0; i < val.string_array_value.size(); ++i) {
          if (i > 0) yaml << ", ";
          yaml << "'" << val.string_array_value[i] << "'";
        }
        yaml << "]";
        break;
      }
      default:
        yaml << "null";
        break;
    }
    yaml << "\n";
  }

  // Write to temp file
  char tmp_path[] = "/tmp/play_launch_params_XXXXXX";
  int fd = mkstemp(tmp_path);
  if (fd < 0) {
    RCLCPP_WARN(get_logger(), "Failed to create temp params file: %s", std::strerror(errno));
    return "";
  }

  std::string content = yaml.str();
  size_t written = 0;
  while (written < content.size()) {
    auto n = ::write(fd, content.data() + written, content.size() - written);
    if (n <= 0) {
      break;
    }
    written += static_cast<size_t>(n);
  }
  close(fd);

  return std::string(tmp_path);
}

// ── spawn_child_process ─────────────────────────────────────────────────
//
// Fork+exec the component_node binary for a single composable node.
// Reads the ready pipe to get the node's full name or error message.

CloneIsolatedComponentManager::ChildInfo CloneIsolatedComponentManager::spawn_child_process(
  uint64_t node_id, const std::shared_ptr<LoadNode::Request> & request)
{
  // Build argument list for component_node
  std::vector<std::string> args;
  args.push_back(component_node_path_);
  args.push_back("--package");
  args.push_back(request->package_name);
  args.push_back("--plugin");
  args.push_back(request->plugin_name);

  // Ready pipe fd — will be set after pipe() below
  args.push_back("--ready-fd");
  args.push_back("");  // placeholder, filled after pipe()

  if (use_multi_threaded_) {
    args.push_back("--use-multi-threaded-executor");
  }

  // Check extra_arguments for use_intra_process_comms
  for (const auto & extra : request->extra_arguments) {
    if (
      extra.name == "use_intra_process_comms" &&
      extra.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL &&
      extra.value.bool_value) {
      args.push_back("--use-intra-process-comms");
      break;
    }
  }

  // Serialize parameters to temp YAML
  std::string param_file = write_params_file(request);

  // --ros-args section
  args.push_back("--ros-args");

  // Node name and namespace remapping
  if (!request->node_name.empty()) {
    args.push_back("-r");
    args.push_back("__node:=" + request->node_name);
  }
  if (!request->node_namespace.empty()) {
    args.push_back("-r");
    args.push_back("__ns:=" + request->node_namespace);
  }

  // Log level (uint8: 0=unset, 10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL)
  if (request->log_level > 0) {
    args.push_back("--log-level");
    args.push_back(std::to_string(request->log_level));
  }

  // Extra remap rules
  for (const auto & remap : request->remap_rules) {
    args.push_back("-r");
    args.push_back(remap);
  }

  // Params file
  if (!param_file.empty()) {
    args.push_back("--params-file");
    args.push_back(param_file);
  }

  // Create ready pipe
  int pipefd[2];
  if (pipe(pipefd) != 0) {
    if (!param_file.empty()) {
      unlink(param_file.c_str());
    }
    throw std::runtime_error("pipe() failed: " + std::string(std::strerror(errno)));
  }

  // Fill in the ready-fd placeholder
  args[6] = std::to_string(pipefd[1]);

  // Build C-style argv
  std::vector<char *> c_argv;
  for (auto & a : args) {
    c_argv.push_back(a.data());
  }
  c_argv.push_back(nullptr);

  pid_t child_pid = fork();
  if (child_pid < 0) {
    int err = errno;
    close(pipefd[0]);
    close(pipefd[1]);
    if (!param_file.empty()) {
      unlink(param_file.c_str());
    }
    throw std::runtime_error("fork() failed: " + std::string(std::strerror(err)));
  }

  if (child_pid == 0) {
    // ── Child process ──
    close(pipefd[0]);  // close read end

    // Ask the kernel to send SIGTERM to this child if the parent (container)
    // dies for any reason (including SIGKILL).  This prevents orphans.
    // Must be called after fork() but before exec() — PR_SET_PDEATHSIG is
    // reset across setuid exec but preserved across normal exec.
    prctl(PR_SET_PDEATHSIG, SIGTERM);

    // Guard against a race: if the parent already died between fork() and
    // the prctl() above, getppid() returns 1 (init adopted us).
    if (getppid() == 1) {
      _exit(1);
    }

    // The write end (pipefd[1]) must NOT be close-on-exec since component_node
    // reads the fd number from --ready-fd.  pipe() fds are not CLOEXEC by default.

    execvp(c_argv[0], c_argv.data());

    // If exec fails, write error to pipe and exit
    std::string err = "ERR execvp failed: " + std::string(std::strerror(errno));
    err += "\n";
    if (write(pipefd[1], err.data(), err.size()) < 0) {
      // Suppress warn_unused_result; best-effort write before _exit
    }
    close(pipefd[1]);
    _exit(127);
  }

  // ── Parent process ──
  close(pipefd[1]);  // close write end

  // Read ready message from pipe with timeout
  struct pollfd pfd
  {
  };
  pfd.fd = pipefd[0];
  pfd.events = POLLIN;
  constexpr int kReadyTimeoutMs = 30000;  // 30s matches LoadNode service timeout

  std::string ready_buf;
  bool got_response = false;

  while (true) {
    int ret = poll(&pfd, 1, kReadyTimeoutMs);
    if (ret <= 0) {
      // Timeout or error
      break;
    }
    char buf[1024];
    ssize_t n = read(pipefd[0], buf, sizeof(buf) - 1);
    if (n <= 0) {
      break;  // EOF or error
    }
    buf[n] = '\0';
    ready_buf += buf;
    if (ready_buf.find('\n') != std::string::npos) {
      got_response = true;
      break;
    }
  }
  close(pipefd[0]);

  // Parse response
  std::string node_name;
  if (!got_response || ready_buf.empty()) {
    // Child died or timed out before responding
    kill(child_pid, SIGKILL);
    waitpid(child_pid, nullptr, 0);
    if (!param_file.empty()) {
      unlink(param_file.c_str());
    }
    throw std::runtime_error("component_node did not respond (timeout or crash)");
  }

  // Trim trailing newline
  if (!ready_buf.empty() && ready_buf.back() == '\n') {
    ready_buf.pop_back();
  }

  if (ready_buf.substr(0, 3) == "OK ") {
    node_name = ready_buf.substr(3);
  } else if (ready_buf.substr(0, 4) == "ERR ") {
    std::string err_msg = ready_buf.substr(4);
    kill(child_pid, SIGKILL);
    waitpid(child_pid, nullptr, 0);
    if (!param_file.empty()) {
      unlink(param_file.c_str());
    }
    throw std::runtime_error("component_node failed: " + err_msg);
  } else {
    kill(child_pid, SIGKILL);
    waitpid(child_pid, nullptr, 0);
    if (!param_file.empty()) {
      unlink(param_file.c_str());
    }
    throw std::runtime_error("component_node: unexpected response: " + ready_buf);
  }

  // Get pidfd for race-free monitoring (Linux 5.3+)
  int pidfd = static_cast<int>(syscall(SYS_pidfd_open, child_pid, 0));

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

  return ChildInfo{child_pid, pidfd, node_id, node_name, param_file};
}

// ── Non-blocking on_load_node ───────────────────────────────────────────
//
// Phase 1 (synchronous): validate plugin exists, pre-assign unique_id, respond.
// Phase 2 (async worker): fork+exec component_node, read ready pipe, publish event.

void CloneIsolatedComponentManager::on_load_node(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<LoadNode::Request> request, std::shared_ptr<LoadNode::Response> response)
{
  // ── Phase 1: synchronous validation + pre-assign unique_id ──

  // Look up component resources (ament index, fast)
  auto resources = get_component_resources(request->package_name);

  // Verify plugin exists
  bool found = false;
  for (const auto & resource : resources) {
    if (resource.first == request->plugin_name) {
      found = true;
      break;
    }
  }

  if (!found) {
    response->success = false;
    response->error_message =
      "Failed to find class with the requested plugin name '" + request->plugin_name + "'";
    RCLCPP_ERROR(get_logger(), "%s", response->error_message.c_str());
    return;
  }

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
  std::string approx_name;
  if (!request->node_namespace.empty() && request->node_namespace != "/") {
    approx_name = request->node_namespace + "/" + request->node_name;
  } else if (!request->node_name.empty()) {
    approx_name = "/" + request->node_name;
  } else {
    approx_name = request->plugin_name;
  }

  // Respond immediately — node is not yet spawned
  response->success = true;
  response->unique_id = node_id;
  response->full_node_name = approx_name;

  RCLCPP_INFO(
    get_logger(),
    "Accepted load request for '%s' (pre-assigned id %lu), "
    "spawning async...",
    request->plugin_name.c_str(), static_cast<uint64_t>(node_id));

  // ── Phase 2: async spawn on worker thread ──

  auto pkg = request->package_name;
  auto plugin = request->plugin_name;

  submit_work([this, request, node_id, pkg, plugin]() {
    // Guard: if shutdown already started, skip spawn entirely.
    if (!rclcpp::ok()) {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_node_ids_.erase(node_id);
      return;
    }

    try {
      auto child = spawn_child_process(node_id, request);
      std::string actual_name = child.node_name;

      // Re-check after spawn — SIGTERM may have arrived
      if (!rclcpp::ok()) {
        RCLCPP_DEBUG(
          get_logger(), "Shutdown during spawn of '%s' (id %lu), killing child", plugin.c_str(),
          static_cast<uint64_t>(node_id));
        cleanup_child(child);
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_node_ids_.erase(node_id);
        return;
      }

      {
        std::lock_guard<std::mutex> lock(children_mutex_);
        children_[node_id] = std::move(child);
      }

      {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_node_ids_.erase(node_id);
      }

      RCLCPP_INFO(
        get_logger(), "Component '%s' loaded as '%s' (id %lu)", plugin.c_str(), actual_name.c_str(),
        static_cast<uint64_t>(node_id));

      // Publish LOADED event
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

      if (!rclcpp::ok()) {
        RCLCPP_DEBUG(
          get_logger(), "Spawn interrupted by shutdown for '%s' (id %lu)", plugin.c_str(),
          static_cast<uint64_t>(node_id));
        return;
      }

      RCLCPP_ERROR(
        get_logger(), "Failed to spawn component '%s' (id %lu): %s", plugin.c_str(),
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

// ── on_unload_node override ─────────────────────────────────────────────

void CloneIsolatedComponentManager::on_unload_node(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
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

  std::string full_name;
  {
    std::lock_guard<std::mutex> lock(children_mutex_);
    auto it = children_.find(request->unique_id);
    if (it == children_.end()) {
      response->success = false;
      response->error_message = "No node with unique_id " + std::to_string(request->unique_id);
      return;
    }

    full_name = it->second.node_name;

    // Deregister pidfd from epoll BEFORE killing
    if (epoll_fd_ >= 0 && it->second.pidfd >= 0) {
      epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, it->second.pidfd, nullptr);
    }

    cleanup_child(it->second);
    children_.erase(it);
  }

  response->success = true;

  RCLCPP_INFO(
    get_logger(), "Unloaded node '%s' (id %lu)", full_name.c_str(),
    static_cast<uint64_t>(request->unique_id));

  // Publish UNLOADED event
  if (rclcpp::ok()) {
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
  std::lock_guard<std::mutex> lock1(children_mutex_);
  std::lock_guard<std::mutex> lock2(pending_mutex_);

  for (const auto & [id, child] : children_) {
    // Skip nodes still being constructed
    if (pending_node_ids_.count(id) > 0) {
      continue;
    }
    response->unique_ids.push_back(id);
    response->full_node_names.push_back(child.node_name);
  }
}

// ── add_node_to_executor ────────────────────────────────────────────────
//
// Not used in fork+exec mode (children manage their own executors).
// Kept as a no-op to satisfy the override requirement.

void CloneIsolatedComponentManager::add_node_to_executor(uint64_t /*node_id*/)
{
  // No-op: fork+exec children manage their own executor.
}

// ── remove_node_from_executor ───────────────────────────────────────────
//
// Not used in fork+exec mode (children are killed directly).
// Kept as a no-op to satisfy the override requirement.

void CloneIsolatedComponentManager::remove_node_from_executor(uint64_t /*node_id*/)
{
  // No-op: fork+exec children are killed directly in on_unload_node.
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
  int status = 0;
  waitpid(child.pid, &status, WNOHANG);

  std::string error_msg;
  if (WIFSIGNALED(status)) {
    error_msg = "killed by signal " + std::to_string(WTERMSIG(status)) + " (" +
                std::string(strsignal(WTERMSIG(status))) + ")";
#ifdef WCOREDUMP
    if (WCOREDUMP(status)) {
      error_msg += " (core dumped)";
    }
#endif
  } else if (WIFEXITED(status)) {
    error_msg = "exited with status " + std::to_string(WEXITSTATUS(status));
  } else {
    error_msg = "died (unknown cause)";
  }

  RCLCPP_ERROR(
    get_logger(), "Child PID %d crashed for node '%s': %s", child.pid, child.node_name.c_str(),
    error_msg.c_str());

  // Publish CRASHED event
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

  // Clean up resources
  if (child.pidfd >= 0) {
    close(child.pidfd);
  }
  if (!child.param_file.empty()) {
    unlink(child.param_file.c_str());
  }

  children_.erase(it);
}

// ── cleanup_child (destructor / unload helper) ──────────────────────────

void CloneIsolatedComponentManager::cleanup_child(ChildInfo & child)
{
  // Graceful shutdown: SIGTERM, wait, then SIGKILL
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

  if (child.pidfd >= 0) {
    close(child.pidfd);
  }
  if (!child.param_file.empty()) {
    unlink(child.param_file.c_str());
  }
}

}  // namespace play_launch_container
