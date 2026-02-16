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

#ifndef PLAY_LAUNCH_CONTAINER__CLONE_ISOLATED_COMPONENT_MANAGER_HPP_
#define PLAY_LAUNCH_CONTAINER__CLONE_ISOLATED_COMPONENT_MANAGER_HPP_

#include <sys/types.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "play_launch_container/observable_component_manager.hpp"

namespace play_launch_container
{

/// Component manager that isolates each composable node in its own Linux process
/// via fork()+exec() of the `component_node` binary.
///
/// Each node runs as a fully independent process with its own address space,
/// glibc, TLS, and DDS participant.  This avoids all TLS/glibc issues that
/// plague clone(CLONE_VM) and supports full MultiThreadedExecutor.
///
/// Class hierarchy:
///   ComponentManager -> ObservableComponentManager -> CloneIsolatedComponentManager
///
/// Inherits event publishing from ObservableComponentManager.
/// Overrides add_node_to_executor / remove_node_from_executor (same pattern as
/// upstream ComponentManagerIsolated, but fork+exec instead of std::thread).
class CloneIsolatedComponentManager : public ObservableComponentManager
{
public:
  CloneIsolatedComponentManager(
    std::weak_ptr<rclcpp::Executor> executor =
      std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
    bool use_multi_threaded = false, std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & node_options =
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));

  ~CloneIsolatedComponentManager() override;

protected:
  void on_load_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response) override;

  void on_unload_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response) override;

  void on_list_nodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response) override;

  void add_node_to_executor(uint64_t node_id) override;
  void remove_node_from_executor(uint64_t node_id) override;

private:
  void worker_loop();
  void submit_work(std::function<void()> work);

  struct ChildInfo
  {
    pid_t pid;
    int pidfd;
    uint64_t node_id;
    std::string node_name;
    std::string param_file;  // temp YAML to unlink on cleanup
  };

  /// Serialize LoadNode::Request parameters to a temp YAML file.
  /// Returns empty string if no parameters.
  std::string write_params_file(const std::shared_ptr<LoadNode::Request> & request);

  /// Fork+exec component_node for the given load request.
  /// Returns ChildInfo on success, throws on failure.
  ChildInfo spawn_child_process(
    uint64_t node_id, const std::shared_ptr<LoadNode::Request> & request);

  void cleanup_child(ChildInfo & child);
  void monitor_loop();
  void handle_child_death(uint64_t node_id);

  bool use_multi_threaded_;
  std::string component_node_path_;  // resolved once in constructor

  // Thread safety for inherited protected members (node_wrappers_, loaders_, unique_id_)
  std::mutex load_mutex_;

  // Worker thread pool for async node construction
  std::vector<std::thread> worker_threads_;
  std::mutex work_queue_mutex_;
  std::condition_variable work_queue_cv_;
  std::queue<std::function<void()>> work_queue_;
  std::atomic<bool> workers_running_{false};
  static constexpr size_t kWorkerThreadCount = 4;

  // Track nodes with unique_id assigned but not yet constructed
  std::set<uint64_t> pending_node_ids_;
  std::mutex pending_mutex_;

  std::mutex children_mutex_;
  std::map<uint64_t, ChildInfo> children_;

  int epoll_fd_ = -1;
  int stop_fd_ = -1;
  std::thread monitor_thread_;
  std::atomic<bool> monitor_running_{false};
};

}  // namespace play_launch_container

#endif  // PLAY_LAUNCH_CONTAINER__CLONE_ISOLATED_COMPONENT_MANAGER_HPP_
