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
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "play_launch_container/observable_component_manager.hpp"

namespace play_launch_container
{

/// Component manager that isolates each composable node in its own Linux process
/// via clone(CLONE_VM).
///
/// Each node's executor spins in a clone'd child process (own PID, own signal
/// disposition) while sharing the container's address space for zero-copy
/// intra-process communication.
///
/// Class hierarchy:
///   ComponentManager -> ObservableComponentManager -> CloneIsolatedComponentManager
///
/// Inherits event publishing from ObservableComponentManager.
/// Overrides add_node_to_executor / remove_node_from_executor (same pattern as
/// upstream ComponentManagerIsolated, but clone() instead of std::thread).
class CloneIsolatedComponentManager : public ObservableComponentManager
{
public:
  CloneIsolatedComponentManager(
    std::weak_ptr<rclcpp::Executor> executor =
      std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
    std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & node_options =
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));

  ~CloneIsolatedComponentManager() override;

protected:
  void add_node_to_executor(uint64_t node_id) override;
  void remove_node_from_executor(uint64_t node_id) override;

private:
  struct ChildInfo
  {
    pid_t pid;
    int pidfd;
    std::shared_ptr<rclcpp::Executor> executor;
    void * stack_ptr;
    size_t stack_size;
    uint64_t node_id;
    std::string node_name;
    void * tls;   // glibc TLS block (allocated via _dl_allocate_tls)
    void * boot;  // ChildBootstrap* — passed to clone child, freed on cleanup
  };

  void cleanup_child(ChildInfo & child);
  void monitor_loop();
  void handle_child_death(uint64_t node_id);

  std::mutex children_mutex_;
  std::map<uint64_t, ChildInfo> children_;

  int epoll_fd_ = -1;
  int stop_fd_ = -1;
  std::thread monitor_thread_;
  std::atomic<bool> monitor_running_{false};

  static constexpr size_t kChildStackSize = 8 * 1024 * 1024;  // 8 MB
};

}  // namespace play_launch_container

#endif  // PLAY_LAUNCH_CONTAINER__CLONE_ISOLATED_COMPONENT_MANAGER_HPP_
