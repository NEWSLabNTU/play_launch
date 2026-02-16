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

#include <memory>
#include <string>
#include <vector>

#include "play_launch_container/clone_isolated_component_manager.hpp"
#include "play_launch_container/observable_component_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  /// Observable component container with configurable executor and isolation.
  ///
  /// Usage:
  ///   component_container                              # single-threaded (default)
  ///   component_container --use_multi_threaded_executor # multi-threaded
  ///   component_container --isolated                   # fork+exec per-node
  ///   component_container --isolated --use_multi_threaded_executor
  rclcpp::init(argc, argv);

  // Parse CLI args (same pattern as upstream component_container_isolated.cpp)
  bool use_multi_threaded = false;
  bool use_isolated = false;
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  for (const auto & arg : args) {
    if (arg == "--use_multi_threaded_executor") {
      use_multi_threaded = true;
    } else if (arg == "--isolated") {
      use_isolated = true;
    }
  }

  // Create executor
  std::shared_ptr<rclcpp::Executor> exec;
  if (use_multi_threaded) {
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  } else {
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  // Create manager (CloneIsolated inherits from Observable)
  std::shared_ptr<play_launch_container::ObservableComponentManager> node;
  if (use_isolated) {
    node = std::make_shared<play_launch_container::CloneIsolatedComponentManager>(
      exec, use_multi_threaded);
  } else {
    node = std::make_shared<play_launch_container::ObservableComponentManager>(exec);
  }

  // Handle thread_num parameter for MT mode
  if (use_multi_threaded && node->has_parameter("thread_num")) {
    const auto thread_num = node->get_parameter("thread_num").as_int();
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, thread_num);
    node->set_executor(exec);
  }

  exec->add_node(node);
  exec->spin();

  // Explicit cleanup order: remove the manager from the parent executor,
  // destroy it (which kills any isolated child processes), then shutdown.
  // Without this, auto-destruction order can trigger "Node needs to be
  // associated with an executor" during rclcpp::shutdown().
  exec->remove_node(node);
  node.reset();
  rclcpp::shutdown();
}
