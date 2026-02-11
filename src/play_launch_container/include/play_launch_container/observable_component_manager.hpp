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

#ifndef PLAY_LAUNCH_CONTAINER__OBSERVABLE_COMPONENT_MANAGER_HPP_
#define PLAY_LAUNCH_CONTAINER__OBSERVABLE_COMPONENT_MANAGER_HPP_

#include <memory>
#include <string>

#include "play_launch_msgs/msg/component_event.hpp"
#include "rclcpp_components/component_manager.hpp"

namespace play_launch_container
{

/// Component manager that publishes load/unload events on a topic.
///
/// Subclasses rclcpp_components::ComponentManager (the same pattern used by
/// ComponentManagerIsolated in rclcpp_components) and overrides on_load_node
/// and on_unload_node to publish ComponentEvent messages after each operation.
///
/// Topic: ~/_container/component_events
/// QoS:   reliable, transient_local, depth 100
class ObservableComponentManager : public rclcpp_components::ComponentManager
{
public:
  ObservableComponentManager(
    std::weak_ptr<rclcpp::Executor> executor =
      std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
    std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & node_options =
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));

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
