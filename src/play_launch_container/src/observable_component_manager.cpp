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

#include "play_launch_container/observable_component_manager.hpp"

namespace play_launch_container
{

ObservableComponentManager::ObservableComponentManager(
  std::weak_ptr<rclcpp::Executor> executor, std::string node_name,
  const rclcpp::NodeOptions & node_options)
: ComponentManager(executor, std::move(node_name), node_options)
{
  // Transient local: late subscribers (e.g. play_launch restarts) get history
  auto qos = rclcpp::QoS(100).reliable().transient_local();
  event_pub_ =
    create_publisher<play_launch_msgs::msg::ComponentEvent>("~/_container/component_events", qos);
}

void ObservableComponentManager::on_load_node(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<LoadNode::Request> request, std::shared_ptr<LoadNode::Response> response)
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
