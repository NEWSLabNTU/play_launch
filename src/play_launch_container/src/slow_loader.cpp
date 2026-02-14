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

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace play_launch_container
{

/// Test component that sleeps during construction to simulate slow loading.
/// Used for testing non-blocking parallel load behavior.
class SlowLoader : public rclcpp::Node
{
public:
  explicit SlowLoader(const rclcpp::NodeOptions & options) : Node("slow_loader", options)
  {
    auto delay_ms = declare_parameter("load_delay_ms", 3000);
    RCLCPP_INFO(get_logger(), "SlowLoader: sleeping for %ld ms", delay_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    RCLCPP_INFO(get_logger(), "SlowLoader: construction complete");
  }
};

}  // namespace play_launch_container

RCLCPP_COMPONENTS_REGISTER_NODE(play_launch_container::SlowLoader)
