// control_node: standalone node, subscribes "points_filtered" published by
// filter_component (cross-namespace, hence the absolute topic name below —
// Phase 44.5: the `points_to_cmd` chain's second hop, sensor_node ->
// filter_component -> control_node, matches bringup.contract.yaml exactly)
// and publishes "cmd" (relative — resolves under this node's own
// namespace, e.g. /control/cmd).
//
// This is the node system.toml's [tiers.control] pins to SCHED_FIFO
// priority 20 / CPU core 0 — see docs/guide/rt-scheduling.md.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control_node")
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("cmd", 10);

    // Absolute: filter_component lives under a different namespace (/perception).
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/perception/points_filtered", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        auto out = std_msgs::msg::String();
        out.data = "cmd_for:" + msg->data;
        pub_->publish(out);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
