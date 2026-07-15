// sensor_node: standalone node, 100 Hz wall-timer publisher on "points_raw"
// (relative — resolves under whatever namespace the launch file gives it,
// e.g. /perception/points_raw). Payload is a std_msgs/String stand-in; the
// contract sidecar documents the intended rate (rate_hz: 100).
//
// This is the "sensor" leg of the rt_workspace example/fixture — see
// docs/guide/rt-scheduling.md and tests/fixtures/rt_workspace/README.md.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SensorNode : public rclcpp::Node
{
public:
  SensorNode()
  : Node("sensor_node"), count_(0)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("points_raw", 10);

    // 100 Hz
    timer_ = this->create_wall_timer(10ms, std::bind(&SensorNode::tick, this));
  }

private:
  void tick()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "points_raw#" + std::to_string(count_++);
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorNode>());
  rclcpp::shutdown();
  return 0;
}
