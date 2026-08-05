// One binary for all three non-safety loads — path_planner, map_loader and
// telemetry_logger differ only in rate and burn, so they are parameters
// rather than three near-identical files.
//
// These exist to CONTEND. On a 32-core machine the safety chain would never
// miss a deadline on its own; the experiment only means something because
// this work competes for the same confined CPU set.
#include "common.hpp"

class LoadNode : public rclcpp::Node
{
public:
  LoadNode()
  : Node("load_node")
  {
    rate_hz_ = this->declare_parameter<double>("rate_hz", 10.0);
    burn_ms_ = this->declare_parameter<double>("burn_ms", 40.0);
    const auto topic = this->declare_parameter<std::string>("out_topic", "load_out");

    pub_ = this->create_publisher<rt_av_demo::msg::Stamped>(topic, 10);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_),
      [this]() {
        const auto stamp = this->now();
        rt_av_demo::burn_ms(burn_ms_);
        pub_->publish(rt_av_demo::make_msg(stamp, seq_++, "load"));
      });
  }

private:
  double rate_hz_, burn_ms_;
  uint32_t seq_ {0};
  rclcpp::Publisher<rt_av_demo::msg::Stamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoadNode>());
  rclcpp::shutdown();
  return 0;
}
