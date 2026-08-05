// /safety/obstacle_detector — middle hop. criticality: safety.
//
// Bounded work on every scan. Forwards the ORIGINAL stamp so latency
// accumulates across the chain rather than resetting at each hop.
#include "common.hpp"

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("obstacle_detector")
  {
    burn_ms_ = this->declare_parameter<double>("burn_ms", 8.0);

    pub_ = this->create_publisher<rt_av_demo::msg::Stamped>("obstacles", 10);
    sub_ = this->create_subscription<rt_av_demo::msg::Stamped>(
      "scan", 10,
      [this](const rt_av_demo::msg::Stamped::SharedPtr in) {
        rt_av_demo::burn_ms(burn_ms_);
        pub_->publish(rt_av_demo::make_msg(in->header.stamp, in->seq, "obstacles"));
      });
  }

private:
  double burn_ms_;
  rclcpp::Publisher<rt_av_demo::msg::Stamped>::SharedPtr pub_;
  rclcpp::Subscription<rt_av_demo::msg::Stamped>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
