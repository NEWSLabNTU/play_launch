// /safety/obstacle_detector — middle hop. criticality: safety.
//
// Bounded work on every scan. Forwards the ORIGINAL stamp so latency
// accumulates across the chain rather than resetting at each hop.
//
// `burn_jitter_ms` (default 0) makes this node a JITTER oracle the way
// `burn_ms` already makes it a cost oracle: successive invocations alternate
// between `burn_ms - j` and `burn_ms + j`, so the true spread of the path's
// latency is 2j by construction. With a fixed burn the spread is ~0 and no
// measurement of it can tell a correct jitter rule from a vacuous one.
#include "common.hpp"

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("obstacle_detector")
  {
    burn_ms_ = this->declare_parameter<double>("burn_ms", 8.0);
    burn_jitter_ms_ = this->declare_parameter<double>("burn_jitter_ms", 0.0);

    pub_ = this->create_publisher<rt_av_demo::msg::Stamped>("obstacles", 10);
    sub_ = this->create_subscription<rt_av_demo::msg::Stamped>(
      "scan", 10,
      [this](const rt_av_demo::msg::Stamped::SharedPtr in) {
        // Deterministic alternation, not random: the oracle's answer has to
        // be exact, and a random draw would make it a distribution.
        const double sign = (seq_++ % 2 == 0) ? -1.0 : 1.0;
        rt_av_demo::burn_ms(burn_ms_ + sign * burn_jitter_ms_);
        pub_->publish(rt_av_demo::make_msg(in->header.stamp, in->seq, "obstacles"));
      });
  }

private:
  double burn_ms_;
  double burn_jitter_ms_;
  uint64_t seq_{0};
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
