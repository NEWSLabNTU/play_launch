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
#include <cstdio>

#include "common.hpp"

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("obstacle_detector")
  {
    burn_ms_ = this->declare_parameter<double>("burn_ms", 8.0);
    burn_jitter_ms_ = this->declare_parameter<double>("burn_jitter_ms", 0.0);
    // Phase 71: the detector. `watchdog_ms` is the contract's
    // `lease_duration` on this subscription, applied by the node itself
    // (mechanism: application). On expiry the `declare_lost` reaction
    // publishes an "unknown obstacles" message once, so the brake reacts.
    watchdog_ms_ = this->declare_parameter<double>("watchdog_ms", 0.0);
    if (watchdog_ms_ > 0.0) {
      watchdog_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        [this]() {
          if (!seen_scan_ || lost_) {return;}
          const double since_ms = (this->now() - last_scan_).nanoseconds() / 1.0e6;
          if (since_ms >= watchdog_ms_) {
            lost_ = true;
            std::printf("SCAN_LOST after_ms=%.1f\n", since_ms);
            std::fflush(stdout);
            pub_->publish(rt_av_demo::make_msg(this->now(), 0, "lost"));
          }
        });
    }

    pub_ = this->create_publisher<rt_av_demo::msg::Stamped>("obstacles", 10);
    sub_ = this->create_subscription<rt_av_demo::msg::Stamped>(
      "scan", 10,
      [this](const rt_av_demo::msg::Stamped::SharedPtr in) {
        last_scan_ = this->now();
        seen_scan_ = true;
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
  double watchdog_ms_;
  uint64_t seq_{0};
  rclcpp::Time last_scan_ {0, 0, RCL_ROS_TIME};
  bool seen_scan_ {false}, lost_ {false};
  rclcpp::TimerBase::SharedPtr watchdog_;
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
