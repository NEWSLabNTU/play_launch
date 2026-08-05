// /safety/brake_controller — chain sink. criticality: safety.
//
// This is the measurement point. It compares the ORIGINAL lidar stamp against
// arrival time, so what it reports is true end-to-end chain latency, not the
// cost of its own hop.
//
// Latencies go to STDOUT in a machine-readable line, which play_launch
// captures to play_log/<ts>/node/brake_controller/out. The Chrome trace is
// for looking at; these numbers are what the A/B harness actually compares.
// Keeping the measurement independent of the trace matters — if the exporter
// were wrong, a harness reading only the trace would be wrong with it.
#include <cstdio>

#include "common.hpp"

class BrakeController : public rclcpp::Node
{
public:
  BrakeController()
  : Node("brake_controller")
  {
    burn_ms_ = this->declare_parameter<double>("burn_ms", 3.0);
    deadline_ms_ = this->declare_parameter<double>("deadline_ms", 60.0);

    pub_ = this->create_publisher<rt_av_demo::msg::Stamped>("brake_cmd", 10);
    sub_ = this->create_subscription<rt_av_demo::msg::Stamped>(
      "obstacles", 10,
      [this](const rt_av_demo::msg::Stamped::SharedPtr in) {
        rt_av_demo::burn_ms(burn_ms_);

        const double latency_ms =
          (this->now() - rclcpp::Time(in->header.stamp)).nanoseconds() / 1.0e6;
        const bool late = latency_ms > deadline_ms_;
        ++total_;
        if (late) {++missed_;}

        // Unbuffered and flushed: a run killed by the harness must not lose
        // its tail, or the miss count silently under-reports.
        std::printf(
          "CHAIN seq=%u latency_ms=%.3f deadline_ms=%.1f late=%d total=%lu missed=%lu\n",
          in->seq, latency_ms, deadline_ms_, late ? 1 : 0,
          static_cast<unsigned long>(total_), static_cast<unsigned long>(missed_));
        std::fflush(stdout);

        pub_->publish(rt_av_demo::make_msg(in->header.stamp, in->seq, "brake"));
      });
  }

private:
  double burn_ms_, deadline_ms_;
  size_t total_ {0}, missed_ {0};
  rclcpp::Publisher<rt_av_demo::msg::Stamped>::SharedPtr pub_;
  rclcpp::Subscription<rt_av_demo::msg::Stamped>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrakeController>());
  rclcpp::shutdown();
  return 0;
}
