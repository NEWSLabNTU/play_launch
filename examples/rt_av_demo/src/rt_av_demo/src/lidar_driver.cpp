// /safety/lidar_driver — head of the safety chain. 50 Hz, criticality: safety.
//
// Stamps each scan once; every downstream hop carries that stamp forward, so
// the brake controller measures true end-to-end latency and the Chrome trace
// links all three hops into a single flow.
#include "common.hpp"

using namespace std::chrono_literals;

class LidarDriver : public rclcpp::Node
{
public:
  LidarDriver()
  : Node("lidar_driver")
  {
    rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);
    burn_ms_ = this->declare_parameter<double>("burn_ms", 2.0);

    pub_ = this->create_publisher<rt_av_demo::msg::Stamped>("scan", 10);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_),
      [this]() {
        // Stamp BEFORE the work: the deadline is measured from when the
        // sample was notionally taken, not from when we finished with it.
        const auto stamp = this->now();
        rt_av_demo::burn_ms(burn_ms_);
        pub_->publish(rt_av_demo::make_msg(stamp, seq_++, "scan"));
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
  rclcpp::spin(std::make_shared<LidarDriver>());
  rclcpp::shutdown();
  return 0;
}
