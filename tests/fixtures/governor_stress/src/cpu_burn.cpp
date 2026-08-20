// A node that is busy for a configurable time during construction, then goes
// idle — the shape the startup governor's "settle" detection is written for.
//
// The governor holds a process's admission slot until its CPU usage drops
// below `settle_threshold_pct` (20% of one core) for `settle_samples`
// consecutive polls, capped by `max_settle_secs`. That distinction only
// matters for a node that is expensive to START and cheap to RUN, which is
// exactly a TensorRT engine build and exactly what this imitates.
//
// `burn_ms` is spent in the CONSTRUCTOR on purpose: a node that burns in a
// timer callback would already be "started" as far as admission is concerned,
// and would test nothing.
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"

class CpuBurn : public rclcpp::Node
{
public:
  CpuBurn()
  : Node("cpu_burn")
  {
    const int64_t burn_ms = declare_parameter("burn_ms", 5000);
    const int64_t idle_hz = declare_parameter("idle_hz", 1);
    // Sustained load AFTER construction, as a duty cycle. 0 (the default)
    // keeps the original behaviour: burn once, then idle.
    //
    // This exists to settle a disagreement. W1 measured the runnable-task
    // ceiling as WORSE than no gate at all, on the reasoning that it cannot
    // tell "busy starting things" from "busy running the things we started" —
    // so already-running nodes hold the gate shut. W3 measured it as merely
    // costly, because a node that burns once and idles lets the runnable
    // count fall back on its own and the gate never gets stuck. Neither
    // result is wrong; they are different workloads. A node that stays busy
    // is the one W1 was describing, and it cannot be imitated by a node that
    // stops.
    const int64_t sustain_duty_pct = declare_parameter("sustain_duty_pct", 0);
    const int64_t sustain_period_ms = declare_parameter("sustain_period_ms", 100);
    // Hard deadline, same reasoning as mem_hog: a burner that outlives its
    // harness is a core held hostage.
    const int64_t max_lifetime_secs = declare_parameter("max_lifetime_secs", 180);
    // Refuse an absurd burn outright rather than trusting the caller.
    const int64_t burn_capped = std::min<int64_t>(burn_ms, 120000);

    const auto t0 = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(), "cpu_burn '%s' constructing: burn_ms=%ld pid=%d", get_name(),
      static_cast<long>(burn_ms), getpid());

    // Real work, not sleep: the governor samples CPU time from /proc, so a
    // sleeping node reads as already settled and would be admitted straight
    // through.
    volatile double acc = 0.0;
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count() < burn_capped)
    {
      for (int i = 1; i < 20000; ++i) {
        acc += std::sqrt(static_cast<double>(i)) / std::log(static_cast<double>(i) + 1.0);
      }
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(
      get_logger(), "cpu_burn '%s' settled after %ldms (acc %.1f)", get_name(),
      static_cast<long>(ms), static_cast<double>(acc));

    if (sustain_duty_pct > 0) {
      // Burn `duty`% of every period, forever. The point is to keep this
      // process RUNNABLE, so the work has to be real and it has to continue
      // after the node is "started" — a sleeping node contributes nothing to
      // the runnable count the gate reads from /proc.
      const int64_t burn_slice_ms =
        std::max<int64_t>(1, sustain_period_ms * std::min<int64_t>(sustain_duty_pct, 100) / 100);
      RCLCPP_INFO(
        get_logger(), "cpu_burn '%s' sustaining %ld%% duty (%ldms per %ldms)", get_name(),
        static_cast<long>(sustain_duty_pct), static_cast<long>(burn_slice_ms),
        static_cast<long>(sustain_period_ms));
      timer_ = create_wall_timer(
        std::chrono::milliseconds(sustain_period_ms), [this, burn_slice_ms]() {
          const auto s0 = std::chrono::steady_clock::now();
          volatile double a = 0.0;
          while (std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - s0).count() < burn_slice_ms)
          {
            for (int i = 1; i < 5000; ++i) {
              a += std::sqrt(static_cast<double>(i));
            }
          }
          tick_++;
        });
    } else {
      timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / std::max<int64_t>(idle_hz, 1)),
        [this]() {tick_++;});
    }

    deadline_ = create_wall_timer(
      std::chrono::seconds(max_lifetime_secs), [this, max_lifetime_secs]() {
        RCLCPP_WARN(
          get_logger(), "cpu_burn '%s' hit max_lifetime_secs=%ld — exiting", get_name(),
          static_cast<long>(max_lifetime_secs));
        rclcpp::shutdown();
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr deadline_;
  uint64_t tick_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CpuBurn>());
  rclcpp::shutdown();
  return 0;
}
