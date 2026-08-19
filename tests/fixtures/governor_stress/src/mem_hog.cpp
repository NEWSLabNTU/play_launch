// A node that occupies a known amount of RAM, to exercise play_launch's
// memory-governed startup admission.
//
// The memory is TOUCHED, not merely allocated. `malloc` of 2 GiB moves
// MemAvailable by almost nothing on Linux — the mapping is not backed until
// it is written — so a hog that only allocates would leave the gate it is
// meant to test permanently open, and the run would "pass" while proving
// nothing. Every page is written, then re-read at the end so the compiler
// cannot elide the store.
//
// It reports its own timing on stdout, because the thing under test is WHEN
// play_launch let this process start, not whether it eventually ran.
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <fstream>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

// MemAvailable in kB, or -1 if unreadable.
static int64_t mem_available_kb()
{
  std::ifstream f("/proc/meminfo");
  std::string key;
  int64_t value = 0;
  std::string unit;
  while (f >> key >> value >> unit) {
    if (key == "MemAvailable:") {
      return value;
    }
  }
  return -1;
}


class MemHog : public rclcpp::Node
{
public:
  MemHog()
  : Node("mem_hog")
  {
    const int64_t alloc_mb = declare_parameter("alloc_mb", 512);
    const int64_t hold_secs = declare_parameter("hold_secs", 60);
    const int64_t ramp_ms = declare_parameter("ramp_ms", 0);
    // Never take the machine below this, whatever alloc_mb says.
    const int64_t safety_reserve_mb = declare_parameter("safety_reserve_mb", 8192);
    // Hard deadline. Independent of hold_secs and of any shutdown path: an
    // orphaned hog releases and exits on its own rather than holding RAM until
    // someone notices.
    const int64_t max_lifetime_secs = declare_parameter("max_lifetime_secs", 180);

    const auto t0 = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(), "mem_hog '%s' starting: alloc_mb=%ld hold_secs=%ld pid=%d",
      get_name(), static_cast<long>(alloc_mb), static_cast<long>(hold_secs), getpid());

    // Touch in 1 MiB chunks so a partially-admitted hog still moves
    // MemAvailable monotonically — the watcher correlates admissions against
    // that curve, and a single bulk write would make it a step function.
    //
    // SAFETY: re-read MemAvailable every chunk and stop short if the machine
    // would drop below `safety_reserve_mb`. This is the innermost of the
    // test's guards and the only one that holds when a hog is started by hand
    // outside the harness. It is deliberately per-process and racy: 24 hogs
    // each watching one shared number will collectively overshoot by up to a
    // chunk each, which is why the reserve is GiB-scale rather than tight.
    //
    // Stopping short is a legitimate outcome, not a failure — it is logged so
    // a short hog is never mistaken for a gate that admitted it early.
    const size_t chunk = 1024 * 1024;
    block_.reserve(static_cast<size_t>(alloc_mb));
    for (int64_t i = 0; i < alloc_mb; ++i) {
      const int64_t avail_mb = mem_available_kb() / 1024;
      if (avail_mb >= 0 && avail_mb - 1 < safety_reserve_mb) {
        RCLCPP_WARN(
          get_logger(),
          "mem_hog '%s' STOPPING SHORT at %ld/%ld MiB: MemAvailable %ld MiB would fall below "
          "safety_reserve_mb=%ld", get_name(), static_cast<long>(i), static_cast<long>(alloc_mb),
          static_cast<long>(avail_mb), static_cast<long>(safety_reserve_mb));
        break;
      }
      auto page = std::vector<uint8_t>(chunk);
      std::memset(page.data(), static_cast<int>(i & 0xff), chunk);
      block_.push_back(std::move(page));
      if (ramp_ms > 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(ramp_ms));
      }
    }

    // Re-read so the writes cannot be optimised away.
    uint64_t sum = 0;
    for (const auto & page : block_) {
      sum += page[0];
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(
      get_logger(), "mem_hog '%s' resident after %ldms (checksum %lu)", get_name(),
      static_cast<long>(ms), static_cast<unsigned long>(sum));

    timer_ = create_wall_timer(
      std::chrono::seconds(hold_secs), [this]() {
        RCLCPP_INFO(get_logger(), "mem_hog '%s' releasing", get_name());
        block_.clear();
        timer_->cancel();
      });

    // The kill switch of last resort. Fires even if hold_secs is huge, the
    // executor is wedged, or the parent died without reaping us.
    deadline_ = create_wall_timer(
      std::chrono::seconds(max_lifetime_secs), [this, max_lifetime_secs]() {
        RCLCPP_WARN(
          get_logger(), "mem_hog '%s' hit max_lifetime_secs=%ld — releasing and exiting",
          get_name(), static_cast<long>(max_lifetime_secs));
        block_.clear();
        rclcpp::shutdown();
      });
  }

private:
  std::vector<std::vector<uint8_t>> block_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr deadline_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MemHog>());
  rclcpp::shutdown();
  return 0;
}
