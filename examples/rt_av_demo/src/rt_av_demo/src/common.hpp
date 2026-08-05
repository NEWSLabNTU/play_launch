// Shared helpers for the mixed-criticality demo nodes.
//
// `burn_ms` spins on the CPU rather than sleeping, and that distinction is the
// whole experiment: a sleeping node yields, so the scheduler never has to
// choose between it and anything else. Only runnable work creates the
// contention that makes priority observable.
#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rt_av_demo/msg/stamped.hpp"

namespace rt_av_demo
{

/// Occupy a CPU for approximately `ms` milliseconds.
///
/// Deliberately a busy loop with a volatile accumulator so the optimiser
/// cannot elide it. Uses steady_clock, so it burns wall-clock time whether or
/// not this thread is actually scheduled — meaning a preempted node takes
/// LONGER than `ms` to finish, which is precisely the effect under test.
inline void burn_ms(double ms)
{
  if (ms <= 0.0) {return;}
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::microseconds(static_cast<long>(ms * 1000.0));
  volatile double sink = 0.0;
  while (std::chrono::steady_clock::now() < deadline) {
    for (int i = 0; i < 1000; ++i) {sink += i * 0.5;}
  }
  (void)sink;
}

/// Stamp a message with the current ROS time.
///
/// Every hop copies the ORIGINAL stamp forward rather than re-stamping, so
/// the final consumer can measure true end-to-end chain latency and the trace
/// exporter can link all three hops into one flow.
inline msg::Stamped make_msg(const rclcpp::Time & stamp, uint32_t seq, const std::string & payload)
{
  msg::Stamped m;
  m.header.stamp = stamp;
  m.seq = seq;
  m.payload = payload;
  return m;
}

}  // namespace rt_av_demo
