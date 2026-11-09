#ifndef IO__ROS2__ROS_TIME_HPP
#define IO__ROS2__ROS_TIME_HPP

// =============================================================================
//  ros_time.hpp —— steady_clock 与 ROS 时间的一次性对齐（方案 §5.6）
//
//  相机取图时刻打的是 steady_clock（io::Camera::read 的 timestamp），而发布的
//  点云 header.stamp、以及导航侧按 stamp 做的插值/超时判断，用的都是 ROS 时间。
//  两者属于不同时钟域，必须标定一次偏移：
//
//      offset = ros_now - steady_now      （启动时标定一次）
//      stamp  = steady_t + offset         （之后每帧只做加法）
//
//  ⚠ 不要每帧重取 now() 相减：那等于把 DDS / 调度的抖动直接写进 stamp，
//    旋转时点会被甩出去（风险 R3）。
//
//  用法（同一进程内只需要调用一次 calibrate）：
//      io::rostime::calibrate(node->now());          // 或 calibrate(ros_ns)
//      auto stamp = io::rostime::to_ros(steady_ts);
// =============================================================================

#include <atomic>
#include <chrono>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

namespace io
{
namespace rostime
{

namespace detail
{
// 全进程共享一份偏移：统一由这里标定，任何模块 to_ros() 都得到同一个结果。
inline std::atomic<int64_t> & offset_ns_ref()
{
  static std::atomic<int64_t> value{0};
  return value;
}

inline std::atomic<bool> & calibrated_ref()
{
  static std::atomic<bool> value{false};
  return value;
}
}  // namespace detail

inline int64_t steady_ns(std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

// 标定：ros_ns 与 steady_now 必须是「同一瞬间」的两种表示。
inline void calibrate(int64_t ros_ns, std::chrono::steady_clock::time_point steady_now)
{
  detail::offset_ns_ref().store(ros_ns - steady_ns(steady_now), std::memory_order_relaxed);
  detail::calibrated_ref().store(true, std::memory_order_relaxed);
}

inline void calibrate(const rclcpp::Time & ros_now)
{
  calibrate(ros_now.nanoseconds(), std::chrono::steady_clock::now());
}

inline void calibrate(rclcpp::Clock & clock) { calibrate(clock.now()); }

inline bool calibrated() { return detail::calibrated_ref().load(std::memory_order_relaxed); }

inline int64_t offset_ns() { return detail::offset_ns_ref().load(std::memory_order_relaxed); }

// steady_clock 时刻 -> ROS 时间戳（未标定时等于「假装同一时钟域」，仅用于自测）
inline rclcpp::Time to_ros(std::chrono::steady_clock::time_point t)
{
  return rclcpp::Time(steady_ns(t) + offset_ns(), RCL_ROS_TIME);
}

// 标定残差监测（§5.6）：返回 |stamp - (ros_now - 处理耗时)| 毫秒，调用方自定阈值。
inline double residual_ms(
  const rclcpp::Time & ros_now, std::chrono::steady_clock::time_point steady_now)
{
  return static_cast<double>(ros_now.nanoseconds() - steady_ns(steady_now) - offset_ns()) / 1e6;
}

}  // namespace rostime
}  // namespace io

#endif  // IO__ROS2__ROS_TIME_HPP
