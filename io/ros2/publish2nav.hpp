#ifndef IO__PBLISH2NAV_HPP
#define IO__PBLISH2NAV_HPP

#include <Eigen/Dense>  // For Eigen::Vector3d
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace io
{
class Publish2Nav : public rclcpp::Node
{
public:
  Publish2Nav();

  ~Publish2Nav();

  void start();

  void send_data(const Eigen::Vector4d & data);

  // 云台关节反馈：每收到一帧新的 0x502 调用一次，header.stamp 取调用时刻的 ROS 时间。
  // yaw_rad / pitch_rad 均为弧度，pitch 无自由度时由调用方传 0。
  void publish_gimbal_joint_state(double yaw_rad, double pitch_rad);

private:
  // ROS2 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // 云台关节状态（供导航侧构建 TF 用）
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  // 0x502 实际可能以 500Hz 上报，按导航侧建议限到 100Hz：
  // 只抽帧发送最新采样，绝不重发旧角度。仅由 SerialBoard 接收线程调用。
  static constexpr double kMaxJointStateHz = 100.0;
  bool joint_state_published_ = false;
  std::chrono::steady_clock::time_point last_joint_state_time_;
};

}  // namespace io

#endif  // Publish2Nav_HPP_
