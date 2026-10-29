#include "publish2nav.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <thread>

#include "tools/logger.hpp"

namespace io
{

Publish2Nav::Publish2Nav() : Node("auto_aim_target_pos_publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("auto_aim_target_pos", 10);

  // 导航侧用这条关节反馈驱动模拟 JointState 的 TF；
  // 话题/字段与导航侧约定一致，节点名沿用现有节点，避免额外起一个 spin 线程。
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/red_standard_robot1/serial/gimbal_joint_state", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "auto_aim_target_pos_publisher node initialized (joint state -> "
    "/red_standard_robot1/serial/gimbal_joint_state).");
}

Publish2Nav::~Publish2Nav()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node shutting down.");
}

void Publish2Nav::send_data(const Eigen::Vector4d & target_pos)
{
  // 创建消息
  auto message = std::make_shared<std_msgs::msg::String>();

  // 将 Eigen::Vector3d 数据转换为字符串并存储在消息中
  message->data = std::to_string(target_pos[0]) + "," + std::to_string(target_pos[1]) + "," +
                  std::to_string(target_pos[2]) + "," + std::to_string(target_pos[3]);

  // 发布消息
  publisher_->publish(*message);

  // RCLCPP_INFO(
  //   this->get_logger(), "auto_aim_target_pos_publisher node sent message: '%s'",
  //   message->data.c_str());
}

void Publish2Nav::publish_gimbal_joint_state(double yaw_rad, double pitch_rad)
{
  // 节流到 100Hz 上限：间隔不足时直接丢弃这一帧新采样（不是重发旧值），
  // 因此话题上出现的每个角度都对应一帧真实 0x502，且 stamp 是最新采样的 ROS 时间。
  const auto now = std::chrono::steady_clock::now();
  if (joint_state_published_ &&
      std::chrono::duration<double>(now - last_joint_state_time_).count() <
        1.0 / kMaxJointStateHz) {
    return;
  }
  joint_state_published_ = true;
  last_joint_state_time_ = now;

  sensor_msgs::msg::JointState message;

  message.header.stamp = this->now();
  message.name = {"gimbal_yaw_joint", "gimbal_pitch_joint"};
  message.position = {yaw_rad, pitch_rad};
  // velocity / effort 无反馈，保持为空

  joint_state_publisher_->publish(message);
}

void Publish2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io
