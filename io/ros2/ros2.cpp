#include "ros2.hpp"
namespace io
{
ROS2::ROS2(std::function<void(const NavCommand &)> nav_sender, bool subscribe_nav)
{
  rclcpp::init(0, nullptr);

  publish2nav_ = std::make_shared<Publish2Nav>();

  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });

  if (subscribe_nav) {
    subscribe2nav_ = std::make_shared<Subscribe2Nav>(std::move(nav_sender));

    subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });
  }
}

ROS2::~ROS2()
{
  rclcpp::shutdown();
  if (publish_spin_thread_ && publish_spin_thread_->joinable()) publish_spin_thread_->join();
  if (subscribe_spin_thread_ && subscribe_spin_thread_->joinable()) subscribe_spin_thread_->join();
}

void ROS2::publish(const Eigen::Vector4d & target_pos) { publish2nav_->send_data(target_pos); }

std::function<void(double, double)> ROS2::gimbal_joint_sender()
{
  auto node = publish2nav_;
  return [node](double yaw_rad, double pitch_rad) {
    // 本对象析构时 rclcpp 已 shutdown，串口接收线程可能还在收尾，
    // 此时直接跳过（否则 publish 会抛异常）。
    if (!rclcpp::ok()) return;
    node->publish_gimbal_joint_state(yaw_rad, pitch_rad);
  };
}

// std::vector<int8_t> ROS2::subscribe_enemy_status()
// {
//   return subscribe2nav_->subscribe_enemy_status();
// }

// std::vector<int8_t> ROS2::subscribe_autoaim_target()
// {
//   return subscribe2nav_->subscribe_autoaim_target();
// }

NavCommand ROS2::subscribe_get_data(){
  if (!subscribe2nav_) return NavCommand();  // subscribe_nav=false 时没有订阅侧
  return subscribe2nav_->subscribe_move_info();
}

}  // namespace io
