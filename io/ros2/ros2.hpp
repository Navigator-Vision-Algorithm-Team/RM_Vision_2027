#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include <functional>

#include "command.hpp"
#include "nav_pose_buffer.hpp"
#include "publish2nav.hpp"
#include "subscribe2nav.hpp"

namespace io
{
class ROS2
{
public:
  // nav_sender: 导航数据发送回调（如串口下发）。
  // 传入后 Subscribe2Nav 内部会启动独立线程"收包即发送"，与主循环解耦。
  // subscribe_nav=false 时只创建发布侧（如仅需发布云台关节反馈的入口）。
  // config_path: 用于读取 nav_pose 段；为空 / 缺该段时不建位姿订阅（视觉侧只发 PnP 点）。
  explicit ROS2(
    std::function<void(const NavCommand &)> nav_sender = nullptr, bool subscribe_nav = true,
    const std::string & config_path = "");

  ~ROS2();

  void publish(const Eigen::Vector4d & target_pos);

  // 云台关节反馈回调，交给 io::SerialBoard::set_gimbal_joint_callback 使用。
  // 回调内部持有发布节点的 shared_ptr，无需额外线程，也不会再开一份串口。
  std::function<void(double yaw_rad, double pitch_rad)> gimbal_joint_sender();

  // std::vector<int8_t> subscribe_enemy_status();

  // std::vector<int8_t> subscribe_autoaim_target();

  NavCommand subscribe_get_data();

  // 导航位姿缓冲（C1）。返回 nullptr 表示配置里没有 nav_pose 段或初始化失败，
  // 调用方应把三角化通道标为 degraded，只发 PnP 点（§5.8）。
  io::NavPoseBuffer * nav_pose() const { return nav_pose_.get(); }

  template <typename T>
  std::shared_ptr<rclcpp::Publisher<T>> create_publisher(
    const std::string & node_name, const std::string & topic_name, size_t queue_size)
  {
    auto node = std::make_shared<rclcpp::Node>(node_name);

    auto publisher = node->create_publisher<T>(topic_name, queue_size);

    // 运行一个单独的线程来 spin 这个节点，确保消息可以被正确发布
    std::thread([node]() { rclcpp::spin(node); }).detach();

    return publisher;
  }

private:
  std::shared_ptr<Publish2Nav> publish2nav_;
  std::shared_ptr<Subscribe2Nav> subscribe2nav_;
  // 导航 -> 视觉的位姿（双向通信 C1），配置里没有 nav_pose 段时为空
  std::unique_ptr<NavPoseBuffer> nav_pose_;

  std::unique_ptr<std::thread> publish_spin_thread_;
  std::unique_ptr<std::thread> subscribe_spin_thread_;
};

}  // namespace io
#endif