#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include <functional>

#include "publish2nav.hpp"
#include "subscribe2nav.hpp"
#include "command.hpp"

namespace io
{
class ROS2
{
public:
  // nav_sender: 导航数据发送回调（如串口下发）。
  // 传入后 Subscribe2Nav 内部会启动独立线程"收包即发送"，与主循环解耦。
  // subscribe_nav=false 时只创建发布侧（如仅需发布云台关节反馈的入口）。
  explicit ROS2(
    std::function<void(const NavCommand &)> nav_sender = nullptr, bool subscribe_nav = true);

  ~ROS2();

  void publish(const Eigen::Vector4d & target_pos);

  // 云台关节反馈回调，交给 io::SerialBoard::set_gimbal_joint_callback 使用。
  // 回调内部持有发布节点的 shared_ptr，无需额外线程，也不会再开一份串口。
  std::function<void(double yaw_rad, double pitch_rad)> gimbal_joint_sender();

  /*
   * 启动裁判状态转发：导航决策节点订阅的 referee/{game_progress,current_hp,maximum_hp}
   * 由已有的 Publish2Nav 节点按 5 Hz 发布，不额外起节点和线程。
   * source 通常是 SerialBoard::referee_state 的包装，因此 SerialBoard 必须比本对象
   * 活得久（先构造、后析构）。
   *
   * 只允许设置一次：重复调用直接忽略并告警。source 为空时不会发布任何裁判话题。
   */
  void set_referee_source(Publish2Nav::RefereeSource source);

  // std::vector<int8_t> subscribe_enemy_status();

  // std::vector<int8_t> subscribe_autoaim_target();

  NavCommand subscribe_get_data();

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

  std::unique_ptr<std::thread> publish_spin_thread_;
  std::unique_ptr<std::thread> subscribe_spin_thread_;
};

}  // namespace io
#endif