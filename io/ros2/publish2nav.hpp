#ifndef IO__PBLISH2NAV_HPP
#define IO__PBLISH2NAV_HPP

#include <Eigen/Dense>  // For Eigen::Vector3d
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "io/command.hpp"

namespace io
{
/*
 * 视觉 -> 导航侧的统一发布节点，复用同一个节点与同一条 spin 线程发布三类数据：
 *   1) 自瞄目标位置（std_msgs/String，历史接口）
 *   2) 云台关节反馈（sensor_msgs/JointState，供导航侧构建 TF）
 *   3) 裁判状态（std_msgs 三个标量话题，供导航决策节点判开赛 / 血量返航）
 */
class Publish2Nav : public rclcpp::Node
{
public:
  /*
   * 裁判状态数据源：返回当前解析结果（通常是 SerialBoard::referee_state 的包装）。
   * 采用「定时拉取」而不是「串口收到帧就回调发布」：裁判帧只有 1~5 Hz，
   * 固定频率发布对迟加入的订阅方更友好，串口接收线程也不必进入 ROS。
   */
  using RefereeSource = std::function<RefereeState()>;

  Publish2Nav();

  ~Publish2Nav();

  void start();

  void send_data(const Eigen::Vector4d & data);

  // 云台关节反馈：每收到一帧新的 0x502 调用一次，header.stamp 取调用时刻的 ROS 时间。
  // yaw_rad / pitch_rad 均为弧度，pitch 无自由度时由调用方传 0。
  void publish_gimbal_joint_state(double yaw_rad, double pitch_rad);

  /*
   * 裁判状态转发：设置数据源后，节点按 5 Hz 拉取并发布
   *   /red_standard_robot1/referee/game_progress  std_msgs/msg/UInt8  <- 0x0001
   *   /red_standard_robot1/referee/current_hp     std_msgs/msg/Int32  <- 0x0201
   *   /red_standard_robot1/referee/maximum_hp     std_msgs/msg/Int32  <- 0x0201
   * 话题名带命名空间是刻意的：导航决策节点在 red_standard_robot1 下用相对名
   * referee/xxx 订阅，换命名空间时这里必须同步修改。
   *
   * 只允许设置一次：重复调用会告警并忽略。source 为空时不发布任何裁判话题。
   */
  void set_referee_source(RefereeSource source);

private:
  // 裁判状态定时器回调：拉取快照 -> 按有效位发布 -> 变化时打日志
  void publish_referee();

  // ROS2 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // 云台关节状态（供导航侧构建 TF 用）
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  // 裁判状态（供导航决策节点消费）
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_progress_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_hp_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr maximum_hp_publisher_;

  // 数据源可能由主线程设置、由 spin 线程读取，用锁保护 std::function 的读写
  mutable std::mutex referee_source_mutex_;
  RefereeSource referee_source_;
  rclcpp::TimerBase::SharedPtr referee_timer_;

  // 仅用于抑制重复日志：话题本身每拍都发，不依赖值的跳变
  bool referee_has_last_ = false;
  uint8_t referee_last_game_progress_ = 0;
  int32_t referee_last_current_hp_ = 0;
  int32_t referee_last_maximum_hp_ = 0;
  // 长时间收不到帧 / 下位机血量异常时各告警一次
  int64_t referee_ticks_ = 0;
  bool referee_empty_warned_ = false;
  bool referee_bad_max_hp_warned_ = false;

  // 0x502 实际可能以 500Hz 上报，按导航侧建议限到 100Hz：
  // 只抽帧发送最新采样，绝不重发旧角度。仅由 SerialBoard 接收线程调用。
  static constexpr double kMaxJointStateHz = 100.0;
  bool joint_state_published_ = false;
  std::chrono::steady_clock::time_point last_joint_state_time_;
  static constexpr double kRefereePublishHz = 5.0;
};

}  // namespace io

#endif  // Publish2Nav_HPP_
