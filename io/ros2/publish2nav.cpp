#include "publish2nav.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <exception>
#include <memory>
#include <thread>
#include <utility>

#include "tools/logger.hpp"

namespace io
{
namespace
{
// 裁判帧只有 1~5 Hz，5 Hz 发布足够，且远低于导航侧 2 Hz 看门狗的量级
constexpr int kRefereePeriodMs = static_cast<int>(1000.0 / 5.0);
// 连续多少拍（5 Hz 下即 5 s）没有收到任何裁判帧就告警一次
constexpr int64_t kRefereeEmptyWarnTicks = 25;

// 话题名与导航决策节点（相对名 referee/xxx + 命名空间 red_standard_robot1）严格对应
constexpr char kGameProgressTopic[] = "/red_standard_robot1/referee/game_progress";
constexpr char kCurrentHpTopic[] = "/red_standard_robot1/referee/current_hp";
constexpr char kMaximumHpTopic[] = "/red_standard_robot1/referee/maximum_hp";
}  // namespace

Publish2Nav::Publish2Nav() : Node("auto_aim_target_pos_publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("auto_aim_target_pos", 10);

  // 导航侧用这条关节反馈驱动模拟 JointState 的 TF；
  // 话题/字段与导航侧约定一致，节点名沿用现有节点，避免额外起一个 spin 线程。
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/red_standard_robot1/serial/gimbal_joint_state", 10);

  // 裁判状态：导航决策节点（pb_nav_goal_decision）的输入。
  // 数据源由 set_referee_source 注入，未注入时定时器空转不发布任何东西。
  game_progress_publisher_ =
    this->create_publisher<std_msgs::msg::UInt8>(kGameProgressTopic, 10);
  current_hp_publisher_ = this->create_publisher<std_msgs::msg::Int32>(kCurrentHpTopic, 10);
  maximum_hp_publisher_ = this->create_publisher<std_msgs::msg::Int32>(kMaximumHpTopic, 10);
  referee_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(kRefereePeriodMs), [this]() { publish_referee(); });

  RCLCPP_INFO(
    this->get_logger(),
    "auto_aim_target_pos_publisher node initialized (joint state -> "
    "/red_standard_robot1/serial/gimbal_joint_state, referee -> %s | %s | %s).",
    kGameProgressTopic, kCurrentHpTopic, kMaximumHpTopic);
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

void Publish2Nav::set_referee_source(RefereeSource source)
{
  std::lock_guard<std::mutex> lock(referee_source_mutex_);

  if (referee_source_) {
    RCLCPP_WARN(
      this->get_logger(), "referee source already set, ignoring the new one (keep the first).");
    return;
  }
  if (!source) {
    RCLCPP_WARN(this->get_logger(), "empty referee source, referee topics stay silent.");
    return;
  }

  referee_source_ = std::move(source);
  RCLCPP_INFO(
    this->get_logger(), "referee source set, publishing at %.0f Hz.", kRefereePublishHz);
}

void Publish2Nav::publish_referee()
{
  // 析构阶段 rclcpp 已 shutdown，定时器可能还会被调度一次；此时发布无意义且可能抛异常
  if (!rclcpp::ok()) return;

  RefereeSource source;
  {
    std::lock_guard<std::mutex> lock(referee_source_mutex_);
    source = referee_source_;
  }
  if (!source) return;

  RefereeState state;
  try {
    state = source();
  } catch (const std::exception & e) {
    // 取快照本不该抛异常；万一抛出也只丢一拍，不能让 ROS 定时器线程退出
    RCLCPP_ERROR(this->get_logger(), "referee state source failed: %s", e.what());
    return;
  }

  ++referee_ticks_;

  if (state.game_status_valid) {
    std_msgs::msg::UInt8 msg;
    msg.data = state.game_progress;
    game_progress_publisher_->publish(msg);
  }

  if (state.robot_status_valid) {
    std_msgs::msg::Int32 current_hp;
    current_hp.data = state.current_hp;
    current_hp_publisher_->publish(current_hp);

    std_msgs::msg::Int32 maximum_hp;
    maximum_hp.data = state.maximum_hp;
    maximum_hp_publisher_->publish(maximum_hp);

    // 帧校验通过但 max_hp 为 0 说明下位机数据异常：照发（导航侧会关闭低血判定），但要报出来
    if (state.maximum_hp <= 0 && !referee_bad_max_hp_warned_) {
      referee_bad_max_hp_warned_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "0x0201 max_hp = %d, navigation will disable the low-HP check until a valid value arrives",
        state.maximum_hp);
    }
  }

  // 一个有效帧都没收到：大概率是没接裁判系统（use_referee_system=false）或下位机未转发
  if (!state.game_status_valid && !state.robot_status_valid) {
    if (!referee_empty_warned_ && referee_ticks_ >= kRefereeEmptyWarnTicks) {
      referee_empty_warned_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "no 0x0001/0x0201 received yet; nothing published, goal decision stays in "
        "WAIT_GAME_START");
    }
    return;
  }

  const bool changed = !referee_has_last_ || state.game_progress != referee_last_game_progress_ ||
                       state.current_hp != referee_last_current_hp_ ||
                       state.maximum_hp != referee_last_maximum_hp_;
  if (!changed) return;

  referee_has_last_ = true;
  referee_last_game_progress_ = state.game_progress;
  referee_last_current_hp_ = state.current_hp;
  referee_last_maximum_hp_ = state.maximum_hp;
  RCLCPP_INFO(
    this->get_logger(),
    "referee state: game_progress=%u current_hp=%d maximum_hp=%d (valid g=%d r=%d)",
    state.game_progress, state.current_hp, state.maximum_hp, state.game_status_valid,
    state.robot_status_valid);
}

void Publish2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io
