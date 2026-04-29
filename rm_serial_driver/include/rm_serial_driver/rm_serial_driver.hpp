// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>

// C++ system
#include <atomic>
#include <cstdint>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/game_status.hpp"
#include "auto_aim_interfaces/msg/game_robot_status.hpp"

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

  void armorsCallback(auto_aim_interfaces::msg::Armors::SharedPtr msg);

  void sendNavigationCmd(const geometry_msgs::msg::Twist::SharedPtr msg);

  void reopenPort();

  void setParam(const rclcpp::Parameter & param);

  void resetTracker();

  void spinTimerCallback();

  void updateGameStartControl(uint8_t game_progress);

  float computePreGameNodPitch();

  void sendPreGameNodCommand();

  void processPacket(const std::vector<uint8_t>& data, uint16_t cmd_id);

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  // Service client to reset tracker
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // Aimimg point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;

  // Broadcast tf from odom to gimbal_link
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;

  // For debug usage
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Navigation subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_state_pub_;

    //==================================================
  rclcpp::Publisher<auto_aim_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::GameRobotStatus>::SharedPtr game_robot_status_pub_;
  //=//==================================================

  //发导航指令
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_point_pub_;

  // 自瞄/自旋运行时开关（参数）。
  rclcpp::TimerBase::SharedPtr spin_timer_;
  bool use_referee_system_ = false;      // true: 有裁判系统时按比赛状态控制自瞄/自旋；false: 一直允许自瞄/自旋。
  bool enable_auto_spin_ = true;        // true: 未跟踪目标时允许自旋。
  bool enable_pre_game_nod_ = true;     // true: 比赛开始前点头运动。
  bool enable_spin_start_delay_ = false;  // true: 比赛开始后延迟启动自旋。
  double spin_start_delay_sec_ = 2.0;
  int game_start_progress_threshold_ = 0;
  std::atomic<bool> game_started_{false};  // 比赛开始状态（裁判系统或手动模式）。
  std::atomic<bool> pre_game_nod_active_{true};  // true: 赛前点头状态会阻止自瞄。
  std::atomic<bool> spin_enabled_by_game_{false};  // true: 比赛状态允许自瞄/自旋。
  std::atomic<int64_t> spin_start_ready_ns_{0};  // 延迟启动自旋的时间戳（ns）。
  std::atomic<bool> reset_spin_motion_pending_{false};  // true: 下次定时器回调重置自旋积分。
  std::atomic<bool> reset_nod_motion_pending_{false};   // true: 下次定时器回调重置点头计时。

  double nod_pitch_min_ = 0.0;
  double nod_pitch_max_ = 0.5;
  double nod_frequency_ = 0.35;
  double nod_elapsed_time_ = 0.0;

  double spin_speed_ = 0.0;          // rad/s
  double spin_timer_period_ = 0.01;  // seconds
  double spin_pitch_ = 0.5;          // Pitch sine coefficient (negative sign applied in callback)
  double spin_yaw_coeff_ = 1.0;      // Yaw angular speed coefficient
  double spin_sine_cycles_per_turn_ = 3.0;  // Sine cycles when yaw rotates 2*pi
  double spin_phase_shift_per_turn_ = 0.35;  // Extra pitch phase shift added after each yaw turn
  double spin_elapsed_time_ = 0.0;   // Auto-spin elapsed time for sine sampling
  double spin_extra_phase_ = 0.0;    // Accumulated extra phase to avoid repeating trajectory
  double last_spin_yaw_for_phase_ = 0.0;  // Previous yaw used for turn-wrap detection
  double current_spin_yaw_ = 0.0;    // rad
  double spin_dir_x_ = 1.0;          // unit direction x for spin yaw integration
  double spin_dir_y_ = 0.0;          // unit direction y for spin yaw integration
  std::atomic<bool> is_tracking_{false};  // true: 跟踪器当前有有效目标。
  std::atomic<bool> detector_has_armors_{false};  // true: 检测器当前有装甲板。
  std::atomic<int64_t> last_armors_msg_ns_{0};    // 最近一次 /detector/armors 的时间戳（ns）。
  double armors_timeout_ = 0.2;                   // 装甲板消息超时阈值。
  std::mutex send_mutex_;                         // 保护串口发送的互斥锁。
  rclcpp::Time last_tracking_time_;               // 最近一次跟踪时间戳。
  double tracking_timeout_ = 2.0;                 // 跟踪超时阈值（秒）。
  rclcpp::Time last_receive_time_;                // 最近一次接收时间戳（用于跟踪超时判断）。
  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
