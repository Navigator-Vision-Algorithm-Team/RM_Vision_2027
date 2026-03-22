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

  // Auto spin when enabled
  rclcpp::TimerBase::SharedPtr spin_timer_;
  bool enable_auto_spin_ = false;
  double spin_speed_ = 0.0;          // rad/s
  double spin_timer_period_ = 0.01;  // seconds
  double spin_pitch_ = -0.10;        // rad, downward pitch used during auto spin
  double current_spin_yaw_ = 0.0;    // rad
  double spin_dir_x_ = 1.0;          // unit direction x for spin yaw integration
  double spin_dir_y_ = 0.0;          // unit direction y for spin yaw integration
  std::atomic<bool> is_tracking_{false};  // Current tracking status
  std::atomic<bool> detector_has_armors_{false};  // True when detector reports at least one armor
  std::atomic<int64_t> last_armors_msg_ns_{0};    // Last /detector/armors timestamp in ns
  double armors_timeout_ = 0.2;                   // Timeout for stale Armors blocking
  std::mutex send_mutex_;                  // Protect serial port send
  rclcpp::Time last_tracking_time_;       // Last tracking timestamp
  double tracking_timeout_ = 2.0;         // Tracking timeout in seconds (increased for stability)
  rclcpp::Time last_receive_time_;        // Last receive timestamp for tracking timeout
  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
