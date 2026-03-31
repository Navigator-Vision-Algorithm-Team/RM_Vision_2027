#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"
#include "rm_serial_driver/solve_trajectory.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  nav_state_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/serial/gimbal_joint_state", 10);
  game_status_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::GameStatus>("/serial/game_status", 10);
  game_robot_status_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::GameRobotStatus>("/serial/game_robot_status", 10);
  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

    // 创建导航指令订阅器
  nav_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/red_standard_robot1/cmd_vel", 10,
    std::bind(&RMSerialDriver::sendNavigationCmd, this, std::placeholders::_1));


  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  tracking_timeout_ = this->declare_parameter("tracking_timeout", 0.7);
  armors_timeout_ = this->declare_parameter("armors_timeout", 0.2);

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::armorsCallback, this, std::placeholders::_1));

  // Auto spin parameters and timer
  enable_auto_spin_ = this->declare_parameter("enable_auto_spin", true);
  spin_speed_ = this->declare_parameter("spin_speed", 1.2);
  spin_timer_period_ = this->declare_parameter("spin_timer_period", 0.05);
  spin_pitch_ = this->declare_parameter("spin_pitch", 0.2);
  current_spin_yaw_ = 0.0;
  spin_dir_x_ = std::cos(current_spin_yaw_);
  spin_dir_y_ = std::sin(current_spin_yaw_);

  if (enable_auto_spin_) {
    spin_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(spin_timer_period_),
      std::bind(&RMSerialDriver::spinTimerCallback, this));
  }
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> flag(1);
  std::vector<uint8_t> headdata;
  std::vector<uint8_t> data;

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(flag);

      if (flag[0] == 0xA5) {
        headdata.resize(sizeof(Header) - 1);
        serial_driver_->port()->receive(headdata);

        headdata.insert(headdata.begin(), flag[0]);
        Header header = fromVector<Header>(headdata);

        bool crc_ok = crc8::Verify_CRC8_Check_Sum(
          reinterpret_cast<const uint8_t *>(&header), sizeof(header) - 2);
        if (crc_ok) {
          IMUPacket imu_packet;
          classisPacket classis_packet;
          bottonPacket botton_packet;
          bassPacket bass_packet;
          GameStatusPacket game_status_packet;
          GameRobotStatusPacket game_robot_status_packet;
          geometry_msgs::msg::TransformStamped t;
          tf2::Quaternion q;
          switch (header.cmd_id) {
            case 0x502:
              data.resize(sizeof(IMUPacket) - sizeof(Header));
              serial_driver_->port()->receive(data);
              data.insert(data.begin(), headdata.begin(), headdata.end());
              imu_packet = fromVector<IMUPacket>(data);
              if (!crc16::Verify_CRC16_Check_Sum(
                    reinterpret_cast<const uint8_t *>(&imu_packet), sizeof(imu_packet))) {
                // RCLCPP_ERROR(get_logger(), "Data CRC error!");
                break;
              }
              //  RCLCPP_INFO(get_logger(), "[Receive] id %d!", imu_packet.id);
              //  RCLCPP_INFO(get_logger(), "[Receive] roll %f!", imu_packet.roll);
              //  RCLCPP_INFO(get_logger(), "[Receive] pitch %f!", imu_packet.pitch);
              //  RCLCPP_INFO(get_logger(), "[Receive] yaw %f!", imu_packet.yaw);

              // RCLCPP_WARN(
              //   get_logger(), "[Receive] roboid %d! color %d", imu_packet.roboid,
              //   imu_packet.roboid > 100 ? 0 : 1);
              if (
                !initial_set_param_ || (imu_packet.roboid > 100 ? 0 : 1) != previous_receive_color_) {
                setParam(rclcpp::Parameter("detect_color", (imu_packet.roboid > 100 ? 0 : 1)));
                previous_receive_color_ = imu_packet.roboid > 100 ? 0 : 1;
              }

              st.current_pitch = imu_packet.pitch;
              st.current_yaw = imu_packet.yaw;

              timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
              t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
              t.header.frame_id = "odom";
              t.child_frame_id = "gimbal_link";

              q.setRPY(imu_packet.roll, imu_packet.pitch, imu_packet.yaw);
              t.transform.rotation = tf2::toMsg(q);
              tf_broadcaster_->sendTransform(t);
              break;
            case 0x503:
              data.resize(sizeof(classisPacket) - sizeof(Header));
              serial_driver_->port()->receive(data);
              data.insert(data.begin(), headdata.begin(), headdata.end());
              classis_packet = fromVector<classisPacket>(data);
              if (!crc16::Verify_CRC16_Check_Sum(
                    reinterpret_cast<const uint8_t *>(&classis_packet), sizeof(classis_packet))) {
                RCLCPP_ERROR(get_logger(), "Data CRC error!");
                break;
              }

              break;
            case 0x504:
              data.resize(sizeof(bottonPacket) - sizeof(Header));
              serial_driver_->port()->receive(data);
              data.insert(data.begin(), headdata.begin(), headdata.end());
              botton_packet = fromVector<bottonPacket>(data);
              if (!crc16::Verify_CRC16_Check_Sum(
                    reinterpret_cast<const uint8_t *>(&botton_packet), sizeof(botton_packet))) {
                RCLCPP_ERROR(get_logger(), "Data CRC error!");
                break;
              }

              break;
            case 0x505:
              data.resize(sizeof(bassPacket) - sizeof(Header));
              serial_driver_->port()->receive(data);
              data.insert(data.begin(), headdata.begin(), headdata.end());
              bass_packet = fromVector<bassPacket>(data);
              if (!crc16::Verify_CRC16_Check_Sum(
                    reinterpret_cast<const uint8_t *>(&bass_packet), sizeof(bass_packet))) {
                RCLCPP_ERROR(get_logger(), "Data CRC error!");
                break;
              }

              if (bass_packet.reset) {
                resetTracker();
              }
              break;
              //=================================================================
            case 0x0001:
              RCLCPP_INFO(get_logger(), "coming");
              data.resize(sizeof(GameStatusPacket) - sizeof(Header));
              serial_driver_->port()->receive(data);
              data.insert(data.begin(), headdata.begin(), headdata.end());
              game_status_packet = fromVector<GameStatusPacket>(data);
              if (!crc16::Verify_CRC16_Check_Sum(
                      reinterpret_cast<const uint8_t *>(&game_status_packet),sizeof(game_status_packet))) {
                RCLCPP_ERROR(get_logger(), "Game status CRC error!");
                break;
              }

              {
                const int game_type = static_cast<int>(game_status_packet.game_info & 0x0F);
                const int game_progress =
                  static_cast<int>((game_status_packet.game_info >> 4) & 0x0F);
                const int stage_remain_time = static_cast<int>(game_status_packet.stage_remain_time);
                const uint64_t sync_timestamp = game_status_packet.sync_timestamp;

                auto_aim_interfaces::msg::GameStatus msg;
                msg.game_type = static_cast<uint8_t>(game_type);
                msg.game_progress = static_cast<uint8_t>(game_progress);
                msg.stage_remain_time = static_cast<uint16_t>(stage_remain_time);
                msg.sync_timestamp = sync_timestamp;
                game_status_pub_->publish(msg);
              }

              RCLCPP_INFO(
                get_logger(),
                "[Recv 0x0001] raw_game_info=0x%02X type=%u progress=%u remain=%u sync=%llu",
                static_cast<unsigned>(game_status_packet.game_info),
                static_cast<unsigned>(game_status_packet.game_info & 0x0F),
                static_cast<unsigned>((game_status_packet.game_info >> 4) & 0x0F),
                static_cast<unsigned>(game_status_packet.stage_remain_time),
                static_cast<unsigned long long>(game_status_packet.sync_timestamp));

              RCLCPP_INFO(
                get_logger(),
                "[GameStatus] type=%u progress=%u remain=%u sync=%llu",
                static_cast<unsigned>(game_status_packet.game_info & 0x0F),
                static_cast<unsigned>((game_status_packet.game_info >> 4) & 0x0F),
                static_cast<unsigned>(game_status_packet.stage_remain_time),
                static_cast<unsigned long long>(game_status_packet.sync_timestamp));
              break;
            case 0x0100:
              RCLCPP_INFO(get_logger(), "coming");
              data.resize(sizeof(GameStatusPacket) - sizeof(Header));
              serial_driver_->port()->receive(data);
              data.insert(data.begin(), headdata.begin(), headdata.end());
              game_status_packet = fromVector<GameStatusPacket>(data);
              if (!crc16::Verify_CRC16_Check_Sum(
                      reinterpret_cast<const uint8_t *>(&game_status_packet),sizeof(game_status_packet))) {
                RCLCPP_ERROR(get_logger(), "Game status CRC error!");
                break;
              }

              {
                const int game_type = static_cast<int>(game_status_packet.game_info & 0x0F);
                const int game_progress =
                  static_cast<int>((game_status_packet.game_info >> 4) & 0x0F);
                const int stage_remain_time = static_cast<int>(game_status_packet.stage_remain_time);
                const uint64_t sync_timestamp = game_status_packet.sync_timestamp;

                auto_aim_interfaces::msg::GameStatus msg;
                msg.game_type = static_cast<uint8_t>(game_type);
                msg.game_progress = static_cast<uint8_t>(game_progress);
                msg.stage_remain_time = static_cast<uint16_t>(stage_remain_time);
                msg.sync_timestamp = sync_timestamp;
                game_status_pub_->publish(msg);
              }

              RCLCPP_INFO(
                get_logger(),
                "[Recv 0x0100] raw_game_info=0x%02X type=%u progress=%u remain=%u sync=%llu",
                static_cast<unsigned>(game_status_packet.game_info),
                static_cast<unsigned>(game_status_packet.game_info & 0x0F),
                static_cast<unsigned>((game_status_packet.game_info >> 4) & 0x0F),
                static_cast<unsigned>(game_status_packet.stage_remain_time),
                static_cast<unsigned long long>(game_status_packet.sync_timestamp));

              RCLCPP_INFO(
                get_logger(),
                "[GameStatus] type=%u progress=%u remain=%u sync=%llu",
                static_cast<unsigned>(game_status_packet.game_info & 0x0F),
                static_cast<unsigned>((game_status_packet.game_info >> 4) & 0x0F),
                static_cast<unsigned>(game_status_packet.stage_remain_time),
                static_cast<unsigned long long>(game_status_packet.sync_timestamp));
              break;
            case 0x0201:
              data.resize(sizeof(GameRobotStatusPacket) - sizeof(Header));
              serial_driver_->port()->receive(data);
              data.insert(data.begin(), headdata.begin(), headdata.end());
              game_robot_status_packet = fromVector<GameRobotStatusPacket>(data);
              if (!crc16::Verify_CRC16_Check_Sum(
                    reinterpret_cast<const uint8_t *>(&game_robot_status_packet),
                    sizeof(game_robot_status_packet))) {
                RCLCPP_ERROR(get_logger(), "Game robot status CRC error!");
                break;
              }

              {
                const int robot_id = static_cast<int>(game_robot_status_packet.robot_id);
                const int robot_level = static_cast<int>(game_robot_status_packet.robot_level);
                const int remain_hp = static_cast<int>(game_robot_status_packet.remain_hp);
                const int max_hp = static_cast<int>(game_robot_status_packet.max_hp);
                const int shooter_cooling_rate =
                  static_cast<int>(game_robot_status_packet.shooter_cooling_rate);
                const int shooter_heat_limit =
                  static_cast<int>(game_robot_status_packet.shooter_heat_limit);
                const int chassis_power_limit =
                  static_cast<int>(game_robot_status_packet.chassis_power_limit);
                const int mains_power_gimbal_output =
                  static_cast<int>(game_robot_status_packet.mains_power_state & 0x01);
                const int mains_power_chassis_output =
                  static_cast<int>((game_robot_status_packet.mains_power_state >> 1) & 0x01);
                const int mains_power_shooter_output =
                  static_cast<int>((game_robot_status_packet.mains_power_state >> 2) & 0x01);

                auto_aim_interfaces::msg::GameRobotStatus msg;
                msg.robot_id = static_cast<uint8_t>(robot_id);
                msg.robot_level = static_cast<uint8_t>(robot_level);
                msg.remain_hp = static_cast<uint16_t>(remain_hp);
                msg.max_hp = static_cast<uint16_t>(max_hp);
                msg.shooter_cooling_rate = static_cast<uint16_t>(shooter_cooling_rate);
                msg.shooter_heat_limit = static_cast<uint16_t>(shooter_heat_limit);
                msg.chassis_power_limit = static_cast<uint16_t>(chassis_power_limit);
                msg.mains_power_gimbal_output = mains_power_gimbal_output != 0;
                msg.mains_power_chassis_output = mains_power_chassis_output != 0;
                msg.mains_power_shooter_output = mains_power_shooter_output != 0;
                game_robot_status_pub_->publish(msg);
              }

              RCLCPP_INFO(
                get_logger(),
                "[Recv 0x0201] id=%u lvl=%u hp=%u/%u cool=%u heat=%u power=%u mains=0x%02X",
                static_cast<unsigned>(game_robot_status_packet.robot_id),
                static_cast<unsigned>(game_robot_status_packet.robot_level),
                static_cast<unsigned>(game_robot_status_packet.remain_hp),
                static_cast<unsigned>(game_robot_status_packet.max_hp),
                static_cast<unsigned>(game_robot_status_packet.shooter_cooling_rate),
                static_cast<unsigned>(game_robot_status_packet.shooter_heat_limit),
                static_cast<unsigned>(game_robot_status_packet.chassis_power_limit),
                static_cast<unsigned>(game_robot_status_packet.mains_power_state));

              RCLCPP_INFO(
                get_logger(),
                "[GameRobotStatus] id=%u lvl=%u hp=%u/%u cool=%u heat=%u power=%u out=%u%u%u",
                static_cast<unsigned>(game_robot_status_packet.robot_id),
                static_cast<unsigned>(game_robot_status_packet.robot_level),
                static_cast<unsigned>(game_robot_status_packet.remain_hp),
                static_cast<unsigned>(game_robot_status_packet.max_hp),
                static_cast<unsigned>(game_robot_status_packet.shooter_cooling_rate),
                static_cast<unsigned>(game_robot_status_packet.shooter_heat_limit),
                static_cast<unsigned>(game_robot_status_packet.chassis_power_limit),
                static_cast<unsigned>(game_robot_status_packet.mains_power_state & 0x01),
                static_cast<unsigned>((game_robot_status_packet.mains_power_state >> 1) & 0x01),
                static_cast<unsigned>((game_robot_status_packet.mains_power_state >> 2) & 0x01));
              break;
            case 0x0102:
              data.resize(sizeof(GameRobotStatusPacket) - sizeof(Header));
              serial_driver_->port()->receive(data);
              data.insert(data.begin(), headdata.begin(), headdata.end());
              game_robot_status_packet = fromVector<GameRobotStatusPacket>(data);
              if (!crc16::Verify_CRC16_Check_Sum(
                    reinterpret_cast<const uint8_t *>(&game_robot_status_packet),
                    sizeof(game_robot_status_packet))) {
                RCLCPP_ERROR(get_logger(), "Game robot status CRC error!");
                break;
              }

              {
                const int robot_id = static_cast<int>(game_robot_status_packet.robot_id);
                const int robot_level = static_cast<int>(game_robot_status_packet.robot_level);
                const int remain_hp = static_cast<int>(game_robot_status_packet.remain_hp);
                const int max_hp = static_cast<int>(game_robot_status_packet.max_hp);
                const int shooter_cooling_rate =
                  static_cast<int>(game_robot_status_packet.shooter_cooling_rate);
                const int shooter_heat_limit =
                  static_cast<int>(game_robot_status_packet.shooter_heat_limit);
                const int chassis_power_limit =
                  static_cast<int>(game_robot_status_packet.chassis_power_limit);
                const int mains_power_gimbal_output =
                  static_cast<int>(game_robot_status_packet.mains_power_state & 0x01);
                const int mains_power_chassis_output =
                  static_cast<int>((game_robot_status_packet.mains_power_state >> 1) & 0x01);
                const int mains_power_shooter_output =
                  static_cast<int>((game_robot_status_packet.mains_power_state >> 2) & 0x01);

                auto_aim_interfaces::msg::GameRobotStatus msg;
                msg.robot_id = static_cast<uint8_t>(robot_id);
                msg.robot_level = static_cast<uint8_t>(robot_level);
                msg.remain_hp = static_cast<uint16_t>(remain_hp);
                msg.max_hp = static_cast<uint16_t>(max_hp);
                msg.shooter_cooling_rate = static_cast<uint16_t>(shooter_cooling_rate);
                msg.shooter_heat_limit = static_cast<uint16_t>(shooter_heat_limit);
                msg.chassis_power_limit = static_cast<uint16_t>(chassis_power_limit);
                msg.mains_power_gimbal_output = mains_power_gimbal_output != 0;
                msg.mains_power_chassis_output = mains_power_chassis_output != 0;
                msg.mains_power_shooter_output = mains_power_shooter_output != 0;
                game_robot_status_pub_->publish(msg);
              }

              RCLCPP_INFO(
                get_logger(),
                "[Recv 0x0102] id=%u lvl=%u hp=%u/%u cool=%u heat=%u power=%u mains=0x%02X",
                static_cast<unsigned>(game_robot_status_packet.robot_id),
                static_cast<unsigned>(game_robot_status_packet.robot_level),
                static_cast<unsigned>(game_robot_status_packet.remain_hp),
                static_cast<unsigned>(game_robot_status_packet.max_hp),
                static_cast<unsigned>(game_robot_status_packet.shooter_cooling_rate),
                static_cast<unsigned>(game_robot_status_packet.shooter_heat_limit),
                static_cast<unsigned>(game_robot_status_packet.chassis_power_limit),
                static_cast<unsigned>(game_robot_status_packet.mains_power_state));

              RCLCPP_INFO(
                get_logger(),
                "[GameRobotStatus] id=%u lvl=%u hp=%u/%u cool=%u heat=%u power=%u out=%u%u%u",
                static_cast<unsigned>(game_robot_status_packet.robot_id),
                static_cast<unsigned>(game_robot_status_packet.robot_level),
                static_cast<unsigned>(game_robot_status_packet.remain_hp),
                static_cast<unsigned>(game_robot_status_packet.max_hp),
                static_cast<unsigned>(game_robot_status_packet.shooter_cooling_rate),
                static_cast<unsigned>(game_robot_status_packet.shooter_heat_limit),
                static_cast<unsigned>(game_robot_status_packet.chassis_power_limit),
                static_cast<unsigned>(game_robot_status_packet.mains_power_state & 0x01),
                static_cast<unsigned>((game_robot_status_packet.mains_power_state >> 1) & 0x01),
                static_cast<unsigned>((game_robot_status_packet.mains_power_state >> 2) & 0x01));
              break;
          }
          // RCLCPP_INFO(get_logger(), "CRC OK!");

          // LOG [Receive] aim_xyz
          // RCLCPP_INFO(get_logger(), "[Receive] aim_pitch %f!", packet.aim_x);
          // RCLCPP_INFO(get_logger(), "[Receive] aim_yaw %f!", packet.aim_y);
          // RCLCPP_INFO(get_logger(), "[Receive] aim_roll %f!", packet.aim_z);

          // LOG [Receive] [Receive] rpy
          // RCLCPP_INFO(get_logger(), "[Receive] roll %f!", packet.roll);
          // RCLCPP_INFO(get_logger(), "[Receive] pitch %f!", packet.pitch);
          // RCLCPP_INFO(get_logger(), "[Receive] yaw %f!", packet.yaw);

          // RCLCPP_INFO(get_logger(), "----------------------------");

          // if (abs(packet.aim_x) > 0.01) {
          //   aiming_point_.header.stamp = this->now();
          //   aiming_point_.pose.position.x = packet.aim_x;
          //   aiming_point_.pose.position.y = packet.aim_y;
          //   aiming_point_.pose.position.z = packet.aim_z;
          //   marker_pub_->publish(aiming_point_);
          // }
        } else {
          RCLCPP_ERROR(get_logger(), "Head CRC error!");
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", flag[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}



void RMSerialDriver::processPacket(const std::vector<uint8_t> & data, uint16_t cmd_id)
{
  switch (cmd_id) {
    case 0x502: {
      IMUPacket imu_packet = fromVector<IMUPacket>(data);
      if (!crc16::Verify_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t *>(&imu_packet), sizeof(imu_packet))) {
        RCLCPP_ERROR(get_logger(), "IMU Data CRC error! %zu", data.size());
        return;
      }

      if (!initial_set_param_ || (imu_packet.roboid > 100 ? 0 : 1) != previous_receive_color_) {
        setParam(rclcpp::Parameter("detect_color", (imu_packet.roboid > 100 ? 0 : 1)));
        previous_receive_color_ = imu_packet.roboid > 100 ? 0 : 1;
      }

      st.current_pitch = imu_packet.pitch;
      st.current_yaw = imu_packet.yaw;

      st.time = imu_packet.timeseries; //时间戳
      // RCLCPP_INFO(get_logger(), "时间戳%d", st.time);

      // 发布TF变换
      auto t = std::make_unique<geometry_msgs::msg::TransformStamped>();
      t->header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      t->header.frame_id = "odom";
      t->child_frame_id = "gimbal_link";

      tf2::Quaternion q;
      q.setRPY(imu_packet.roll, imu_packet.pitch, imu_packet.yaw);
      t->transform.rotation = tf2::toMsg(q);
      tf_broadcaster_->sendTransform(*t);

      //导航部分////////////////////////////////////////
      // 发布导航状态////////////////////////////////////////////
      auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
      twist_msg->angular.x = imu_packet.pitch;  // pitch
      twist_msg->angular.z = imu_packet.yaw;    // yaw
      nav_state_pub_->publish(*twist_msg);

      // RCLCPP_INFO(get_logger(), "[接收导航状态] pitch:%.2f, yaw:%.2f",
      //             imu_packet.pitch, imu_packet.yaw);
      break;
    }
    case 0x503: {
      classisPacket classis_packet = fromVector<classisPacket>(data);
      if (!crc16::Verify_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t *>(&classis_packet), sizeof(classis_packet))) {
        RCLCPP_ERROR(get_logger(), "Classis Data CRC error!");
      }
      break;
    }
    case 0x504: {
      bottonPacket botton_packet = fromVector<bottonPacket>(data);
      if (!crc16::Verify_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t *>(&botton_packet), sizeof(botton_packet))) {
        RCLCPP_ERROR(get_logger(), "Button Data CRC error!");
      }
      break;
    }
    case 0x505: {
      bassPacket bass_packet = fromVector<bassPacket>(data);
      if (!crc16::Verify_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t *>(&bass_packet), sizeof(bass_packet))) {
        RCLCPP_ERROR(get_logger(), "Bass Data CRC error!");
        return;
      }

      if (bass_packet.reset) {
        resetTracker();
      }

      break;
    }
        // ========================================================
    case 0x0001: {
      RCLCPP_INFO(get_logger(), "coming");
      GameStatusPacket game_status_packet = fromVector<GameStatusPacket>(data);
      if (!crc16::Verify_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t *>(&game_status_packet), sizeof(game_status_packet))) {
        RCLCPP_ERROR(get_logger(), "Game status CRC error!");
        return;
      }

      const int game_type = static_cast<int>(game_status_packet.game_info & 0x0F);
      const int game_progress = static_cast<int>((game_status_packet.game_info >> 4) & 0x0F);
      const int stage_remain_time = static_cast<int>(game_status_packet.stage_remain_time);
      const uint64_t sync_timestamp = game_status_packet.sync_timestamp;

      auto_aim_interfaces::msg::GameStatus msg;
      msg.game_type = static_cast<uint8_t>(game_type);
      msg.game_progress = static_cast<uint8_t>(game_progress);
      msg.stage_remain_time = static_cast<uint16_t>(stage_remain_time);
      msg.sync_timestamp = sync_timestamp;
      game_status_pub_->publish(msg);

      RCLCPP_INFO(
        get_logger(),
        "[Recv 0x0001] raw_game_info=0x%02X type=%u progress=%u remain=%u sync=%llu",
        static_cast<unsigned>(game_status_packet.game_info),
        static_cast<unsigned>(game_type),
        static_cast<unsigned>(game_progress),
        static_cast<unsigned>(stage_remain_time),
        static_cast<unsigned long long>(sync_timestamp));

      RCLCPP_INFO(
        get_logger(),
        "[GameStatus] type=%u progress=%u remain=%u sync=%llu",
        static_cast<unsigned>(game_type),
        static_cast<unsigned>(game_progress),
        static_cast<unsigned>(stage_remain_time),
        static_cast<unsigned long long>(sync_timestamp));
      break;
    }
    case 0x0100: {
      RCLCPP_INFO(get_logger(), "coming");
      GameStatusPacket game_status_packet = fromVector<GameStatusPacket>(data);
      if (!crc16::Verify_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t *>(&game_status_packet), sizeof(game_status_packet))) {
        RCLCPP_ERROR(get_logger(), "Game status CRC error!");
        return;
      }

      const int game_type = static_cast<int>(game_status_packet.game_info & 0x0F);
      const int game_progress = static_cast<int>((game_status_packet.game_info >> 4) & 0x0F);
      const int stage_remain_time = static_cast<int>(game_status_packet.stage_remain_time);
      const uint64_t sync_timestamp = game_status_packet.sync_timestamp;

      auto_aim_interfaces::msg::GameStatus msg;
      msg.game_type = static_cast<uint8_t>(game_type);
      msg.game_progress = static_cast<uint8_t>(game_progress);
      msg.stage_remain_time = static_cast<uint16_t>(stage_remain_time);
      msg.sync_timestamp = sync_timestamp;
      game_status_pub_->publish(msg);

      RCLCPP_INFO(
        get_logger(),
        "[Recv 0x0100] raw_game_info=0x%02X type=%u progress=%u remain=%u sync=%llu",
        static_cast<unsigned>(game_status_packet.game_info),
        static_cast<unsigned>(game_type),
        static_cast<unsigned>(game_progress),
        static_cast<unsigned>(stage_remain_time),
        static_cast<unsigned long long>(sync_timestamp));

      RCLCPP_INFO(
        get_logger(),
        "[GameStatus] type=%u progress=%u remain=%u sync=%llu",
        static_cast<unsigned>(game_type),
        static_cast<unsigned>(game_progress),
        static_cast<unsigned>(stage_remain_time),
        static_cast<unsigned long long>(sync_timestamp));
      break;
    }
    case 0x0201: {
      GameRobotStatusPacket game_robot_status_packet = fromVector<GameRobotStatusPacket>(data);
      if (!crc16::Verify_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t *>(&game_robot_status_packet),
            sizeof(game_robot_status_packet))) {
        RCLCPP_ERROR(get_logger(), "Game robot status CRC error!");
        return;
      }

      const int robot_id = static_cast<int>(game_robot_status_packet.robot_id);
      const int robot_level = static_cast<int>(game_robot_status_packet.robot_level);
      const int remain_hp = static_cast<int>(game_robot_status_packet.remain_hp);
      const int max_hp = static_cast<int>(game_robot_status_packet.max_hp);
      const int shooter_cooling_rate =
        static_cast<int>(game_robot_status_packet.shooter_cooling_rate);
      const int shooter_heat_limit = static_cast<int>(game_robot_status_packet.shooter_heat_limit);
      const int chassis_power_limit = static_cast<int>(game_robot_status_packet.chassis_power_limit);
      const int mains_power_gimbal_output =
        static_cast<int>(game_robot_status_packet.mains_power_state & 0x01);
      const int mains_power_chassis_output =
        static_cast<int>((game_robot_status_packet.mains_power_state >> 1) & 0x01);
      const int mains_power_shooter_output =
        static_cast<int>((game_robot_status_packet.mains_power_state >> 2) & 0x01);

      auto_aim_interfaces::msg::GameRobotStatus msg;
      msg.robot_id = static_cast<uint8_t>(robot_id);
      msg.robot_level = static_cast<uint8_t>(robot_level);
      msg.remain_hp = static_cast<uint16_t>(remain_hp);
      msg.max_hp = static_cast<uint16_t>(max_hp);
      msg.shooter_cooling_rate = static_cast<uint16_t>(shooter_cooling_rate);
      msg.shooter_heat_limit = static_cast<uint16_t>(shooter_heat_limit);
      msg.chassis_power_limit = static_cast<uint16_t>(chassis_power_limit);
      msg.mains_power_gimbal_output = mains_power_gimbal_output != 0;
      msg.mains_power_chassis_output = mains_power_chassis_output != 0;
      msg.mains_power_shooter_output = mains_power_shooter_output != 0;
      game_robot_status_pub_->publish(msg);

      RCLCPP_INFO(
        get_logger(),
        "[Recv 0x0201] id=%u lvl=%u hp=%u/%u cool=%u heat=%u power=%u mains=0x%02X",
        static_cast<unsigned>(robot_id),
        static_cast<unsigned>(robot_level),
        static_cast<unsigned>(remain_hp),
        static_cast<unsigned>(max_hp),
        static_cast<unsigned>(shooter_cooling_rate),
        static_cast<unsigned>(shooter_heat_limit),
        static_cast<unsigned>(chassis_power_limit),
        static_cast<unsigned>(game_robot_status_packet.mains_power_state));

      RCLCPP_INFO(
        get_logger(),
        "[GameRobotStatus] id=%u lvl=%u hp=%u/%u cool=%u heat=%u power=%u out=%u%u%u",
        static_cast<unsigned>(robot_id),
        static_cast<unsigned>(robot_level),
        static_cast<unsigned>(remain_hp),
        static_cast<unsigned>(max_hp),
        static_cast<unsigned>(shooter_cooling_rate),
        static_cast<unsigned>(shooter_heat_limit),
        static_cast<unsigned>(chassis_power_limit),
        static_cast<unsigned>(mains_power_gimbal_output),
        static_cast<unsigned>(mains_power_chassis_output),
        static_cast<unsigned>(mains_power_shooter_output));
      break;
    }
    case 0x0102: {
      GameRobotStatusPacket game_robot_status_packet = fromVector<GameRobotStatusPacket>(data);
      if (!crc16::Verify_CRC16_Check_Sum(
            reinterpret_cast<const uint8_t *>(&game_robot_status_packet),
            sizeof(game_robot_status_packet))) {
        RCLCPP_ERROR(get_logger(), "Game robot status CRC error!");
        return;
      }

      const int robot_id = static_cast<int>(game_robot_status_packet.robot_id);
      const int robot_level = static_cast<int>(game_robot_status_packet.robot_level);
      const int remain_hp = static_cast<int>(game_robot_status_packet.remain_hp);
      const int max_hp = static_cast<int>(game_robot_status_packet.max_hp);
      const int shooter_cooling_rate =
        static_cast<int>(game_robot_status_packet.shooter_cooling_rate);
      const int shooter_heat_limit = static_cast<int>(game_robot_status_packet.shooter_heat_limit);
      const int chassis_power_limit = static_cast<int>(game_robot_status_packet.chassis_power_limit);
      const int mains_power_gimbal_output =
        static_cast<int>(game_robot_status_packet.mains_power_state & 0x01);
      const int mains_power_chassis_output =
        static_cast<int>((game_robot_status_packet.mains_power_state >> 1) & 0x01);
      const int mains_power_shooter_output =
        static_cast<int>((game_robot_status_packet.mains_power_state >> 2) & 0x01);

      auto_aim_interfaces::msg::GameRobotStatus msg;
      msg.robot_id = static_cast<uint8_t>(robot_id);
      msg.robot_level = static_cast<uint8_t>(robot_level);
      msg.remain_hp = static_cast<uint16_t>(remain_hp);
      msg.max_hp = static_cast<uint16_t>(max_hp);
      msg.shooter_cooling_rate = static_cast<uint16_t>(shooter_cooling_rate);
      msg.shooter_heat_limit = static_cast<uint16_t>(shooter_heat_limit);
      msg.chassis_power_limit = static_cast<uint16_t>(chassis_power_limit);
      msg.mains_power_gimbal_output = mains_power_gimbal_output != 0;
      msg.mains_power_chassis_output = mains_power_chassis_output != 0;
      msg.mains_power_shooter_output = mains_power_shooter_output != 0;
      game_robot_status_pub_->publish(msg);

      RCLCPP_INFO(
        get_logger(),
        "[Recv 0x0102] id=%u lvl=%u hp=%u/%u cool=%u heat=%u power=%u mains=0x%02X",
        static_cast<unsigned>(robot_id),
        static_cast<unsigned>(robot_level),
        static_cast<unsigned>(remain_hp),
        static_cast<unsigned>(max_hp),
        static_cast<unsigned>(shooter_cooling_rate),
        static_cast<unsigned>(shooter_heat_limit),
        static_cast<unsigned>(chassis_power_limit),
        static_cast<unsigned>(game_robot_status_packet.mains_power_state));

      RCLCPP_INFO(
        get_logger(),
        "[GameRobotStatus] id=%u lvl=%u hp=%u/%u cool=%u heat=%u power=%u out=%u%u%u",
        static_cast<unsigned>(robot_id),
        static_cast<unsigned>(robot_level),
        static_cast<unsigned>(remain_hp),
        static_cast<unsigned>(max_hp),
        static_cast<unsigned>(shooter_cooling_rate),
        static_cast<unsigned>(shooter_heat_limit),
        static_cast<unsigned>(chassis_power_limit),
        static_cast<unsigned>(mains_power_gimbal_output),
        static_cast<unsigned>(mains_power_chassis_output),
        static_cast<unsigned>(mains_power_shooter_output));
      break;
    }
  }
}


void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  const static std::map<std::string, uint8_t> robo_id_unit8_map{
    {"", 0x000},  {"outpost", 0x010}, {"1", 0x001},     {"2", 0x002},   {"3", 0x003},
    {"4", 0x004}, {"5", 0x005},       {"guard", 0x007}, {"base", 0x011}};

  try {
    std::lock_guard<std::mutex> lock(send_mutex_);  // Ensure thread safety when accessing send-related variables
    Header header;
    SendPacket packet;
    header.data_length = sizeof(packet) - sizeof(header) - 2;
    crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&header), sizeof(header) - 2);
    header.cmd_id = 0x0402;
    packet.header = header;
    float aim_x = 0, aim_y = 0, aim_z = 0;
    float pitch = 0, yaw = 0;

    st.xw = msg->position.x;
    st.yw = msg->position.y;
    st.zw = msg->position.z;
    st.vxw = msg->velocity.x;
    st.vyw = msg->velocity.y;
    st.vzw = msg->velocity.z;
    st.tar_yaw = msg->yaw;
    st.v_yaw = msg->v_yaw;
    st.r1 = msg->radius_1;
    st.r2 = msg->radius_2;
    st.dz = msg->dz;
    st.armor_id = ARMOR_ID(id_unit8_map.at(msg->id));
    st.armor_num = ARMOR_NUM(msg->armors_num);

    // Control auto spin state
    if (msg->tracking) {
      is_tracking_.store(true);
      last_tracking_time_ = this->now();
    } 
    // else {
    //   is_tracking_.store(false);
    // }
    last_receive_time_ = this->now();

    // If not tracking and auto spin is disabled, send current gimbal pose
    if (!msg->tracking && !enable_auto_spin_) {
      packet.pitch = st.current_pitch;
      packet.yaw = st.current_yaw;
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      serial_driver_->port()->send(toVector(packet));
      return;
    }

    // If not tracking and auto spin is enabled, let spinTimerCallback handle it
    if (!msg->tracking && enable_auto_spin_) {
      return;
    }

    // Calculate pitch and yaw for tracking
    aim_x = msg->position.x;
    aim_y = msg->position.y;
    aim_z = msg->position.z;
    yaw = msg->yaw;

    packet.shoot =
      autoSolveTrajectory(&pitch, &yaw, &aim_x, &aim_y, &aim_z) & (msg->tracking ? 1 : 0);
    // packet.tracking = msg->tracking;
    // packet.id = id_unit8_map.at(msg->id);
    // packet.armors_num = msg->armors_num;
    // packet.x = msg->position.x;
    // packet.y = msg->position.y;
    // packet.z = msg->position.z;
    // packet.yaw = msg->yaw;
    // packet.vx = msg->velocity.x;
    // packet.vy = msg->velocity.y;
    // packet.vz = msg->velocity.z;
    // packet.v_yaw = msg->v_yaw;
    // packet.r1 = msg->radius_1;
    // packet.r2 = msg->radius_2;
    // packet.dz = msg->dz;
    packet.robo_id = robo_id_unit8_map.at(msg->id);

    RCLCPP_INFO(get_logger(), "[Target Received] tracking=%d, id=%s, position=(%.2f,%.2f,%.2f)", 
                msg->tracking, msg->id.c_str(), msg->position.x, msg->position.y, msg->position.z);    
    
    // Control auto spin based on tracking status
    if (msg->tracking) {
      // Target detected: stop auto spin, use tracking angles
      // Update tracking state ATOMICALLY before logging

      is_tracking_.store(true);
      last_tracking_time_ = this->now();
      packet.pitch = pitch;
      packet.yaw = yaw;
      
      // Send tracking command and CRC
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

      // RCLCPP_INFO(get_logger(), "[Send] id %d!", packet.id);
      RCLCPP_INFO(get_logger(), "[Send Tracking] pitch=%.4f, yaw=%.4f, shoot=%d", 
                  packet.pitch, packet.yaw, packet.shoot);
    } else {
      // No tracking
      is_tracking_.store(false);
      // if (enable_auto_spin_) {
      //   RCLCPP_DEBUG(get_logger(), "[Auto Spin] Letting spinTimerCallback take over");
      //   return;  // Let spinTimerCallback take over
      // } else {
      //   packet.pitch = st.current_pitch;
      //   packet.yaw = st.current_yaw;
      //   crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      // }
    }

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);

    if (abs(aim_x) > 0.01) {
      aiming_point_.header.stamp = this->now();
      aiming_point_.pose.position.x = aim_x;
      aiming_point_.pose.position.y = aim_y;
      aiming_point_.pose.position.z = aim_z;
      marker_pub_->publish(aiming_point_);
    }

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::sendNavigationCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    NavigationPacket packet;
    packet.header.data_length = sizeof(packet) - sizeof(Header) - 2;
    packet.header.cmd_id = 0x405;
    crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&packet.header), sizeof(Header) - 2);

    packet.vx = msg->linear.x;
    packet.vy = msg->linear.y;
    packet.wz = msg->angular.z;

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data(sizeof(packet));
    std::copy(
      reinterpret_cast<const uint8_t *>(&packet),
      reinterpret_cast<const uint8_t *>(&packet) + sizeof(packet), data.begin());

    serial_driver_->port()->send(data);

    RCLCPP_INFO(
      get_logger(), "[发送导航指令] vx:%.2f, vy:%.2f, wz:%.2f", packet.vx, packet.vy, packet.wz);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "发送导航指令失败: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::armorsCallback(auto_aim_interfaces::msg::Armors::SharedPtr msg)
{
  last_armors_msg_ns_.store(this->now().nanoseconds());
  detector_has_armors_.store(!msg->armors.empty());
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

void RMSerialDriver::spinTimerCallback()
{
  // Keep timer frequency unchanged; only block while Armors state is fresh.
  if (detector_has_armors_.load()) {
    const int64_t now_ns = this->now().nanoseconds();
    const int64_t last_msg_ns = last_armors_msg_ns_.load();
    const int64_t timeout_ns = static_cast<int64_t>(armors_timeout_ * 1e9);

    if (last_msg_ns > 0 && (now_ns - last_msg_ns) <= timeout_ns) {
      return;
    }

    detector_has_armors_.store(false);
  }

  // If tracking, do not send spin command
  if (is_tracking_.load()) {
    // Check for tracking timeout
    auto time_since_last_tracking = (this->now() - last_tracking_time_).seconds();
    if (time_since_last_tracking >= tracking_timeout_) {
      // Tracking timeout, resume auto spin
      is_tracking_.store(false);
      // Re-seed spin direction from measured gimbal yaw to keep transition continuous.
      current_spin_yaw_ = st.current_yaw;
      spin_dir_x_ = std::cos(current_spin_yaw_);
      spin_dir_y_ = std::sin(current_spin_yaw_);
    } 
    return;
  }

  // If not tracking, send spin command
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 2000, "[Spin Timer] is_tracking=false, proceeding with auto spin");

  std::lock_guard<std::mutex> lock(send_mutex_);

  // Update spin direction by Z-axis rotation matrix, then recover yaw from the vector.
  const double dtheta = spin_speed_ * spin_timer_period_;
  const double cos_theta = std::cos(dtheta);
  const double sin_theta = std::sin(dtheta);
  const double next_x = cos_theta * spin_dir_x_ - sin_theta * spin_dir_y_;
  const double next_y = sin_theta * spin_dir_x_ + cos_theta * spin_dir_y_;

  spin_dir_x_ = next_x;
  spin_dir_y_ = next_y;

  const double norm = std::hypot(spin_dir_x_, spin_dir_y_);
  if (norm > 1e-9) {
    spin_dir_x_ /= norm;
    spin_dir_y_ /= norm;
  }

  current_spin_yaw_ = std::atan2(spin_dir_y_, spin_dir_x_);

  // Prepare and send spin packet
  Header header;
  SendPacket packet;
  header.data_length = sizeof(packet) - sizeof(header) - 2;
  header.cmd_id = 0x0402;
  crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&header), sizeof(header) - 2);
  packet.header = header;
  packet.pitch = static_cast<float>(spin_pitch_);
  packet.yaw = static_cast<float>(current_spin_yaw_);
  packet.shoot = 0;
  packet.robo_id = 0;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "[Spin Timer SENDING] Auto spin command: pitch=%.4f rad, yaw=%.4f rad", packet.pitch,
    packet.yaw);

  try {
    serial_driver_->port()->send(toVector(packet));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "[Spin Timer] Send error: %s", ex.what());
    reopenPort();
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
