// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <chrono>
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

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  // Auto spin parameters and timer
  enable_auto_spin_ = this->declare_parameter("enable_auto_spin", true);
  spin_speed_ = this->declare_parameter("spin_speed", 1.0);
  spin_timer_period_ = this->declare_parameter("spin_timer_period", 0.01);
  current_spin_yaw_ = 0.0;

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
                RCLCPP_ERROR(get_logger(), "Data CRC error!");
                break;
              }
              // RCLCPP_INFO(get_logger(), "[Receive] id %d!", imu_packet.id);
              // RCLCPP_INFO(get_logger(), "[Receive] roll %f!", imu_packet.roll);
              // RCLCPP_INFO(get_logger(), "[Receive] pitch %f!", imu_packet.pitch);
              // RCLCPP_INFO(get_logger(), "[Receive] yaw %f!", imu_packet.yaw);

              RCLCPP_WARN(
                get_logger(), "[Receive] roboid %d! color %d", imu_packet.roboid,
                imu_packet.roboid > 100 ? 0 : 1);
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

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  const static std::map<std::string, uint8_t> robo_id_unit8_map{
    {"", 0x000},  {"outpost", 0x010}, {"1", 0x001},     {"2", 0x002},   {"3", 0x003},
    {"4", 0x004}, {"5", 0x005},       {"guard", 0x007}, {"base", 0x011}};

  try {
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
    
    // Auto-rotate when no target (tracking=false), stop when target found (tracking=true)
    if (msg->tracking) {
      // 检测到目标：停止旋转，使用弹道解算的 pitch/yaw 跟踪目标
      packet.pitch = pitch;
      packet.yaw = yaw;
      packet.shoot = 0;  // Stop rotation, start tracking
    } else {
      // 未检测到目标：继续旋转搜索
      static float search_yaw = 0.0f;
      search_yaw += 0.5f;  // Continuous rotation
      if (search_yaw > M_PI) search_yaw -= 2 * M_PI;
      packet.pitch = 0.0f;
      packet.yaw = search_yaw;
      packet.shoot = 1;  // Continue searching
    }
    
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    // RCLCPP_INFO(get_logger(), "[Send] id %d!", packet.id);
    RCLCPP_INFO(get_logger(), "[Send] pitch %f!", packet.pitch);
    RCLCPP_INFO(get_logger(), "[Send] yaw %f!", packet.yaw);
    // RCLCPP_INFO(get_logger(), "[Send] accuracy %d!", packet.accuracy);
    RCLCPP_INFO(get_logger(), "[Send] shoot %d!", packet.shoot);

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
  if (!serial_driver_ || !serial_driver_->port() || !serial_driver_->port()->is_open()) {
    return;
  }

  current_spin_yaw_ += spin_speed_ * spin_timer_period_;
  while (current_spin_yaw_ > M_PI) {
    current_spin_yaw_ -= 2 * M_PI;
  }
  while (current_spin_yaw_ < -M_PI) {
    current_spin_yaw_ += 2 * M_PI;
  }

  Header header;
  SendPacket packet;
  header.data_length = sizeof(packet) - sizeof(header) - 2;
  header.cmd_id = 0x0402;
  crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&header), sizeof(header) - 2);
  packet.header = header;
  packet.pitch = 0.0f;
  packet.yaw = static_cast<float>(current_spin_yaw_);
  packet.shoot = 0;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

  try {
    serial_driver_->port()->send(toVector(packet));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 20, "Auto spin send failed: %s", ex.what());
    reopenPort();
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
