#include "serial_board.hpp"

#include "io/gimbal/packet.hpp"
#include "io/gimbal/protocol_crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
SerialBoard::SerialBoard(const std::string & config_path)
: mode(Mode::auto_aim),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(28.0),
  ft_angle(0)
{
  auto yaml = tools::load(config_path);

  device_name_ = tools::read<std::string>(yaml, "com_port");
  baud_rate_ = yaml["baud_rate"].as<uint32_t>(921600);
  bullet_speed = yaml["bullet_speed"].as<double>(28.0);

  try {
    serial_.setPort(device_name_);
    serial_.setBaudrate(baud_rate_);
    serial_.setBytesize(serial::eightbits);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[SerialBoard] Failed to open serial: {}", e.what());
    exit(1);
  }

  tools::logger()->info(
    "[SerialBoard] Serial port {} opened, baud_rate={}", device_name_, baud_rate_);

  receive_thread_ = std::thread(&SerialBoard::receiveThread, this);

  tools::logger()->info("[SerialBoard] Waiting for first quaternion...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[SerialBoard] Ready.");
}

SerialBoard::~SerialBoard()
{
  quit_ = true;
  if (receive_thread_.joinable()) receive_thread_.join();
  serial_.close();
}

Eigen::Quaterniond SerialBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void SerialBoard::send(Command command) const
{
  SendPacket packet;
  packet.header.header = 0xA5;
  packet.header.data_length = sizeof(SendPacket) - sizeof(Header) - 2;
  packet.header.seq = 0;
  packet.header.cmd_id = 0x0402;
  crc8::Append_CRC8_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet.header), sizeof(Header) - 2);

  packet.pitch = static_cast<float>(command.pitch);
  packet.yaw = static_cast<float>(command.yaw);
  packet.shoot = (command.control && command.shoot) ? 1 : 0;
  packet.robo_id = 0;

  crc16::Append_CRC16_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet), sizeof(SendPacket));

  try {
    std::lock_guard<std::mutex> lock(send_mutex_);
    serial_.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(SendPacket));
  } catch (const std::exception & e) {
    tools::logger()->warn("[SerialBoard] Failed to write serial: {}", e.what());
  }
}

bool SerialBoard::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    return false;
  }
}

void SerialBoard::receiveThread()
{
  tools::logger()->info("[SerialBoard] receiveThread started.");
  int error_count = 0;

  std::vector<uint8_t> flag(1);
  std::vector<uint8_t> headdata;
  std::vector<uint8_t> data;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[SerialBoard] Too many errors, reconnecting...");
      reconnect();
      continue;
    }

    if (!read(flag.data(), 1)) {
      error_count++;
      continue;
    }

    if (flag[0] != 0xA5) continue;

    headdata.resize(sizeof(Header) - 1);
    if (!read(headdata.data(), headdata.size())) {
      error_count++;
      continue;
    }

    headdata.insert(headdata.begin(), flag[0]);
    Header header = fromVector<Header>(headdata);

    if (!crc8::Verify_CRC8_Check_Sum(
          reinterpret_cast<const uint8_t *>(&header), sizeof(header))) {
      continue;
    }

    switch (header.cmd_id) {
      case 0x502: {
        data.resize(sizeof(IMUPacket) - sizeof(Header));
        if (!read(data.data(), data.size())) {
          error_count++;
          break;
        }
        data.insert(data.begin(), headdata.begin(), headdata.end());
        IMUPacket imu_packet = fromVector<IMUPacket>(data);

        if (!crc16::Verify_CRC16_Check_Sum(
              reinterpret_cast<const uint8_t *>(&imu_packet), sizeof(imu_packet))) {
          break;
        }

        error_count = 0;

        auto t_now = std::chrono::steady_clock::now();

        // Convert Euler angles to quaternion (RPY: yaw about Z, pitch about Y, roll about X)
        Eigen::Quaterniond q =
          Eigen::AngleAxisd(imu_packet.yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(imu_packet.pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(imu_packet.roll, Eigen::Vector3d::UnitX());

        queue_.push({q, t_now});

        // Update bullet_speed from YAML value (static config)
        // Mode stays as auto_aim for the sentry
        break;
      }
      default: {
        // Discard unknown packet payload
        uint16_t payload_len = header.data_length;
        if (payload_len > 0 && payload_len < 256) {
          data.resize(payload_len);
          read(data.data(), data.size());
        }
        break;
      }
    }
  }

  tools::logger()->info("[SerialBoard] receiveThread stopped.");
}

void SerialBoard::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn(
      "[SerialBoard] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.setBaudrate(baud_rate_);
      serial_.open();
      queue_.clear();
      tools::logger()->info("[SerialBoard] Reconnected successfully.");
      return;
    } catch (const std::exception & e) {
      tools::logger()->warn("[SerialBoard] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
