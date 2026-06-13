#include "serial_board.hpp"

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
  use_referee_system_ = yaml["use_referee_system"].as<bool>(false);
  game_start_progress_threshold_ = yaml["game_start_progress_threshold"].as<int>(4);

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

  // Without referee system, auto-aim is always enabled
  if (!use_referee_system_) {
    game_started_.store(true);
  }

  tools::logger()->info(
    "[SerialBoard] Serial port {} opened, baud={}, referee={}",
    device_name_, baud_rate_, use_referee_system_);

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

GameStatus SerialBoard::game_status() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return game_status_;
}

RobotStatus SerialBoard::robot_status() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return robot_status_;
}

bool SerialBoard::is_game_started() const
{
  return game_started_.load();
}

bool SerialBoard::reset_pending()
{
  return reset_pending_.exchange(false);
}

void SerialBoard::handleGameStatus(const GameStatusPacket & pkt)
{
  GameStatus gs;
  gs.game_type = static_cast<uint8_t>(pkt.game_info & 0x0F);
  gs.game_progress = static_cast<uint8_t>((pkt.game_info >> 4) & 0x0F);
  gs.stage_remain_time = pkt.stage_remain_time;
  gs.sync_timestamp = pkt.sync_timestamp;

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    game_status_ = gs;
  }

  // Update game-started state based on progress
  if (use_referee_system_) {
    bool started = gs.game_progress >= static_cast<uint8_t>(game_start_progress_threshold_);
    bool was_started = game_started_.exchange(started);

    if (started && !was_started) {
      tools::logger()->info("[SerialBoard] Game started (progress={})", gs.game_progress);
    } else if (!started && was_started) {
      tools::logger()->info("[SerialBoard] Game ended / returned to pre-game");
    }
  }

  tools::logger()->debug(
    "[SerialBoard] GameStatus: type={} progress={} remain={} sync={}",
    gs.game_type, gs.game_progress, gs.stage_remain_time, gs.sync_timestamp);
}

void SerialBoard::handleGameRobotStatus(const GameRobotStatusPacket & pkt)
{
  RobotStatus rs;
  rs.robot_id = pkt.robot_id;
  rs.robot_level = pkt.robot_level;
  rs.remain_hp = pkt.remain_hp;
  rs.max_hp = pkt.max_hp;
  rs.shooter_cooling_rate = pkt.shooter_cooling_rate;
  rs.shooter_heat_limit = pkt.shooter_heat_limit;
  rs.chassis_power_limit = pkt.chassis_power_limit;
  rs.mains_power_gimbal = (pkt.mains_power_state & 0x01) != 0;
  rs.mains_power_chassis = ((pkt.mains_power_state >> 1) & 0x01) != 0;
  rs.mains_power_shooter = ((pkt.mains_power_state >> 2) & 0x01) != 0;

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    robot_status_ = rs;
  }

  tools::logger()->debug(
    "[SerialBoard] RobotStatus: id={} lv={} hp={}/{} cool={} heat={} power={} mains=g{}c{}s{}",
    rs.robot_id, rs.robot_level, rs.remain_hp, rs.max_hp,
    rs.shooter_cooling_rate, rs.shooter_heat_limit, rs.chassis_power_limit,
    rs.mains_power_gimbal, rs.mains_power_chassis, rs.mains_power_shooter);
}

void SerialBoard::handleBass(const bassPacket & pkt)
{
  if (pkt.reset) {
    reset_pending_.store(true);
    tools::logger()->info("[SerialBoard] Reset signal received from MCU");
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
        if (!read(data.data(), data.size())) { error_count++; break; }
        data.insert(data.begin(), headdata.begin(), headdata.end());
        IMUPacket imu_packet = fromVector<IMUPacket>(data);

        if (!crc16::Verify_CRC16_Check_Sum(
              reinterpret_cast<const uint8_t *>(&imu_packet), sizeof(imu_packet))) {
          break;
        }

        error_count = 0;
        auto t_now = std::chrono::steady_clock::now();

        Eigen::Quaterniond q =
          Eigen::AngleAxisd(imu_packet.yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(imu_packet.pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(imu_packet.roll, Eigen::Vector3d::UnitX());

        queue_.push({q, t_now});
        break;
      }
      case 0x503: {
        data.resize(sizeof(classisPacket) - sizeof(Header));
        if (!read(data.data(), data.size())) { error_count++; break; }
        data.insert(data.begin(), headdata.begin(), headdata.end());
        classisPacket pkt = fromVector<classisPacket>(data);
        if (!crc16::Verify_CRC16_Check_Sum(
              reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt))) break;
        error_count = 0;
        break;
      }
      case 0x504: {
        data.resize(sizeof(bottonPacket) - sizeof(Header));
        if (!read(data.data(), data.size())) { error_count++; break; }
        data.insert(data.begin(), headdata.begin(), headdata.end());
        bottonPacket pkt = fromVector<bottonPacket>(data);
        if (!crc16::Verify_CRC16_Check_Sum(
              reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt))) break;
        error_count = 0;
        break;
      }
      case 0x505: {
        data.resize(sizeof(bassPacket) - sizeof(Header));
        if (!read(data.data(), data.size())) { error_count++; break; }
        data.insert(data.begin(), headdata.begin(), headdata.end());
        bassPacket pkt = fromVector<bassPacket>(data);
        if (!crc16::Verify_CRC16_Check_Sum(
              reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt))) break;
        error_count = 0;
        handleBass(pkt);
        break;
      }
      case 0x0001:
      case 0x0100: {
        data.resize(sizeof(GameStatusPacket) - sizeof(Header));
        if (!read(data.data(), data.size())) { error_count++; break; }
        data.insert(data.begin(), headdata.begin(), headdata.end());
        GameStatusPacket pkt = fromVector<GameStatusPacket>(data);
        if (!crc16::Verify_CRC16_Check_Sum(
              reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt))) break;
        error_count = 0;
        handleGameStatus(pkt);
        break;
      }
      case 0x0201:
      case 0x0102: {
        data.resize(sizeof(GameRobotStatusPacket) - sizeof(Header));
        if (!read(data.data(), data.size())) { error_count++; break; }
        data.insert(data.begin(), headdata.begin(), headdata.end());
        GameRobotStatusPacket pkt = fromVector<GameRobotStatusPacket>(data);
        if (!crc16::Verify_CRC16_Check_Sum(
              reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt))) break;
        error_count = 0;
        handleGameRobotStatus(pkt);
        break;
      }
      default: {
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
