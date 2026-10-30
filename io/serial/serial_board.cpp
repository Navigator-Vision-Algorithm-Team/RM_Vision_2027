#include "serial_board.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "io/serial/gimbal/protocol_crc.hpp"
#include "io/sim/common.hpp"
#include "io/sim/recorder.hpp"
#include "io/sim/transport.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
SerialBoard::SerialBoard(const std::string & config_path)
: bullet_speed(28.0)
{
  auto yaml = tools::load(config_path);

  device_name_ = tools::read<std::string>(yaml, "com_port");
  baud_rate_ = yaml["baud_rate"].as<uint32_t>(921600);
  bullet_speed = yaml["bullet_speed"].as<double>(28.0);
  use_referee_system_ = yaml["use_referee_system"].as<bool>(false);
  game_start_progress_threshold_ = yaml["game_start_progress_threshold"].as<int>(4);

  // 数据源选择：real / fake / replay，由配置决定（业务代码无感知）
  auto sim_config = sim::SimConfig::load(yaml);
  source_mode_ = sim_config.serial.mode;

  if (sim_config.record.enable) {
    recorder_ = std::make_shared<sim::SerialRecorder>(sim_config.record.dir);
  }

  try {
    transport_ = sim::make_transport(sim_config);
    if (!transport_) throw std::runtime_error("make_transport returned null");
    transport_->open();
  } catch (const std::exception & e) {
    tools::logger()->error(
      "[SerialBoard] Failed to open serial source (mode={}): {}", sim::to_string(source_mode_),
      e.what());
    exit(1);
  }

  // Without referee system, auto-aim is always enabled
  if (!use_referee_system_) {
    game_started_.store(true);
  }

  tools::logger()->info(
    "[{}] [SerialBoard] source={} baud={} referee={} record={}",
    sim::log_tag(source_mode_), transport_->describe(), baud_rate_, use_referee_system_,
    recorder_ ? recorder_->path() : std::string("off"));

  // 初始化 IMU 数据为全零姿态，收到真实数据后会被覆盖
  auto t0 = std::chrono::steady_clock::time_point(std::chrono::seconds(0));
  data_ahead_ = IMUData{ImuState{}, t0};
  data_behind_ = IMUData{ImuState{}, t0 + std::chrono::milliseconds(1)};

  receive_thread_ = std::thread(&SerialBoard::receiveThread, this);
  tools::logger()->info("[SerialBoard] Ready (IMU will update when MCU sends 0x502).");
}

SerialBoard::~SerialBoard()
{
  quit_ = true;
  if (receive_thread_.joinable()) receive_thread_.join();

  {
    // 接收线程已退出，回调可以安全释放（其捕获的 ROS 节点 shared_ptr 随之释放）
    std::lock_guard<std::mutex> lock(joint_callback_mutex_);
    joint_callback_ = nullptr;
  }

  if (transport_) transport_->close();
  if (recorder_) recorder_->close();
}

namespace
{
// 归一化到 [0, 2π)，与下位机的上报约定一致
double wrap_2pi(double a)
{
  a = std::fmod(a, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a;
}
}  // namespace

ImuState SerialBoard::imu_state_at(std::chrono::steady_clock::time_point timestamp)
{
  /*
    非阻塞地取出队列里的新样本，保留"上一帧/最新帧"两帧，
    再按图像时间戳在这两帧之间线性插值（与相机曝光对齐）。
    角度插值而不是四元数插值：四个量都是欧拉角，插完再复合更直观，
    yaw 走最短弧，避免 0/2π 边界被插成"绕一圈"。
  */

  update_imu_samples();

  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  const ImuState & a = data_ahead_.state;
  const ImuState & b = data_behind_.state;

  const std::chrono::duration<double> t_ab = data_behind_.timestamp - data_ahead_.timestamp;
  if (t_ab.count() <= 0) return a;  // 尚无两帧可用时退化为最近值

  const std::chrono::duration<double> t_ac = timestamp - data_ahead_.timestamp;
  const double k = std::clamp(t_ac / t_ab, 0.0, 1.0);

  ImuState out;
  out.pitch = a.pitch + k * (b.pitch - a.pitch);
  out.roll = a.roll + k * (b.roll - a.roll);
  out.upper_yaw = wrap_2pi(a.upper_yaw + k * tools::limit_rad(b.upper_yaw - a.upper_yaw));
  out.lower_yaw = wrap_2pi(a.lower_yaw + k * tools::limit_rad(b.lower_yaw - a.lower_yaw));
  return out;
}

Eigen::Quaterniond SerialBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  // 单yaw 兵种的兼容语义：云台 = yaw + pitch + roll（大yaw 的 lower_yaw 即其唯一 yaw）
  const ImuState s = imu_state_at(timestamp);
  const Eigen::Quaterniond q(
    Eigen::AngleAxisd(s.lower_yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(s.pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(s.roll, Eigen::Vector3d::UnitX()));
  return q.normalized();
}

void SerialBoard::update_imu_samples()
{
  IMUData new_data;
  while (queue_.try_pop(new_data)) {
    if (new_data.timestamp > data_behind_.timestamp) {
      data_ahead_ = data_behind_;
      data_behind_ = new_data;
    }
  }
}

void SerialBoard::send(Command command) const
{
  SendPacket packet{};
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
  packet.timeseries = 0;  // 占位（接收方不使用）

  crc16::Append_CRC16_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet), sizeof(SendPacket));

  try {
    std::lock_guard<std::mutex> lock(send_mutex_);
    const size_t n =
      transport_->write(reinterpret_cast<const uint8_t *>(&packet), sizeof(SendPacket));
    if (recorder_ && n > 0)
      recorder_->record_tx(reinterpret_cast<const uint8_t *>(&packet), n);

    // 非真实模式下按 1Hz 记录下发内容，便于验收时区分真实/伪数据
    // 注意：不可用 time_point::min() 初始化，相减会整数溢出导致该日志永不触发
    static bool first_log = true;
    static auto last_log = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (source_mode_ != sim::SourceMode::Real &&
        (first_log || std::chrono::duration<double>(now - last_log).count() >= 1.0)) {
      first_log = false;
      last_log = now;
      tools::logger()->info(
        "[{}] [SerialBoard] TX 0x0402 yaw={:.3f} pitch={:.3f} shoot={}", sim::log_tag(source_mode_),
        command.yaw, command.pitch, static_cast<int>(packet.shoot));
    }
  } catch (const std::exception & e) {
    tools::logger()->warn("[SerialBoard] Failed to write source: {}", e.what());
  }
}

void SerialBoard::send(NavCommand command) const
{
  NavigationPacket packet{};
  packet.header.header = 0xA5;
  packet.header.data_length = sizeof(NavigationPacket) - sizeof(Header) - 2;
  packet.header.seq = 0;
  packet.header.cmd_id = 0x0405;
  // 导航 0x0405 与自瞄 0x0402 共用同一套 RM 标准 CRC（init 0xFF / 0xFFFF）
  crc8::Append_CRC8_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet.header), sizeof(Header) - 2);

  packet.vx = static_cast<float>(command.vx);
  packet.vy = static_cast<float>(command.vy);
  packet.wz = static_cast<float>(command.wz);

  crc16::Append_CRC16_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet), sizeof(NavigationPacket));

  try {
    std::lock_guard<std::mutex> lock(send_mutex_);
    const size_t n =
      transport_->write(reinterpret_cast<const uint8_t *>(&packet), sizeof(NavigationPacket));
    if (recorder_ && n > 0)
      recorder_->record_tx(reinterpret_cast<const uint8_t *>(&packet), n);
  } catch (const std::exception & e) {
    tools::logger()->warn("[SerialBoard] Failed to write source: {}", e.what());
  }
}

void SerialBoard::set_gimbal_joint_callback(GimbalJointCallback callback)
{
  std::lock_guard<std::mutex> lock(joint_callback_mutex_);
  joint_callback_ = std::move(callback);
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

  // tools::logger()->debug(
  //   "[SerialBoard] GameStatus: type={} progress={} remain={} sync={}",
  //   gs.game_type, gs.game_progress, gs.stage_remain_time, gs.sync_timestamp);
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

  // tools::logger()->debug(
  //   "[SerialBoard] RobotStatus: id={} lv={} hp={}/{} cool={} heat={} power={} mains=g{}c{}s{}",
  //   rs.robot_id, rs.robot_level, rs.remain_hp, rs.max_hp,
  //   rs.shooter_cooling_rate, rs.shooter_heat_limit, rs.chassis_power_limit,
  //   rs.mains_power_gimbal, rs.mains_power_chassis, rs.mains_power_shooter);
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
    const size_t n = transport_->read(buffer, size, 50);
    if (recorder_ && n > 0) recorder_->record_rx(buffer, n);
    return n == size;
  } catch (const std::exception & e) {
    return false;
  }
}

void SerialBoard::receiveThread()
{
  tools::logger()->info("[SerialBoard] receiveThread started.");
  int error_count = 0;
  int valid_packet_count = 0;
  bool first_imu_received = false;
  bool first_valid_header = false;

  std::vector<uint8_t> flag(1);
  std::vector<uint8_t> headdata;
  std::vector<uint8_t> data;

  auto last_status_time = std::chrono::steady_clock::now();

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[SerialBoard] Too many errors, reconnecting...");
      reconnect();
      continue;
    }

    // 每 10 秒报告一次串口状态（直到收到第一个 IMU 包为止） -- 该功能废止
    auto now = std::chrono::steady_clock::now();
    auto since_last = std::chrono::duration_cast<std::chrono::seconds>(now - last_status_time).count();
    // if (!first_imu_received && since_last >= 10) {
    //   tools::logger()->warn(
    //     "[SerialBoard] No IMU (0x502) yet — {} packets seen, {} read errors (using identity q)",
    //     valid_packet_count, error_count);
    //   last_status_time = now;
    // }

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
          reinterpret_cast<const uint8_t *>(&header), sizeof(header) - 2)) {
      continue;
    }

    valid_packet_count++;

    if (!first_valid_header) {
      first_valid_header = true;
      uint16_t first_cmd_id = header.cmd_id;
      // tools::logger()->info("[SerialBoard] First valid packet received: cmd_id=0x{:x}", first_cmd_id);
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

        // 双yaw：lower = 大yaw（IMU 与四个全向相机所在轴），upper = 小yaw（主相机所在轴）
        // packed 结构体的字段不能直接绑定到引用，先拷到局部变量（fmt 也受益）
        const float pitch = imu_packet.pitch;
        const float roll = imu_packet.roll;
        const double lower = imu_packet.lower_yaw;
        const double upper = imu_packet.upper_yaw;
        // psi 取最短弧，避免 0/2π 边界（如 lower=350°、upper=10°）被算成 -340°
        const double psi = tools::limit_rad(upper - lower);

        // —— 云台关节反馈（ROS2 JointState）——
        // 每帧新采样回调一次，绝不在没收到新帧时重复包装旧角度。
        // 注意：0x502 目前只上报「以上电时车头朝向为 0」的世界系绝对 lower_yaw，
        // 没有底盘 yaw / 相对关节角字段，因此这里暂时用绝对角充当关节角。
        // 导航侧要求的是「相对底盘」，需下位机在 0x502 补 float chassis_yaw
        // （或直接补 relative_yaw，包长 26 -> 30 字节），补齐后把下面改成
        // tools::limit_rad(lower - chassis_yaw) 即可，其余代码无需改动。
        // 大yaw 轴没有俯仰自由度，pitch 恒为 0。
        {
          GimbalJointCallback callback;
          {
            std::lock_guard<std::mutex> lock(joint_callback_mutex_);
            callback = joint_callback_;
          }
          if (callback) {
            try {
              callback(tools::limit_rad(lower), 0.0);
            } catch (const std::exception & e) {
              tools::logger()->warn("[SerialBoard] Gimbal joint callback failed: {}", e.what());
            }
          }
        }

        if (!first_imu_received) {
          first_imu_received = true;
          tools::logger()->info(
            "[SerialBoard] First IMU packet received (pitch={:.3f} roll={:.3f} "
            "upper_yaw={:.3f} lower_yaw={:.3f} psi={:.3f} rad) — unblocking",
            pitch, roll, upper, lower, psi);

          // 防呆：视觉侧一律用 lower_yaw 当云台(大yaw)角。若固件把单yaw 平台的
          // 唯一 yaw 填进了 upper_yaw，云台会被当成"从不转动"，这里提前报出来。
          if (std::abs(upper) > 1e-3 && std::abs(lower) < 1e-3) {
            tools::logger()->warn(
              "[SerialBoard] upper_yaw 非零而 lower_yaw 为 0：单yaw 兵种若把 yaw 填在 "
              "upper_yaw，云台姿态会恒为 0，请与固件确认字段归属");
          }
        }

        error_count = 0;
        auto t_now = std::chrono::steady_clock::now();

        // 只存原始角度，各坐标系姿态由调用方 / tf 树复合：
        //   大yaw  = Rz(lower)                          （纯 yaw，无 pitch/roll）
        //   小yaw  = Rz(upper)·Ry(pitch)·Rx(roll)       （主相机所在板）
        queue_.push({ImuState{pitch, roll, upper, lower}, t_now});
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
      if (transport_) transport_->close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      if (!transport_) throw std::runtime_error("null transport");
      transport_->open();
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
