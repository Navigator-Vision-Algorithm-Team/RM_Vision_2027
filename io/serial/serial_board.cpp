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
  // 到热量上限前 heat_stop_margin 就停火；回落到 limit - 2*margin 后恢复。
  // 17mm 单发热量约 10，默认 10 即"再打一发就顶到上限"时收手。
  heat_stop_margin_ = yaml["heat_stop_margin"].as<int>(10);

  // 0x0503 的 yaw_angle 是 IMU 世界系 yaw。哨兵的双yaw 平台把 IMU 装在小yaw（相机）板上，
  // 因此默认 upper；若实车 IMU 在大yaw 轴上，配成 lower。
  const std::string imu_axis = yaml["gimbal_imu_axis"].as<std::string>("upper");
  imu_on_upper_axis_ = (imu_axis != "lower");

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

  // 解包器每解出一帧就回调：接收线程内同步执行，因此业务状态无需额外加锁即可读写
  link_.on_frame = [this](uint16_t cmd_id) { on_frame(cmd_id); };

  tools::logger()->info(
    "[{}] [SerialBoard] source={} baud={} referee={} imu_axis={} record={}",
    sim::log_tag(source_mode_), transport_->describe(), baud_rate_, use_referee_system_,
    imu_on_upper_axis_ ? "upper" : "lower", recorder_ ? recorder_->path() : std::string("off"));

  // 初始化 IMU 数据为全零姿态，收到真实数据后会被覆盖
  auto t0 = std::chrono::steady_clock::time_point(std::chrono::seconds(0));
  data_ahead_ = IMUData{ImuState{}, t0};
  data_behind_ = IMUData{ImuState{}, t0 + std::chrono::milliseconds(1)};

  receive_thread_ = std::thread(&SerialBoard::receiveThread, this);
  tools::logger()->info("[SerialBoard] Ready (IMU will update when MCU sends 0x0503).");
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

  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(SendPacket));

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
  packet.vy = -packet.vy;
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

void SerialBoard::on_frame(uint16_t cmd_id)
{
  using namespace referee_comm;
  switch (cmd_id) {
    case host_comm::HOST_VERSION: {
      const uint32_t uptime = link_.host.version.uptime_ms;
      // uptime 回落视为 MCU 重启：视觉侧状态一并复位（等价旧 0x505 的 reset 位）
      if (last_uptime_ms_ != 0 && uptime < last_uptime_ms_) {
        reset_pending_.store(true);
        tools::logger()->info(
          "[SerialBoard] MCU restart detected (uptime {} -> {} ms), reset vision state",
          last_uptime_ms_, uptime);
      }
      last_uptime_ms_ = uptime;

      if (!config_logged_) {
        config_logged_ = true;
        // packed 结构体的字段不能直接绑定到 fmt 的引用参数，先拷到局部变量
        const uint8_t version = link_.host.version.host_version;
        const uint16_t season = link_.host.version.referee_season;
        const uint8_t robot_type = link_.host.version.robot_type;
        const uint8_t chassis_type = link_.host.version.chassis_type;
        const uint8_t gimbal_type = link_.host.version.gimbal_type;
        tools::logger()->info(
          "[SerialBoard] MCU handshake: host_protocol=v{} referee_season={} robot_type={} "
          "chassis_type={} gimbal_type={}",
          version, season, robot_type, chassis_type, gimbal_type);
        if (version != host_comm::HOST_PROTOCOL_VERSION) {
          tools::logger()->warn(
            "[SerialBoard] host 协议版本不一致：固件 {} / 上位机 {}，字段可能错位", version,
            host_comm::HOST_PROTOCOL_VERSION);
        }
        if (season != 2027) {
          tools::logger()->warn(
            "[SerialBoard] 裁判转发帧按 2027 赛季解析，固件上报 referee_season={}", season);
        }
      }
      break;
    }
    case host_comm::GIMBAL_STATUS:
      handle_gimbal(link_.host.gimbal);
      break;
    case v2027::GAME_STATUS:
      handle_game_status(link_.referee.game_status);
      break;
    case v2027::ROBOT_STATUS:
      handle_robot_status(link_.referee.robot_status);
      break;
    case v2027::POWER_HEAT_DATA:
      handle_power_heat(link_.referee.power_heat_data);
      break;
    default:
      // 其余上行/转发帧已经解到 link_ 里，业务侧需要时直接读，不在这里做副作用
      break;
  }
}

void SerialBoard::handle_gimbal(const host_comm::host_gimbal_status_t & pkt)
{
  /*
    0x0503 只给「IMU 世界系姿态 + 小yaw 关节角」，双yaw 的两个轴要自己复合：
      · yaw_angle 是 IMU（相机/枪管）世界系 yaw，即小yaw 板的世界朝向
      · upper_yaw_angle 是小yaw 相对大yaw 的关节角
    因此 小yaw 世界角 = yaw_angle，大yaw 世界角 = yaw_angle - upper_yaw_angle。
    单yaw 构型的 upper_yaw_angle 恒为 0，两个角自动相等。
  */
  const double pitch = pkt.pitch_angle;
  const double roll = pkt.roll_angle;
  const double imu_yaw = pkt.yaw_angle;

  double upper = imu_yaw;
  double lower = imu_yaw;
  if (link_.host.version.gimbal_type == 2) {  // gimbal_DoubleYaw
    if (imu_on_upper_axis_) {
      upper = imu_yaw;
      lower = imu_yaw - pkt.upper_yaw_angle;
    } else {
      lower = imu_yaw;
      upper = imu_yaw + pkt.upper_yaw_angle;
    }
  }

  upper = wrap_2pi(upper);
  lower = wrap_2pi(lower);

  // —— 云台关节反馈（ROS2 JointState）——
  // 每帧新采样回调一次，绝不在没收到新帧时重复包装旧角度。
  // 大yaw 轴没有俯仰自由度，pitch 恒为 0。
  // 注意：这里下发的是世界系绝对角；导航侧要的「相对底盘」可用 0x0502 的
  // feedback_angle 修正为 tools::limit_rad(lower - chassis_yaw)，待导航接口定稿再接。
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

  if (!first_imu_received_) {
    first_imu_received_ = true;
    const double joint = pkt.upper_yaw_angle;
    const uint8_t gimbal_type = link_.host.version.gimbal_type;
    tools::logger()->info(
      "[SerialBoard] First gimbal frame received (gimbal_type={} pitch={:.3f} roll={:.3f} "
      "yaw={:.3f} upper_yaw_angle={:.3f} -> upper={:.3f} lower={:.3f} rad) — unblocking",
      gimbal_type, pitch, roll, imu_yaw, joint, upper, lower);
  }

  // 只存原始角度，各坐标系姿态由调用方 / tf 树复合：
  //   大yaw  = Rz(lower)                          （纯 yaw，无 pitch/roll）
  //   小yaw  = Rz(upper)·Ry(pitch)·Rx(roll)       （主相机所在板）
  queue_.push({ImuState{pitch, roll, upper, lower}, std::chrono::steady_clock::now()});
}

void SerialBoard::handle_game_status(const referee_comm::v2027::game_status_t & pkt)
{
  GameStatus gs;
  gs.game_type = static_cast<uint8_t>(pkt.game_type);
  gs.game_progress = static_cast<uint8_t>(pkt.game_progress);
  gs.stage_remain_time = pkt.stage_remain_time;
  gs.sync_timestamp = pkt.SyncTimeStamp;

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
}

void SerialBoard::handle_robot_status(const referee_comm::v2027::robot_status_t & pkt)
{
  RobotStatus rs;
  rs.robot_id = pkt.robot_id;
  rs.robot_level = pkt.robot_level;
  rs.remain_hp = pkt.current_HP;
  rs.max_hp = pkt.maximum_HP;
  rs.shooter_cooling_rate = pkt.shooter_barrel_cooling_value;
  rs.shooter_heat_limit = pkt.shooter_barrel_heat_limit;
  rs.chassis_power_limit = pkt.chassis_power_limit;
  rs.bullet_speed_limit = pkt.bullet_speed_limit;
  rs.mains_power_gimbal = pkt.power_management_gimbal_output != 0;
  rs.mains_power_chassis = pkt.power_management_chassis_output != 0;
  rs.mains_power_shooter = pkt.power_management_shooter_output != 0;

  bool new_team = false;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (robot_id_ != pkt.robot_id) {
      robot_id_ = pkt.robot_id;
      new_team = true;
    }
    heat_limit_ = pkt.shooter_barrel_heat_limit;
    robot_status_ = rs;
  }

  if (new_team) {
    // 红方 1~11，蓝方 101~111（蓝方 = 红方 + 100），2/102 为保留位
    if (referee_comm::v2027::IsValidId(robot_id_)) {
      const bool own_red = referee_comm::v2027::IsRedId(robot_id_);
      tools::logger()->info(
        "[SerialBoard] 机器人 ID = {}（{}方{}）-> 敌人{}方（enemy_color 配置值仅作兜底）",
        robot_id_, own_red ? "红" : "蓝", referee_comm::v2027::RobotRoleName(robot_id_),
        own_red ? "蓝" : "红");
    } else if (robot_id_ != 0) {
      tools::logger()->warn(
        "[SerialBoard] 收到非法 robot_id={}（应为红 1~11 / 蓝 101~111），敌我颜色继续用配置值",
        robot_id_);
    }
  }

  update_heat_gate();
}

void SerialBoard::handle_power_heat(const referee_comm::v2027::power_heat_data_t & pkt)
{
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    barrel_heat_ = pkt.shooter_17mm_barrel_heat;
  }
  update_heat_gate();
}

void SerialBoard::update_heat_gate()
{
  const int limit = heat_limit_;
  const int heat = barrel_heat_;

  // 没有裁判数据（limit=0）时不限火，避免离线/仿真时开不出枪
  if (limit <= 0) {
    if (heat_blocked_.exchange(false))
      tools::logger()->info("[SerialBoard] 热量门限解除（无热量上限数据）");
    return;
  }

  const int margin = std::max(0, heat_stop_margin_);
  const int stop_at = limit - margin;
  const int resume_at = std::max(0, limit - 2 * margin);

  if (!heat_blocked_.load() && heat >= stop_at) {
    heat_blocked_.store(true);
    tools::logger()->info(
      "[SerialBoard] 枪管热量 {}/{} 已达停火线 {}，暂停开火（冷却 {} 后恢复）", heat, limit,
      stop_at, resume_at);
  } else if (heat_blocked_.load() && heat <= resume_at) {
    heat_blocked_.store(false);
    tools::logger()->info(
      "[SerialBoard] 枪管热量 {}/{} 已回落到 {} 以下，恢复开火", heat, limit, resume_at);
  }
}

uint8_t SerialBoard::robot_id() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return robot_id_;
}

std::optional<bool> SerialBoard::own_team_red() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  // 红方 1~11、蓝方 101~111；0 与越界值一律当作"未知"，交给调用方用配置兜底
  if (referee_comm::v2027::IsRedId(robot_id_)) return true;
  if (referee_comm::v2027::IsBlueId(robot_id_)) return false;
  return std::nullopt;
}

uint16_t SerialBoard::barrel_heat() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return barrel_heat_;
}

uint16_t SerialBoard::heat_limit() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return heat_limit_;
}

bool SerialBoard::can_shoot() const
{
  return !heat_blocked_.load();
}

void SerialBoard::receiveThread()
{
  tools::logger()->info("[SerialBoard] receiveThread started.");

  std::vector<uint8_t> buffer(1024);
  int silence_count = 0;
  auto last_warn = std::chrono::steady_clock::now();

  while (!quit_) {
    size_t n = 0;
    try {
      n = transport_->read(buffer.data(), buffer.size(), 50);
    } catch (const std::exception &) {
      n = 0;
    }

    if (n > 0) {
      silence_count = 0;
      if (recorder_) recorder_->record_rx(buffer.data(), n);
      // 分帧、CRC 与解包全部在 Receiver 里完成，解出的帧经 link_.on_frame 回调业务处理
      receiver_.Feed(buffer.data(), n);
      continue;
    }

    if (++silence_count > 5000) {
      tools::logger()->warn("[SerialBoard] No bytes for a long time, reconnecting...");
      silence_count = 0;
      reconnect();
      continue;
    }

    // 链路上一直没解出合法帧时给一次提示，避免"接错线但串口能开"时完全静默
    const auto now = std::chrono::steady_clock::now();
    if (receiver_.stats().frames == 0 &&
        std::chrono::duration<double>(now - last_warn).count() >= 10.0) {
      last_warn = now;
      tools::logger()->warn(
        "[SerialBoard] No valid frame yet (crc8_err={} crc16_err={} garbage={}B) — "
        "检查接线/TX-RX 交叉/波特率/下位机是否上电",
        receiver_.stats().crc8_errors, receiver_.stats().crc16_errors,
        receiver_.stats().garbage_bytes);
    }
  }

  tools::logger()->info(
    "[SerialBoard] receiveThread stopped (frames={} crc8_err={} crc16_err={} garbage={}B).",
    receiver_.stats().frames, receiver_.stats().crc8_errors, receiver_.stats().crc16_errors,
    receiver_.stats().garbage_bytes);
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
      // 重连后旧半包已经没有意义，清掉重新同步；统计量保留便于定位问题
      receiver_.Reset();
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
