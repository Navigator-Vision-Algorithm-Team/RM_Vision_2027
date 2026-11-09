#include "io/sim/virtual_mcu.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "io/serial/protocol/host_protocol.hpp"
#include "io/serial/protocol/referee.hpp"
#include "io/serial/protocol/seasons/2027/protocol.hpp"
#include "tools/logger.hpp"

namespace io::sim
{

namespace
{
void log_fault(const char * tag, const char * what, long long n)
{
  tools::logger()->warn("[FAULT:{}] {} (count={})", tag, what, n);
}

std::chrono::steady_clock::duration capped(const std::chrono::steady_clock::duration & remain)
{
  const auto cap =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(2));
  return remain < cap ? remain : cap;
}
}  // namespace

// ============================ VirtualMcu ============================
VirtualMcu::VirtualMcu(
  const SerialSourceConfig & cfg, const FaultConfig & fault, int seed, uint8_t robot_id)
: cfg_(cfg), fault_(fault), injector_(fault, seed), robot_id_(robot_id)
{
  if (cfg_.imu_hz <= 0) cfg_.imu_hz = 500.0;
  if (cfg_.chassis_hz <= 0) cfg_.chassis_hz = 50.0;
  if (cfg_.status_hz <= 0) cfg_.status_hz = 1.0;
}

VirtualMcu::~VirtualMcu() { stop(); }

void VirtualMcu::start(const Sink & sink)
{
  quit_ = false;
  thread_ = std::thread([this, sink] { run(sink); });
}

void VirtualMcu::stop()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
}

void VirtualMcu::run(const Sink & sink)
{
  tools::logger()->info(
    "[FAKE] VirtualMcu started: gimbal={}Hz chassis={}Hz status={}Hz reset={}s robot_id={} "
    "faults={}",
    cfg_.imu_hz, cfg_.chassis_hz, cfg_.status_hz, cfg_.reset_period_s, robot_id_,
    fault_.enabled() ? "on" : "off");

  const double gimbal_period = 1.0 / cfg_.imu_hz;
  const double chassis_period = 1.0 / cfg_.chassis_hz;
  const double status_period = 1.0 / cfg_.status_hz;
  const double slow_period = 1.0;  // 0x0500 / 0x0501 都是 1Hz

  const auto t0 = SteadyClock::now();
  double next_gimbal = 0.0, next_chassis = 0.0, next_status = 0.0, next_slow = 0.0;
  double next_reset = cfg_.reset_period_s > 0 ? cfg_.reset_period_s : 1e18;
  auto last_log = t0;

  auto rel_now = [&t0] { return std::chrono::duration<double>(SteadyClock::now() - t0).count(); };

  while (!quit_) {
    const double rel = rel_now();
    t_now_ = rel;
    uptime_ms_ = static_cast<uint32_t>(rel * 1000.0);

    if (rel >= next_gimbal) {
      emit_gimbal(sink);
      next_gimbal += gimbal_period;
      if (next_gimbal < rel) next_gimbal = rel + gimbal_period;
    }
    if (rel >= next_chassis) {
      emit_chassis(sink);
      next_chassis += chassis_period;
      if (next_chassis < rel) next_chassis = rel + chassis_period;
    }
    if (rel >= next_status) {
      emit_referee(sink);
      next_status += status_period;
      if (next_status < rel) next_status = rel + status_period;
    }
    if (rel >= next_slow) {
      emit_host_version(sink);
      emit_host_robot(sink);
      next_slow += slow_period;
      if (next_slow < rel) next_slow = rel + slow_period;
    }
    if (rel >= next_reset) {
      // 新协议没有 reset 位：MCU 重启（uptime_ms 回落）就是视觉侧的复位信号
      uptime_ms_ = 0;
      emit_host_version(sink);
      tools::logger()->info("[FAKE] Inject MCU restart (0x0500.uptime_ms -> 0)");
      next_reset += cfg_.reset_period_s;
    }

    if (std::chrono::duration<double>(SteadyClock::now() - last_log).count() >= 2.0) {
      maybe_log();
      last_log = SteadyClock::now();
    }

    const double next = std::min({next_gimbal, next_chassis, next_status, next_slow});
    const double wait = next - rel_now();
    if (wait > 0)
      std::this_thread::sleep_for(capped(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(wait))));
  }

  maybe_log();
  tools::logger()->info("[FAKE] VirtualMcu stopped.");
}

void VirtualMcu::emit_host_version(const Sink & sink)
{
  host_comm::host_version_t pkt{};
  pkt.host_version = host_comm::HOST_PROTOCOL_VERSION;
  pkt.referee_season = 2027;
  pkt.robot_type = 7;       // 哨兵
  pkt.chassis_type = 2;     // omni
  pkt.gimbal_type = 2;      // gimbal_DoubleYaw
  pkt.uptime_ms = uptime_ms_;

  uint8_t frame[referee_comm::MAX_PACKET];
  const size_t n = referee_comm::Receiver::Encode(
    frame, sizeof(frame), seq_++, host_comm::HOST_VERSION, &pkt, sizeof(pkt));
  if (n > 0) sink(frame, n);
}

void VirtualMcu::emit_host_robot(const Sink & sink)
{
  host_comm::host_robot_status_t pkt{};
  pkt.enable = 0x0F;              // 底盘/云台/发射/超电全开
  pkt.controller_online = 1;
  pkt.robot_mode = 3;             // 全自动哨兵
  pkt.fric_mode = 2;              // 就绪
  pkt.shoot_mode = 0;
  pkt.module_online = 0x3F;

  uint8_t frame[referee_comm::MAX_PACKET];
  const size_t n = referee_comm::Receiver::Encode(
    frame, sizeof(frame), seq_++, host_comm::ROBOT_STATUS, &pkt, sizeof(pkt));
  if (n > 0) sink(frame, n);
}

void VirtualMcu::emit_chassis(const Sink & sink)
{
  host_comm::host_chassis_status_t pkt{};
  pkt.chassis_enabled = 1;
  pkt.speed_x = static_cast<float>(1.0 * std::sin(2.0 * M_PI * 0.2 * t_now_));
  pkt.speed_y = static_cast<float>(0.3 * std::cos(2.0 * M_PI * 0.2 * t_now_));
  pkt.speed_turn = static_cast<float>(0.5 * std::sin(2.0 * M_PI * 0.05 * t_now_));
  pkt.feedback_angle = static_cast<float>(M_PI * std::sin(2.0 * M_PI * 0.02 * t_now_));

  uint8_t frame[referee_comm::MAX_PACKET];
  const size_t n = referee_comm::Receiver::Encode(
    frame, sizeof(frame), seq_++, host_comm::CHASSIS_STATUS, &pkt, sizeof(pkt));
  if (n > 0) sink(frame, n);
}

void VirtualMcu::emit_gimbal(const Sink & sink)
{
  const long long idx = imu_index_++;

  // 双yaw：大yaw 慢速摆动，小yaw 相对大yaw 有自己的小幅快摆。
  // 0x0503 只发「IMU 世界角 + 小yaw 关节角」，两轴由上位机自己复合，
  // 因此这里按关节角给，便于验证上位机的 lower = yaw - joint 是否正确。
  double lower = M_PI * std::sin(2.0 * M_PI * 0.1 * t_now_);
  // 加一个相位偏移，让 joint 在启动时就不为 0，避免"joint 恒 0"掩盖接线错误
  const double joint = 0.35 * std::sin(2.0 * M_PI * 0.45 * t_now_ + 0.6);
  double pitch = 0.15 * std::sin(2.0 * M_PI * 0.5 * t_now_);
  double roll = 0.05 * std::sin(2.0 * M_PI * 0.3 * t_now_);

  bool injected = false;

  if (fault_.imu_noise_rad > 0) {
    lower += injector_.gauss() * fault_.imu_noise_rad;
    pitch += injector_.gauss() * fault_.imu_noise_rad;
  }

  if (fault_.imu_spike_every > 0 && idx > 0 && idx % fault_.imu_spike_every == 0) {
    lower += fault_.imu_spike_rad;
    injected = true;
    log_fault("spike", "imu jump injected", idx);
  }

  // 噪声/跳变只加在大yaw 上，关节角保持干净：上位机应当能精确还原出 lower/upper
  double yaw = lower + joint;

  // INS 实际给 (-π, π]，故意按这个区间发，顺带验证上位机的 2π 归一化
  auto wrap_pi = [](double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
  };
  yaw = wrap_pi(yaw);

  if (injector_.chance(fault_.imu_drop_rate)) {
    dropped_++;
    log_fault("drop", "gimbal packet dropped", dropped_);
    return;
  }

  if (fault_.imu_delay_ms > 0) {
    std::this_thread::sleep_for(
      std::chrono::duration<double, std::milli>(fault_.imu_delay_ms));
    delayed_++;
    if (idx % 500 == 0) log_fault("delay", "gimbal packet delayed", delayed_);
  }

  host_comm::host_gimbal_status_t pkt{};
  pkt.gimbal_enable = 1;
  pkt.autoaim_enabled = 1;
  pkt.pitch_angle = static_cast<float>(pitch);
  pkt.yaw_angle = static_cast<float>(yaw);
  pkt.roll_angle = static_cast<float>(roll);
  pkt.upper_yaw_angle = static_cast<float>(joint);
  pkt.fold_yaw_angle = 0.0f;

  uint8_t frame[referee_comm::MAX_PACKET];
  size_t n = referee_comm::Receiver::Encode(
    frame, sizeof(frame), seq_++, host_comm::GIMBAL_STATUS, &pkt, sizeof(pkt));
  if (n == 0) return;

  if (injector_.chance(fault_.imu_corrupt_crc_rate)) {
    frame[n - 1] ^= 0xFF;  // 破坏 CRC16 高字节
    corrupt_++;
    injected = true;
    log_fault("crc", "gimbal crc corrupted", corrupt_);
  }

  sink(frame, n);
  emitted_++;
  if (injected) injected_++;
}

void VirtualMcu::emit_referee(const Sink & sink)
{
  uint8_t frame[referee_comm::MAX_PACKET];

  // 0x0001 比赛状态：game_type=1, game_progress=4（比赛中）
  referee_comm::v2027::game_status_t game{};
  game.game_type = 1;
  game.game_progress = 4;
  game.stage_remain_time = 300;
  game.SyncTimeStamp = SessionClock::now_rel_ns();
  size_t n = referee_comm::Receiver::Encode(
    frame, sizeof(frame), seq_++, referee_comm::v2027::GAME_STATUS, &game, sizeof(game));
  if (n > 0) sink(frame, n);

  // 0x0201 机器人状态：哨兵 400 血，17mm 热量相关字段给个常数
  referee_comm::v2027::robot_status_t robot{};
  robot.robot_id = robot_id_;
  robot.robot_level = 3;
  robot.current_HP = 400;
  robot.maximum_HP = 400;
  robot.shooter_barrel_cooling_value = 40;
  robot.shooter_barrel_heat_limit = 120;
  robot.chassis_power_limit = 40;
  robot.bullet_speed_limit = 30.0f;
  robot.power_management_gimbal_output = 1;
  robot.power_management_chassis_output = 1;
  robot.power_management_shooter_output = 1;
  n = referee_comm::Receiver::Encode(
    frame, sizeof(frame), seq_++, referee_comm::v2027::ROBOT_STATUS, &robot, sizeof(robot));
  if (n > 0) sink(frame, n);

  // 0x0202 热量/缓冲能量：heat 可配，用来验证上位机的热量保护
  referee_comm::v2027::power_heat_data_t heat{};
  heat.buffer_energy = 60;
  heat.shooter_17mm_barrel_heat = cfg_.heat;
  heat.shooter_42mm_barrel_heat = 0;
  n = referee_comm::Receiver::Encode(
    frame, sizeof(frame), seq_++, referee_comm::v2027::POWER_HEAT_DATA, &heat, sizeof(heat));
  if (n > 0) sink(frame, n);
}

void VirtualMcu::maybe_log()
{
  tools::logger()->info(
    "[FAKE] VirtualMcu stats: emitted={} dropped={} delayed={} injected={} corrupt_crc={}",
    emitted_, dropped_, delayed_, injected_, corrupt_);
}

// ============================ SerialReplayer ============================
SerialReplayer::SerialReplayer(std::string file, double speed, bool loop)
: file_(std::move(file)), speed_(speed > 0 ? speed : 1.0), loop_(loop), reader_(file_)
{
}

SerialReplayer::~SerialReplayer() { stop(); }

void SerialReplayer::start(const Sink & sink)
{
  quit_ = false;
  thread_ = std::thread([this, sink] { run(sink); });
}

void SerialReplayer::stop()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
}

void SerialReplayer::run(const Sink & sink)
{
  std::string error;
  if (!reader_.load(error)) {
    tools::logger()->error("[REPLAY] Serial replay load failed: {}", error);
    return;
  }

  const auto & records = reader_.records();
  if (records.empty()) {
    tools::logger()->error("[REPLAY] Serial replay file has no records: {}", file_);
    return;
  }

  const uint64_t base = records.front().t_ns;
  const uint64_t span = records.back().t_ns - base;
  ready_ = true;

  tools::logger()->info(
    "[REPLAY] Serial replay started: file={} records={} rx_bytes={} span={:.2f}s speed={}x loop={}",
    file_, records.size(), reader_.rx_bytes(), span / 1e9, speed_, loop_);

  size_t sent = 0;
  do {
    const auto t0 = SteadyClock::now();
    for (const auto & rec : records) {
      if (quit_) return;
      if (rec.dir != 0) continue;  // 只回放下位机→视觉方向

      const uint64_t dt = static_cast<uint64_t>((rec.t_ns - base) / speed_);
      const auto target = t0 + std::chrono::nanoseconds(dt);
      while (!quit_) {
        const auto now = SteadyClock::now();
        if (now >= target) break;
        const auto remain = target - now;
        std::this_thread::sleep_for(capped(remain));
      }
      if (quit_) return;
      sink(rec.data.data(), rec.data.size());
      sent++;
    }
  } while (loop_ && !quit_);

  tools::logger()->info("[REPLAY] Serial replay finished: {} packets sent.", sent);
}

// ============================ 传输层工厂 ============================
IByteTransport::Ptr make_transport(const SimConfig & config)
{
  const auto & s = config.serial;
  switch (s.mode) {
    case SourceMode::Real:
      tools::logger()->info("[SOURCE] Serial transport = REAL ({})", s.port);
      return std::make_unique<RealSerialTransport>(s.port, s.baud, 50);

    case SourceMode::Fake:
      if (s.external) {
        tools::logger()->info(
          "[SOURCE] Serial transport = FAKE/EXTERNAL pty ({}) — 由外部(Python)虚拟下位机供数", s.port);
        return std::make_unique<RealSerialTransport>(s.port, s.baud, 50);
      }
      tools::logger()->info(
        "[SOURCE] Serial transport = FAKE/INTERNAL VirtualMcu (gimbal={}Hz)", s.imu_hz);
      return std::make_unique<VirtualSerialTransport>(
        std::make_unique<VirtualMcu>(s, config.fault, config.seed, s.robot_id));

    case SourceMode::Replay:
      tools::logger()->info("[SOURCE] Serial transport = REPLAY ({})", s.replay_file);
      return std::make_unique<VirtualSerialTransport>(
        std::make_unique<SerialReplayer>(s.replay_file, 1.0, false));
  }
  return nullptr;
}

}  // namespace io::sim
