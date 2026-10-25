#include "io/sim/virtual_mcu.hpp"

#include <algorithm>
#include <cmath>

#include "io/serial/gimbal/packet.hpp"
#include "io/serial/gimbal/protocol_crc.hpp"
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
    "[FAKE] VirtualMcu started: imu={}Hz chassis={}Hz status={}Hz reset={}s robot_id={} faults={}",
    cfg_.imu_hz, cfg_.chassis_hz, cfg_.status_hz, cfg_.reset_period_s, robot_id_,
    fault_.enabled() ? "on" : "off");

  const double imu_period = 1.0 / cfg_.imu_hz;
  const double chassis_period = 1.0 / cfg_.chassis_hz;
  const double status_period = 1.0 / cfg_.status_hz;
  const double bass_period = 0.1;

  const auto t0 = SteadyClock::now();
  double next_imu = 0.0, next_chassis = 0.0, next_status = 0.0, next_bass = 0.0;
  double next_reset = cfg_.reset_period_s > 0 ? cfg_.reset_period_s : 1e18;
  auto last_log = t0;

  auto rel_now = [&t0] { return std::chrono::duration<double>(SteadyClock::now() - t0).count(); };

  while (!quit_) {
    const double rel = rel_now();
    t_now_ = rel;

    if (rel >= next_imu) {
      emit_imu(sink);
      next_imu += imu_period;
      if (next_imu < rel) next_imu = rel + imu_period;
    }
    if (rel >= next_chassis) {
      emit_chassis(sink);
      next_chassis += chassis_period;
      if (next_chassis < rel) next_chassis = rel + chassis_period;
    }
    if (rel >= next_bass) {
      emit_bass(sink, false);
      next_bass += bass_period;
      if (next_bass < rel) next_bass = rel + bass_period;
    }
    if (rel >= next_status) {
      emit_status(sink);
      next_status += status_period;
      if (next_status < rel) next_status = rel + status_period;
    }
    if (rel >= next_reset) {
      emit_bass(sink, true);
      tools::logger()->info("[FAKE] Inject reset signal (0x505.reset=1)");
      next_reset += cfg_.reset_period_s;
    }

    if (std::chrono::duration<double>(SteadyClock::now() - last_log).count() >= 2.0) {
      maybe_log();
      last_log = SteadyClock::now();
    }

    const double next = std::min({next_imu, next_chassis, next_bass, next_status});
    const double wait = next - rel_now();
    if (wait > 0)
      std::this_thread::sleep_for(capped(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(wait))));
  }

  maybe_log();
  tools::logger()->info("[FAKE] VirtualMcu stopped.");
}

void VirtualMcu::emit_imu(const Sink & sink)
{
  const long long idx = imu_index_++;

  // 双yaw：大yaw 慢速摆动，小yaw 在大yaw 基础上有自己的小幅快摆。
  // 对外按协议发两个绝对角（[0, 2π)），其差即 psi，便于验证视觉侧的解算。
  double lower = M_PI * std::sin(2.0 * M_PI * 0.1 * t_now_);
  // 加一个相位偏移，让 psi 在启动时就不为 0，避免"psi 恒 0"掩盖接线错误
  const double psi_rel = 0.35 * std::sin(2.0 * M_PI * 0.45 * t_now_ + 0.6);
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

  // 噪声/跳变只加在大yaw 上，小yaw 相对角保持干净：psi 应当能被视觉侧精确还原
  double upper = lower + psi_rel;

  // 归一化到 [0, 2π)，与下位机的上报约定一致
  auto wrap_2pi = [](double a) {
    while (a < 0.0) a += 2.0 * M_PI;
    while (a >= 2.0 * M_PI) a -= 2.0 * M_PI;
    return a;
  };
  lower = wrap_2pi(lower);
  upper = wrap_2pi(upper);

  if (injector_.chance(fault_.imu_drop_rate)) {
    dropped_++;
    log_fault("drop", "imu packet dropped", dropped_);
    return;
  }

  if (fault_.imu_delay_ms > 0) {
    std::this_thread::sleep_for(
      std::chrono::duration<double, std::milli>(fault_.imu_delay_ms));
    delayed_++;
    if (idx % 500 == 0) log_fault("delay", "imu packet delayed", delayed_);
  }

  IMUPacket pkt{};
  pkt.header.header = 0xA5;
  pkt.header.data_length = static_cast<uint16_t>(sizeof(IMUPacket) - sizeof(Header) - 2);
  pkt.header.seq = static_cast<uint8_t>(idx & 0xFF);
  pkt.header.cmd_id = 0x502;
  crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&pkt.header), sizeof(Header) - 2);

  pkt.pitch = static_cast<float>(pitch);
  pkt.roll = static_cast<float>(roll);
  pkt.upper_yaw = static_cast<float>(upper);
  pkt.lower_yaw = static_cast<float>(lower);
  pkt.roboid = robot_id_;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));

  if (injector_.chance(fault_.imu_corrupt_crc_rate)) {
    pkt.crc16 ^= 0xFFFF;
    corrupt_++;
    injected = true;
    log_fault("crc", "imu crc corrupted", corrupt_);
  }

  sink(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
  emitted_++;
  if (injected) injected_++;
}

void VirtualMcu::emit_chassis(const Sink & sink)
{
  classisPacket pkt{};
  pkt.header.header = 0xA5;
  pkt.header.data_length = static_cast<uint16_t>(sizeof(classisPacket) - sizeof(Header) - 2);
  pkt.header.seq = 0;
  pkt.header.cmd_id = 0x503;
  crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&pkt.header), sizeof(Header) - 2);

  pkt.vx = static_cast<float>(1.0 * std::sin(2.0 * M_PI * 0.2 * t_now_));
  pkt.vy = static_cast<float>(0.3 * std::cos(2.0 * M_PI * 0.2 * t_now_));
  pkt.vz = static_cast<float>(0.5 * std::sin(2.0 * M_PI * 0.05 * t_now_));
  pkt.enable = 1;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));

  sink(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
}

void VirtualMcu::emit_status(const Sink & sink)
{
  GameStatusPacket pkt{};
  pkt.header.header = 0xA5;
  pkt.header.data_length = static_cast<uint16_t>(sizeof(GameStatusPacket) - sizeof(Header) - 2);
  pkt.header.seq = 0;
  pkt.header.cmd_id = 0x0001;
  crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&pkt.header), sizeof(Header) - 2);

  // game_type=1, game_progress=4(比赛中)
  pkt.game_info = static_cast<uint8_t>((4 << 4) | 1);
  pkt.stage_remain_time = 300;
  pkt.sync_timestamp = SessionClock::now_rel_ns();
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));

  sink(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
}

void VirtualMcu::emit_bass(const Sink & sink, bool reset)
{
  bassPacket pkt{};
  pkt.header.header = 0xA5;
  pkt.header.data_length = static_cast<uint16_t>(sizeof(bassPacket) - sizeof(Header) - 2);
  pkt.header.seq = 0;
  pkt.header.cmd_id = 0x505;
  crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&pkt.header), sizeof(Header) - 2);

  pkt.roboid = robot_id_;
  pkt.reset = reset;
  pkt.position[0] = 0;
  pkt.position[1] = 0;
  pkt.HP = 400;
  pkt.robomode = 1;
  pkt.canshoot = 1;
  pkt.shootmode = 0;
  pkt.superpower = 0;
  pkt.online = 1;
  pkt.two = 0;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));

  sink(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
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
        "[SOURCE] Serial transport = FAKE/INTERNAL VirtualMcu (imu={}Hz)", s.imu_hz);
      return std::make_unique<VirtualSerialTransport>(
        std::make_unique<VirtualMcu>(s, config.fault, config.seed));

    case SourceMode::Replay:
      tools::logger()->info("[SOURCE] Serial transport = REPLAY ({})", s.replay_file);
      return std::make_unique<VirtualSerialTransport>(
        std::make_unique<SerialReplayer>(s.replay_file, 1.0, false));
  }
  return nullptr;
}

}  // namespace io::sim
