#include "io/sim/quaternion_source.hpp"

#include <cmath>
#include <stdexcept>

#include "io/serial/protocol/host_protocol.hpp"
#include "io/serial/protocol/referee.hpp"
#include "tools/logger.hpp"

namespace io::sim
{

SerialQuaternionSource::SerialQuaternionSource(
  IByteTransport::Ptr transport, SourceMode mode, std::string name, bool external)
: transport_(std::move(transport)), mode_(mode), name_(std::move(name)), external_(external)
{
}

SerialQuaternionSource::~SerialQuaternionSource() { close(); }

void SerialQuaternionSource::open()
{
  if (!transport_) throw std::runtime_error("SerialQuaternionSource: null transport");
  transport_->open();
  quit_ = false;
  thread_ = std::thread([this] { parse_loop(); });
  tools::logger()->info(
    "[{}] Quaternion source '{}' opened ({}{})", log_tag(mode_), name_, transport_->describe(),
    mode_ == SourceMode::Fake && external_ ? ", external pty" : "");
}

void SerialQuaternionSource::close()
{
  if (closed_) return;
  closed_ = true;
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  if (transport_) transport_->close();
  tools::logger()->info(
    "[{}] Quaternion source '{}' closed: packets={} imu={}", log_tag(mode_), name_, packet_count_,
    imu_count_);
}

bool SerialQuaternionSource::read(QuaternionSample & out, int timeout_ms)
{
  return queue_.try_pop_for(out, std::chrono::milliseconds(timeout_ms));
}

void SerialQuaternionSource::push_sample(const Eigen::Quaterniond & q)
{
  QuaternionSample s;
  s.timestamp = SteadyClock::now();
  s.q = q.normalized();
  s.valid = true;
  s.seq = seq_.fetch_add(1, std::memory_order_relaxed);
  queue_.push(s);
  imu_count_++;
}

void SerialQuaternionSource::parse_loop()
{
  auto last_log = SteadyClock::now();

  // 复用与 SerialBoard 同一套解包：0xA5 分帧 + CRC 校验 + host 0x0503 解码
  host_comm::Protocol host;
  referee_comm::Receiver receiver(host);

  std::vector<uint8_t> buffer(1024);

  while (!quit_) {
    const size_t n = transport_->read(buffer.data(), buffer.size(), 50);
    if (n == 0) continue;
    if (recorder_) recorder_->record_rx(buffer.data(), n);

    const uint64_t before = host.gimbal_count();
    const uint64_t frames_before = receiver.stats().frames;
    receiver.Feed(buffer.data(), n);
    packet_count_ += static_cast<long long>(receiver.stats().frames - frames_before);

    if (host.gimbal_count() != before) {
      // 与 SerialBoard::imu_at 同一约定：用大yaw(lower_yaw) 构造 R_gimbal2world。
      // 双yaw 平台按 0x0503 的 yaw_angle(IMU/相机世界角) - upper_yaw_angle(小yaw 关节角)
      // 还原大yaw；单yaw 构型 upper_yaw_angle 为 0，两者自动相等。
      const double lower = host.gimbal.yaw_angle - host.gimbal.upper_yaw_angle;
      const Eigen::Quaterniond q =
        Eigen::AngleAxisd(lower, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(host.gimbal.pitch_angle, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(host.gimbal.roll_angle, Eigen::Vector3d::UnitX());
      push_sample(q);
    }

    if (std::chrono::duration<double>(SteadyClock::now() - last_log).count() >= 2.0) {
      tools::logger()->info(
        "[{}] '{}' stats: packets={} gimbal={} bad_crc8={} bad_crc16={}", log_tag(mode_), name_,
        packet_count_, imu_count_, receiver.stats().crc8_errors, receiver.stats().crc16_errors);
      last_log = SteadyClock::now();
    }
  }
}

// ============================ 具体数据源 ============================
RealSerialSource::RealSerialSource(const SimConfig & config)
: SerialQuaternionSource(
    make_transport(with_serial_mode(config, SourceMode::Real)), SourceMode::Real, "RealSerial", true)
{
}

FakeSerialSource::FakeSerialSource(const SimConfig & config)
: SerialQuaternionSource(
    make_transport(with_serial_mode(config, SourceMode::Fake)), SourceMode::Fake, "FakeSerial",
    config.serial.external)
{
}

ReplaySerialSource::ReplaySerialSource(const SimConfig & config)
: SerialQuaternionSource(
    make_transport(with_serial_mode(config, SourceMode::Replay)), SourceMode::Replay,
    "ReplaySerial", false)
{
}

QuaternionSource::Ptr make_quaternion_source(const SimConfig & config)
{
  switch (config.serial.mode) {
    case SourceMode::Real:
      return std::make_unique<RealSerialSource>(config);
    case SourceMode::Fake:
      return std::make_unique<FakeSerialSource>(config);
    case SourceMode::Replay:
      return std::make_unique<ReplaySerialSource>(config);
  }
  return std::make_unique<RealSerialSource>(config);
}

}  // namespace io::sim
