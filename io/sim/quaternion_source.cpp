#include "io/sim/quaternion_source.hpp"

#include <cmath>
#include <stdexcept>

#include "io/serial/gimbal/packet.hpp"
#include "io/serial/gimbal/protocol_crc.hpp"
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

bool SerialQuaternionSource::read_exact(uint8_t * buffer, size_t size, int timeout_ms)
{
  size_t got = 0;
  const auto deadline = SteadyClock::now() + std::chrono::milliseconds(timeout_ms);
  while (got < size) {
    if (quit_) return false;
    const auto now = SteadyClock::now();
    if (now >= deadline) return false;

    int remain = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
    if (remain <= 0) remain = 1;

    const size_t n = transport_->read(buffer + got, size - got, remain);
    if (n == 0) return false;
    if (recorder_) recorder_->record_rx(buffer + got, n);
    got += n;
  }
  return true;
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
  long long bad_crc8 = 0, bad_crc16 = 0;

  std::vector<uint8_t> flag(1);
  std::vector<uint8_t> headdata;
  std::vector<uint8_t> payload;
  std::vector<uint8_t> full;

  while (!quit_) {
    if (!read_exact(flag.data(), 1, 50)) continue;
    if (flag[0] != 0xA5) continue;

    headdata.resize(sizeof(Header) - 1);
    if (!read_exact(headdata.data(), headdata.size(), 50)) continue;
    headdata.insert(headdata.begin(), flag[0]);

    Header header = fromVector<Header>(headdata);
    if (!crc8::Verify_CRC8_Check_Sum(
          reinterpret_cast<const uint8_t *>(&header), sizeof(header) - 2)) {
      bad_crc8++;
      continue;
    }

    const size_t payload_len = static_cast<size_t>(header.data_length);
    if (payload_len == 0 || payload_len > 512) continue;  // 防御非法长度

    payload.resize(payload_len + 2);  // 载荷 + CRC16
    if (!read_exact(payload.data(), payload.size(), 50)) continue;

    full.clear();
    full.reserve(headdata.size() + payload.size());
    full.insert(full.end(), headdata.begin(), headdata.end());
    full.insert(full.end(), payload.begin(), payload.end());

    if (!crc16::Verify_CRC16_Check_Sum(full.data(), static_cast<uint32_t>(full.size()))) {
      bad_crc16++;
      continue;
    }
    packet_count_++;

    if (header.cmd_id == 0x502 && full.size() >= sizeof(IMUPacket)) {
      IMUPacket pkt = fromVector<IMUPacket>(full);
      // 与 SerialBoard::imu_at 同一约定：用大yaw(lower_yaw) 构造 R_gimbal2world。
      // 小yaw 的 psi 只有双yaw 平台的业务入口需要，回放器不关心，故不在这里保留。
      const Eigen::Quaterniond q =
        Eigen::AngleAxisd(pkt.lower_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pkt.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(pkt.roll, Eigen::Vector3d::UnitX());
      push_sample(q);
    }

    if (std::chrono::duration<double>(SteadyClock::now() - last_log).count() >= 2.0) {
      tools::logger()->info(
        "[{}] '{}' stats: packets={} imu={} bad_crc8={} bad_crc16={}", log_tag(mode_), name_,
        packet_count_, imu_count_, bad_crc8, bad_crc16);
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
