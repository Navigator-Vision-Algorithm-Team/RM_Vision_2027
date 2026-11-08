#include "soldier_can_protocol.hpp"

#include <bit>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace io::soldier_can
{
namespace
{

std::uint16_t read_u16(std::span<const std::uint8_t> data, std::size_t offset)
{
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(data[offset]) |
    (static_cast<std::uint16_t>(data[offset + 1]) << 8));
}

std::int16_t read_i16(std::span<const std::uint8_t> data, std::size_t offset)
{
  return std::bit_cast<std::int16_t>(read_u16(data, offset));
}

void write_u16(Payload & data, std::size_t offset, std::uint16_t value)
{
  data[offset] = static_cast<std::uint8_t>(value & 0xFF);
  data[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

std::int16_t milliradians(double radians)
{
  // Match Python round(): nearest integer, with ties rounded to even.
  const double scaled = radians * 1000.0;
  const double lower = std::floor(scaled);
  const double fraction = scaled - lower;
  int rounded = static_cast<int>(lower);
  if (fraction > 0.5 || (fraction == 0.5 && rounded % 2 != 0)) ++rounded;
  return static_cast<std::int16_t>(rounded);
}

}  // namespace

Payload encode_command(const Command & command)
{
  Payload data{};
  data[0] = command.sequence;
  data[1] = 0x10;
  write_u16(data, 6, command.reference_ms);
  if (!command.tracking) return data;

  if (!std::isfinite(command.yaw_rad) || command.yaw_rad < -std::numbers::pi ||
      command.yaw_rad > std::numbers::pi) {
    throw std::invalid_argument("yaw_rad must be finite and within [-pi, pi]");
  }
  if (!std::isfinite(command.pitch_raw_rad) || command.pitch_raw_rad < -0.25 ||
      command.pitch_raw_rad > 0.4) {
    throw std::invalid_argument("pitch_raw_rad must be finite and within [-0.25, 0.4]");
  }

  data[1] |= 0x01;
  if (command.fire_advice) data[1] |= 0x02;
  write_u16(data, 2, static_cast<std::uint16_t>(milliradians(command.yaw_rad)));
  write_u16(data, 4, static_cast<std::uint16_t>(milliradians(command.pitch_raw_rad)));
  return data;
}

std::optional<Telemetry> decode_telemetry(
  std::uint32_t can_id, std::span<const std::uint8_t> payload)
{
  // SocketCAN places EFF/RTR/ERR flags above the 11-bit standard identifier.
  if (can_id > 0x7FF || payload.size() != 8) return std::nullopt;
  const auto sequence = payload[0];
  const auto flags = payload[1];

  switch (can_id) {
    case kPoseId:
      if ((flags >> 4) != config::kProtocolVersion) return std::nullopt;
      return Pose{
        sequence, read_i16(payload, 2) / 1000.0, read_i16(payload, 4) / 1000.0,
        read_u16(payload, 6), (flags & 0x01) != 0, (flags & 0x02) != 0, (flags & 0x04) != 0};
    case kChassisId:
      return Chassis{
        sequence, flags, read_i16(payload, 2) / 1000.0, read_i16(payload, 4) / 1000.0,
        read_i16(payload, 6) / 1000.0};
    case kRobotStatusId:
      return RobotStatus{
        sequence, payload[2], payload[3], read_u16(payload, 4) / 100.0,
        read_u16(payload, 6), (flags & 0x01) != 0, (flags & 0x02) != 0,
        (flags & 0x04) != 0, (flags & 0x08) != 0, (flags & 0x10) != 0,
        (flags & 0x20) != 0, (flags & 0x40) != 0, (flags & 0x80) != 0};
    case kHeatId:
      return Heat{
        sequence, read_i16(payload, 2) / 10.0, read_i16(payload, 4) / 10.0,
        read_i16(payload, 6) / 10.0, (flags & 0x01) != 0, (flags & 0x02) != 0,
        (flags & 0x04) != 0};
    default:
      return std::nullopt;
  }
}

}  // namespace io::soldier_can
