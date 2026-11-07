#include "io/can/minipc_codec.hpp"

// Keep checks active when the standalone project is built in Release mode.
#ifdef NDEBUG
#undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace protocol = io::minipc;

bool rejects(protocol::Command command)
{
  try {
    protocol::encode_command(command);
    return false;
  } catch (const std::invalid_argument &) {
    return true;
  }
}

int main(int argc, char ** argv)
{
  assert(argc == 1 || argc == 2);
  protocol::Command command{254, 65530, -1.234, -0.250, true, true};
  const protocol::Payload example{0xFE, 0x13, 0x2E, 0xFB, 0x06, 0xFF, 0xFA, 0xFF};
  assert(protocol::encode_command(command) == example);

  command.tracking = false;
  command.yaw_rad = std::numeric_limits<double>::quiet_NaN();
  command.pitch_raw_rad = std::numeric_limits<double>::infinity();
  const protocol::Payload idle{0xFE, 0x10, 0, 0, 0, 0, 0xFA, 0xFF};
  assert(protocol::encode_command(command) == idle);

  command = {255, 65535, std::numbers::pi, 0.4, true, false};
  const auto upper = protocol::encode_command(command);
  assert(upper[2] == 0x46 && upper[3] == 0x0C);
  assert(upper[4] == 0x90 && upper[5] == 0x01);
  command.yaw_rad = -std::numbers::pi;
  assert(protocol::encode_command(command)[2] == 0xBA);
  for (const double invalid : {3.142, -3.142, std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN()}) {
    command.yaw_rad = invalid;
    assert(rejects(command));
  }
  command.yaw_rad = 0;
  for (const double invalid : {-0.250001, 0.400001, std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN()}) {
    command.pitch_raw_rad = invalid;
    assert(rejects(command));
  }

  protocol::Payload payload{42, 0x17, 0x2E, 0xFB, 0x06, 0xFF, 0xFA, 0xFF};
  auto decoded = protocol::decode_telemetry(protocol::kPoseId, payload);
  const auto pose = std::get<protocol::Pose>(decoded.value());
  assert(pose.sequence == 42 && pose.reference_ms == 65530);
  assert(pose.yaw_rad == -1.234 && pose.pitch_raw_rad == -0.25);
  assert(pose.pose_valid && pose.autoaim_active && pose.unfolded);
  for (unsigned version = 0; version < 16; ++version) {
    payload[1] = static_cast<std::uint8_t>(version << 4);
    assert(protocol::decode_telemetry(protocol::kPoseId, payload).has_value() == (version == 1));
  }
  payload[1] = 0x10;
  const auto invalid_pose = std::get<protocol::Pose>(
    protocol::decode_telemetry(protocol::kPoseId, payload).value());
  assert(!invalid_pose.pose_valid);

  payload = {42, 2, 0x18, 0xFC, 0xD0, 0x07, 0x0C, 0xFE};
  const auto chassis = std::get<protocol::Chassis>(
    protocol::decode_telemetry(protocol::kChassisId, payload).value());
  assert(chassis.sequence == 42 && chassis.mode == 2);
  assert(chassis.yaw_rad == -1.0 && chassis.yaw_rate_rad_s == 2.0);
  assert(chassis.relative_yaw_rad == -0.5);

  for (unsigned bit = 0; bit < 8; ++bit) {
    payload = {255, static_cast<std::uint8_t>(1U << bit), 103, 7, 0xF0, 0x0A, 0xFF, 0xFF};
    const auto status = std::get<protocol::RobotStatus>(
      protocol::decode_telemetry(protocol::kRobotStatusId, payload).value());
    assert(status.sequence == 255 && status.robot_id == 103 && status.fold_state == 7);
    assert(status.bullet_speed_m_s == 28.0 && status.shot_event_count == 65535);
    const bool flags[]{status.enabled, status.autoaim_requested, status.friction_ready,
                       status.referee_allows_fire, status.advice_and_aligned, status.feeder_busy,
                       status.robot_status_valid, status.bullet_speed_valid};
    for (unsigned index = 0; index < 8; ++index) assert(flags[index] == (index == bit));
  }
  for (unsigned bit = 0; bit < 3; ++bit) {
    payload = {0, static_cast<std::uint8_t>(1U << bit), 0x85, 0xFF, 0xD0, 0x07, 0x64, 0};
    const auto heat = std::get<protocol::Heat>(
      protocol::decode_telemetry(protocol::kHeatId, payload).value());
    assert(heat.sequence == 0 && heat.heat == -12.3);
    assert(heat.heat_limit == 200.0 && heat.cooling_rate == 10.0);
    assert(heat.heat_valid == (bit == 0));
    assert(heat.feeder_online == (bit == 1));
    assert(heat.friction_on == (bit == 2));
  }
  std::array<std::uint8_t, 9> buffer{};
  for (std::size_t length = 0; length <= buffer.size(); ++length) {
    if (length == 8) continue;
    assert(!protocol::decode_telemetry(protocol::kHeatId, {buffer.data(), length}));
  }
  for (const std::uint32_t identifier : {0x510U, 0x515U, 0x80000514U, 0x40000514U, 0x20000514U}) {
    assert(!protocol::decode_telemetry(identifier, payload));
  }

  if (argc == 1) {
    std::cout << "PASS: protocol example, angle limits, signed fields, all status bits and invalid frames\n";
    return 0;
  }

  // Optional cross-check against the electrical team's Python encoder vectors.
  std::ifstream vectors(argv[1]);
  assert(vectors.is_open());
  unsigned sequence = 0, reference = 0;
  bool tracking = false, advice = false;
  double yaw = 0, pitch = 0;
  std::string expected;
  std::size_t count = 0;
  while (vectors >> sequence >> reference >> yaw >> pitch >> tracking >> advice >> expected) {
    const auto actual = protocol::encode_command(
      {static_cast<std::uint8_t>(sequence), static_cast<std::uint16_t>(reference), yaw, pitch,
       tracking, advice});
    const char digits[] = "0123456789abcdef";
    std::string encoded;
    for (const auto byte : actual) {
      encoded.push_back(digits[byte >> 4]);
      encoded.push_back(digits[byte & 0x0F]);
    }
    assert(encoded == expected);
    ++count;
  }
  assert(count > 1000);
  std::cout << "PASS: protocol example, signed fields, all status bits, invalid frames, "
            << count << " Python reference vectors\n";
}
