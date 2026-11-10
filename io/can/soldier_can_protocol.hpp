#ifndef IO__CAN__SOLDIER_CAN_PROTOCOL_HPP
#define IO__CAN__SOLDIER_CAN_PROTOCOL_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>

#include "soldier_can_config.hpp"

namespace io::soldier_can
{

inline constexpr std::uint16_t kCommandId = config::kCommandId;
inline constexpr std::uint16_t kPoseId = config::kFirstTelemetryId;
inline constexpr std::uint16_t kChassisId = config::kFirstTelemetryId + 1;
inline constexpr std::uint16_t kRobotStatusId = config::kFirstTelemetryId + 2;
inline constexpr std::uint16_t kHeatId = config::kLastTelemetryId;
using Payload = std::array<std::uint8_t, 8>;

struct Command
{
  std::uint8_t sequence = 0;
  std::uint16_t reference_ms = 0;  // Echo the 0x511 sample used for this result.
  double yaw_rad = 0.0;
  double pitch_raw_rad = 0.0;
  bool tracking = false;
  bool fire_advice = false;
};

struct Pose
{
  std::uint8_t sequence;
  double yaw_rad;
  double pitch_raw_rad;
  std::uint16_t reference_ms;
  bool pose_valid;
  bool autoaim_active;
  bool unfolded;
};

struct Chassis
{
  std::uint8_t sequence;
  std::uint8_t mode;
  double yaw_rad;
  double yaw_rate_rad_s;
  double relative_yaw_rad;
};

struct RobotStatus
{
  std::uint8_t sequence;
  std::uint8_t robot_id;
  std::uint8_t fold_state;
  double bullet_speed_m_s;
  std::uint16_t shot_event_count;
  bool enabled;
  bool autoaim_requested;
  bool friction_ready;
  bool referee_allows_fire;
  bool advice_and_aligned;
  bool feeder_busy;
  bool robot_status_valid;
  bool bullet_speed_valid;
};

struct Heat
{
  std::uint8_t sequence;
  double heat;
  double heat_limit;
  double cooling_rate;
  bool heat_valid;
  bool feeder_online;
  bool friction_on;
};

using Telemetry = std::variant<Pose, Chassis, RobotStatus, Heat>;

// Throws std::invalid_argument for out-of-range tracking angles. Idle commands
// always encode zero angles and clear fire_advice, as required by the protocol.
Payload encode_command(const Command & command);

// Pass the 11-bit identifier and payload of a classic CAN standard data frame.
// Unsupported IDs, bad lengths and unsupported pose versions return nullopt.
std::optional<Telemetry> decode_telemetry(
  std::uint32_t can_id, std::span<const std::uint8_t> payload);

}  // namespace io::soldier_can

#endif  // IO__CAN__SOLDIER_CAN_PROTOCOL_HPP
