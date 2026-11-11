#ifndef IO__CAN__SOLDIER_CAN_CONFIG_HPP
#define IO__CAN__SOLDIER_CAN_CONFIG_HPP

#include <chrono>
#include <cstdint>

namespace io::soldier_can::config
{
inline constexpr std::uint16_t kCommandId = 0x510;
inline constexpr std::uint16_t kFirstTelemetryId = 0x511;
inline constexpr std::uint16_t kLastTelemetryId = 0x514;
inline constexpr std::uint8_t kProtocolVersion = 1;
inline constexpr std::uint8_t kPayloadSize = 8;
inline constexpr std::uint32_t kBitrate = 1'000'000;
inline constexpr std::uint32_t kMaxStandardId = 0x7FF;
inline constexpr auto kTimeout = std::chrono::milliseconds(100);
inline constexpr auto kMinimumSendPeriod = std::chrono::milliseconds(10);  // Maximum 100 Hz.
inline constexpr double kMinPitchRawRad = -0.25;
inline constexpr double kMaxPitchRawRad = 0.4;
}  // namespace io::soldier_can::config

#endif  // IO__CAN__SOLDIER_CAN_CONFIG_HPP
