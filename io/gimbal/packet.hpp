#ifndef IO__GIMBAL__PACKET_HPP
#define IO__GIMBAL__PACKET_HPP

#include <algorithm>
#include <cstdint>
#include <vector>

namespace io
{
struct Header
{
  uint8_t header = 0xA5;
  uint16_t data_length = 15;
  uint8_t seq = 0;
  uint8_t crc8 = 0;
  uint16_t cmd_id;
} __attribute__((packed));

struct IMUPacket
{
  Header header;
  float pitch;
  float roll;
  float yaw;
  uint8_t roboid;
  uint8_t id;
  uint8_t timeseries;
  uint16_t crc16 = 0;
} __attribute__((packed));

struct SendPacket
{
  Header header;
  uint8_t id = 0;
  uint8_t robo_id = 0;
  float pitch;
  float yaw;
  uint8_t accuracy = 50;
  uint8_t shoot;
  uint16_t checksum = 0;
} __attribute__((packed));

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace io

#endif  // IO__GIMBAL__PACKET_HPP
