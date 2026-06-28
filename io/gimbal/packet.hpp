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

// cmd_id 0x502 — IMU data from gimbal
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

// cmd_id 0x503 — chassis velocity
struct classisPacket
{
  Header header;
  float vx;
  float vy;
  float vz;
  uint8_t enable;
  uint16_t crc16 = 0;
} __attribute__((packed));

// cmd_id 0x504 — button / trigger state
struct bottonPacket
{
  Header header;
  bool start;
  uint16_t crc16 = 0;
} __attribute__((packed));

// cmd_id 0x505 — robot base state (reset, HP, mode, etc.)
struct bassPacket
{
  Header header;
  uint8_t roboid;
  bool reset;
  uint8_t position[2];
  uint8_t HP;
  uint8_t robomode;
  bool canshoot;
  uint8_t shootmode;
  bool superpower;
  bool online;
  bool two;
  uint16_t crc16 = 0;
} __attribute__((packed));

// cmd_id 0x0001 / 0x0100 — game / match status
// game_info: low 4 bits = game_type, high 4 bits = game_progress
struct GameStatusPacket
{
  Header header;
  uint8_t game_info;
  uint16_t stage_remain_time;
  uint64_t sync_timestamp;
  uint16_t crc16 = 0;
} __attribute__((packed));

// cmd_id 0x0201 / 0x0102 — robot health / shooting / power status
// mains_power_state bit0/bit1/bit2: gimbal / chassis / shooter output
struct GameRobotStatusPacket
{
  Header header;
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_hp;
  uint16_t max_hp;
  uint16_t shooter_cooling_rate;
  uint16_t shooter_heat_limit;
  uint16_t chassis_power_limit;
  uint8_t mains_power_state;
  uint16_t crc16 = 0;
} __attribute__((packed));

// cmd_id 0x0402 — vision → MCU aim command
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

// cmd_id 0x0405 — vision → MCU navigation command
struct NavigationPacket
{
  Header header;
  float vx;
  float vy;
  float wz;
  uint16_t crc16;
} __attribute__((packed, aligned(1)));

// Parsed game status (extracted from GameStatusPacket)
struct GameStatus
{
  Header header;
  uint8_t game_type;         // 比赛类型
  uint8_t game_progress;     // 比赛阶段 (4 = 比赛中)
  uint16_t stage_remain_time; // 阶段剩余时间
  uint64_t sync_timestamp;   // 同步时间戳
};

// Parsed robot status (extracted from GameRobotStatusPacket)
struct RobotStatus
{
  Header header;
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_hp;
  uint16_t max_hp;
  uint16_t shooter_cooling_rate;
  uint16_t shooter_heat_limit;
  uint16_t chassis_power_limit;
  bool mains_power_gimbal;    // 云台供电
  bool mains_power_chassis;   // 底盘供电
  bool mains_power_shooter;   // 发射机构供电
};

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
