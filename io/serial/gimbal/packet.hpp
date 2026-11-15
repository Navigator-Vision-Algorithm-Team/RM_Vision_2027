#ifndef IO__GIMBAL__PACKET_HPP
#define IO__GIMBAL__PACKET_HPP

#include <cstdint>

// ============================================================================
// 上位机 → 下位机 的下行结构体（0x0402 自瞄 / 0x0405 导航）。
//
// 接收方向（下位机 → 上位机）按下位机固件的协议分层解析，不再用
// 「一个 cmd 一个整帧结构体」的写法：
//   · 线格式与分帧    io/serial/protocol/referee.hpp
//   · host 上行 0x05xx io/serial/protocol/host_protocol.hpp
//   · 裁判转发 0x0xxx  io/serial/protocol/seasons/2027/protocol.hpp
//   · 号段分派        io/serial/protocol/link.hpp
//
// 下行只发这两帧，字段以本文件为准（2026-11 精简后的定义）：
//   0x0402  data 9 字节  = pitch(f32) yaw(f32) shoot(u8)
//   0x0405  data 16 字节 = vx(f32) vy(f32) wz(f32) target_angle(f32)
// ★ 固件按 sizeof 做严格长度校验，多一字节少一字节都会整帧丢弃，改动必须两边同步。
// ============================================================================

namespace io
{

// 裁判系统帧头（5 字节）
struct Header
{
  uint8_t header = 0xA5;
  uint16_t data_length = 15;
  uint8_t seq = 0;
  uint8_t crc8 = 0;
  uint16_t cmd_id;
} __attribute__((packed));

// cmd_id 0x0402 — vision → MCU aim command（data 段 9 字节）
struct SendPacket
{
  Header header;
  float pitch = 0.0f;  // 世界系绝对 pitch（rad）
  float yaw = 0.0f;    // 世界系绝对 yaw（rad）
  uint8_t shoot = 0;   // 1 = 允许发弹
  uint16_t crc16 = 0;
} __attribute__((packed));

// cmd_id 0x0405 — vision → MCU navigation command（data 段 16 字节）
struct NavigationPacket
{
  Header header;
  float vx;            // 体坐标系前进速度，m/s
  float vy;            // 体坐标系左移速度，m/s；★下发前取反（SerialBoard::send），符号约定已确认保留
  float wz;            // 转向角速度，rad/s
  float target_angle;  // 目标航向角，rad；★已确认暂时固定发 0，等导航侧提供航向后透传
  uint16_t crc16;
} __attribute__((packed, aligned(1)));

// 下行帧长必须与固件结构体完全一致（固件按 sizeof 严格校验，不一致会整帧丢弃）
static_assert(
  sizeof(SendPacket) - sizeof(Header) - 2 == 9,
  "0x0402 data 段必须是 9 字节（pitch/yaw/shoot），改动需同步下位机固件");
static_assert(
  sizeof(NavigationPacket) - sizeof(Header) - 2 == 16,
  "0x0405 data 段必须是 16 字节（vx/vy/wz/target_angle），改动需同步下位机固件");

// 解析后的比赛状态（裁判 0x0001，由 SerialBoard 从 v2027::game_status_t 搬运而来）
struct GameStatus
{
  uint8_t game_type;          // 比赛类型
  uint8_t game_progress;      // 比赛阶段 (4 = 比赛中)
  uint16_t stage_remain_time; // 阶段剩余时间
  uint64_t sync_timestamp;    // 同步时间戳
};

// 解析后的机器人状态（裁判 0x0201，由 SerialBoard 从 v2027::robot_status_t 搬运而来）
struct RobotStatus
{
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_hp;
  uint16_t max_hp;
  uint16_t shooter_cooling_rate;
  uint16_t shooter_heat_limit;
  uint16_t chassis_power_limit;
  float bullet_speed_limit;  // 2027 新增：射击初速度上限 m/s
  bool mains_power_gimbal;   // 云台供电
  bool mains_power_chassis;  // 底盘供电
  bool mains_power_shooter;  // 发射机构供电
};

}  // namespace io

#endif  // IO__GIMBAL__PACKET_HPP
