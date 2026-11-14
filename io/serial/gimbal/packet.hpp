#ifndef IO__GIMBAL__PACKET_HPP
#define IO__GIMBAL__PACKET_HPP

#include <algorithm>
#include <cstdint>
#include <vector>

namespace io
{
/*
 * 所有帧共用的 7 字节帧头。
 *
 * data_length 只统计数据段字节数，不含本帧头 7 字节与末尾 2 字节 CRC16，
 * 所以打包时的算法统一是 sizeof(Packet) - sizeof(Header) - 2。
 * CRC8 覆盖帧头前 5 字节（0xA5 + data_length + seq + crc8 占位），
 * CRC16 覆盖整帧（crc16 字段按 0 参与计算），两者都是 RoboMaster 标准算法，
 * 实现见 protocol_crc.hpp，不可替换为标准位运算版本。
 *
 * 字段依次为：固定帧头 0xA5、数据段长度、帧序号（当前恒 0）、帧头校验、命令字。
 */
struct Header
{
  uint8_t header = 0xA5;
  uint16_t data_length = 15;
  uint8_t seq = 0;
  uint8_t crc8 = 0;
  uint16_t cmd_id;
} __attribute__((packed));

/*
 * cmd_id 0x502 — 下位机上报的 IMU 姿态。
 *
 * 双yaw 结构（哨兵）：底盘 → 大yaw(lower) → 小yaw(upper，主相机与 IMU 所在板)。
 *
 *   · 大yaw 只有 yaw：lower_yaw 是「大yaw 旋转轴」的世界系绝对角，
 *     四个全向相机刚性挂在这根轴所在的支架上 -> R_大yaw2world = Rz(lower_yaw)。
 *   · 小yaw 板带 yaw + pitch + roll：upper_yaw 是其世界系绝对角，
 *     pitch/roll 也是这块板的姿态 -> R_小yaw2world = Rz(upper)·Ry(pitch)·Rx(roll)。
 *     主相机装在小yaw 上，所以这个姿态就是主相机的基座姿态。
 *   · 两者都相对「上电时车头朝向」为 0，范围 [0, 2π)，逆时针（左转）为正。
 *   · 只有哨兵是双yaw 平台；其他兵种把唯一的 yaw 填在 lower_yaw（upper_yaw 不使用）。
 *
 * 单位：弧度。pitch 低头为正、抬头为负，roll 绕 x 轴。
 * 长度：header(7) + 4×float(16) + roboid(1) + crc16(2) = 26 字节，
 *       Header.data_length 应为 17（= 26 - 7 - 2，与 SendPacket 的算法一致）。
 */
struct IMUPacket
{
  Header header;
  float pitch;
  float roll;
  float upper_yaw;
  float lower_yaw;
  uint8_t roboid;
  uint16_t crc16 = 0;
} __attribute__((packed));

/* 下位机按固定长度组帧，结构体一旦被改动必须同步固件，这里锁住长度防止误改 */
static_assert(sizeof(IMUPacket) == 26, "IMUPacket 必须是 26 字节，改动需同步下位机固件");

/* cmd_id 0x0503 — 底盘速度反馈 */
struct classisPacket
{
  Header header;
  float vx;
  float vy;
  float vz;
  uint8_t enable;  // 底盘是否使能
  uint16_t crc16 = 0;
} __attribute__((packed));

/* cmd_id 0x0504 — 按键 / 拨杆状态 */
struct bottonPacket
{
  Header header;
  bool start;
  uint16_t crc16 = 0;
} __attribute__((packed));

/*
 * cmd_id 0x0505 — 机器人基础状态（复位、血量、模式等）。
 *
 * 业务侧目前只使用 reset（下位机请求视觉复位）；其余字段仅在测试工具里解包打印。
 */
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

/*
 * cmd_id 0x0001 / 0x0100 — 比赛状态（本项目压缩格式）。
 *
 * game_info 的低 4 位是 game_type，高 4 位是 game_progress。
 * 数据段 11 字节（1 + 2 + 8），整帧 20 字节。
 *
 * ⚠ 这与 RM 裁判系统标准协议不同：标准 0x0001 的 game_type / game_progress
 *   是两个独立字节，数据段 12 字节、整帧 21 字节。下位机如果「原样转发」
 *   标准裁判帧，用下面的结构体会整体错位 1 字节，CRC16 必然失败，并且
 *   串口游标少前进 1 字节，会连累后续帧。两种格式都必须能解析，
 *   见 GameStatusPacketStd 与 SerialBoard 里按 header.data_length 的分派。
 */
struct GameStatusPacket
{
  Header header;
  uint8_t game_info;
  uint16_t stage_remain_time;
  uint64_t sync_timestamp;
  uint16_t crc16 = 0;
} __attribute__((packed));

/*
 * cmd_id 0x0001 / 0x0100 — 比赛状态（RM 裁判系统标准格式，下位机原样转发时用）。
 *
 *   uint8  game_type
 *   uint8  game_progress
 *   uint16 stage_remain_time
 *   uint64 SyncTimeStamp
 * 数据段 12 字节，整帧 21 字节。
 */
struct GameStatusPacketStd
{
  Header header;
  uint8_t game_type;
  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint64_t sync_timestamp;
  uint16_t crc16 = 0;
} __attribute__((packed));

// 两种格式只差 1 字节：靠这两个断言锁住，改动结构体时能立刻发现
static_assert(sizeof(GameStatusPacket) == 20, "压缩格式 0x0001 必须是 20 字节");
static_assert(sizeof(GameStatusPacketStd) == 21, "标准格式 0x0001 必须是 21 字节");

/*
 * cmd_id 0x0201 / 0x0102 — 机器人血量 / 射击 / 功率状态。
 *
 * mains_power_state 的 bit0/bit1/bit2 依次表示云台 / 底盘 / 发射机构输出。
 */
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

/*
 * cmd_id 0x0402 — 视觉 → 下位机的自瞄指令。
 *
 * 实际生效的只有 pitch / yaw / shoot：pitch、yaw 为目标角（弧度），
 * shoot 等于 control && shoot。id、robo_id、accuracy、timeseries 是协议保留位，
 * 当前恒为默认值（0 / 0 / 50 / 0），下位机不读取。
 *
 * 结构体布局必须与固件严格一致，固件按固定长度校验，长度不符整帧丢弃。
 */
struct SendPacket
{
  Header header;
  uint8_t id = 0;
  uint8_t robo_id = 0;
  float pitch = 0.0f;
  float yaw = 0.0f;
  uint8_t accuracy = 50;
  uint8_t shoot = 0;
  uint8_t timeseries = 0;
  uint16_t crc16 = 0;
} __attribute__((packed));

/*
 * cmd_id 0x0405 — 视觉 → 下位机的导航速度指令。
 *
 * vx 纵向速度、vy 横向速度（写入前已取反以适配下位机约定）、wz 角速度。
 */
struct NavigationPacket
{
  Header header;
  float vx;
  float vy;
  float wz;
  uint16_t crc16;
} __attribute__((packed, aligned(1)));

/* 从 GameStatusPacket 拆出来、业务侧实际使用的比赛状态 */
struct GameStatus
{
  Header header;
  uint8_t game_type;         // 比赛类型
  uint8_t game_progress;     // 比赛阶段 (4 = 比赛中)
  uint16_t stage_remain_time; // 阶段剩余时间
  uint64_t sync_timestamp;   // 同步时间戳
};

/* 从 GameRobotStatusPacket 拆出来、业务侧实际使用的机器人状态 */
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

/*
 * 按内存布局原样拷贝，依赖结构体的 packed 布局与下位机的小端字节序；
 * 不做字节序转换，跨大端平台使用需要另行处理。
 */
template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

/* 同上，把下行包序列化成待发送字节流 */
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
