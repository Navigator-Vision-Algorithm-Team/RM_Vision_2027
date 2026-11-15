#ifndef IO__SERIAL__PROTOCOL__SEASONS__2027__PROTOCOL_HPP
#define IO__SERIAL__PROTOCOL__SEASONS__2027__PROTOCOL_HPP

#include <cstddef>
#include <cstdint>

#include "io/serial/protocol/referee.hpp"

/*
 * 2027 赛季载荷定义，依据《RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0》。
 * 与下位机 boards/communication/referee/include/seasons/2027/protocol.h 逐字段对应。
 *
 * 这些帧由下位机从裁判系统收下来后原样转发（forward_to），上位机自己解包。
 * 与 2026 相比的主要变化：
 *   0x0003 血量改为己方血量 + 伤害差 + 累计控制度（32 -> 24 字节）
 *   0x0201 新增 bullet_speed_limit（13 -> 17 字节）
 *   0x0202 保留位重排（13 -> 14 字节）
 *   0x0206 新增命中序号与命中坐标（1 -> 7 字节）
 *   0x0208 新增堡垒储备弹量（6 -> 8 字节）
 *   0x020D 扩展 sentry_info_2 / sentry_info_3（4 -> 14 字节）
 *   删除 0x0102 补给站、0x0205 空中能量、0x0302、0x0306
 *
 * 说明：这里只接线视觉侧实际消费的命令；其余命令码（地图交互 0x0303/0x0305/0x0307、
 * 雷达无线链路 0x0A0x、图传链路 0x0310/0x0311 等）按下位机 default 分支的约定静默
 * 忽略，需要时按同样格式追加 case 与结构体即可。
 */

namespace referee_comm
{
namespace v2027
{

/** @brief 2027 赛季命令码。 */
enum Cmd : uint16_t
{
  /* 常规链路 */
  GAME_STATUS = 0x0001,
  GAME_RESULT = 0x0002,
  GAME_ROBOT_HP = 0x0003,
  ROBOT_STATUS = 0x0201,
  POWER_HEAT_DATA = 0x0202,
  HURT_DATA = 0x0206,
  SHOOT_DATA = 0x0207,
  PROJECTILE_ALLOWANCE = 0x0208,
  SENTRY_INFO = 0x020D,
};

/**
 * @brief 机器人 ID（`0x0201.robot_id`，uint8_t）。
 * @details 依据《RoboMaster 2027 机甲大师高校系列赛通信协议》：红方 1~11，蓝方 101~111
 * （蓝方 ID = 红方 ID + 100）。2/102 为保留位。
 */
enum RobotId : uint8_t
{
  RED_HEAVY = 1,         ///< 红方重装机器人
  RED_RESERVED = 2,      ///< 红方保留位
  RED_INFANTRY_3 = 3,    ///< 红方 3 号步兵
  RED_INFANTRY_4 = 4,    ///< 红方 4 号步兵
  RED_INFANTRY_5 = 5,    ///< 红方 5 号步兵
  RED_AERIAL = 6,        ///< 红方空中系统
  RED_SENTRY = 7,        ///< 红方哨兵机器人
  RED_DART = 8,          ///< 红方飞镖
  RED_RADAR = 9,         ///< 红方雷达
  RED_BASE = 10,         ///< 红方基地
  RED_OUTPOST_R = 11,    ///< R 前哨

  BLUE_HEAVY = 101,      ///< 蓝方重装机器人
  BLUE_RESERVED = 102,   ///< 蓝方保留位
  BLUE_INFANTRY_3 = 103, ///< 蓝方 3 号步兵
  BLUE_INFANTRY_4 = 104, ///< 蓝方 4 号步兵
  BLUE_INFANTRY_5 = 105, ///< 蓝方 5 号步兵
  BLUE_AERIAL = 106,     ///< 蓝方空中系统
  BLUE_SENTRY = 107,     ///< 蓝方哨兵机器人
  BLUE_DART = 108,       ///< 蓝方飞镖
  BLUE_RADAR = 109,      ///< 蓝方雷达
  BLUE_BASE = 110,       ///< 蓝方基地
  BLUE_OUTPOST_B = 111,  ///< B 前哨
};

/** @brief 红方 ID 上界（1~11 为红方）。 */
constexpr uint8_t ID_RED_MAX = 11;
/** @brief 蓝方 ID = 红方 ID + 100（101~111 为蓝方）。 */
constexpr uint8_t ID_BLUE_OFFSET = 100;

/** @brief 是否红方 ID（1~11）。 */
constexpr bool IsRedId(uint8_t id) { return id >= 1 && id <= ID_RED_MAX; }
/** @brief 是否蓝方 ID（101~111）。 */
constexpr bool IsBlueId(uint8_t id)
{
  return id >= ID_BLUE_OFFSET + 1 && id <= ID_BLUE_OFFSET + ID_RED_MAX;
}
/** @brief 是否合法的机器人 ID（0 表示未收到）。 */
constexpr bool IsValidId(uint8_t id) { return IsRedId(id) || IsBlueId(id); }
/** @brief 换成对应的红方编号（107 -> 7），便于按兵种查表。 */
constexpr uint8_t RedEquivalentId(uint8_t id) { return IsBlueId(id) ? id - ID_BLUE_OFFSET : id; }

/** @brief 兵种名（红蓝共用，按红方编号），用于日志。 */
inline const char * RobotRoleName(uint8_t id)
{
  switch (RedEquivalentId(id)) {
    case RED_HEAVY:
      return "重装";
    case RED_RESERVED:
      return "保留位";
    case RED_INFANTRY_3:
      return "3号步兵";
    case RED_INFANTRY_4:
      return "4号步兵";
    case RED_INFANTRY_5:
      return "5号步兵";
    case RED_AERIAL:
      return "空中系统";
    case RED_SENTRY:
      return "哨兵";
    case RED_DART:
      return "飞镖";
    case RED_RADAR:
      return "雷达";
    case RED_BASE:
      return "基地";
    case RED_OUTPOST_R:
      return "前哨";
    default:
      return "未知";
  }
}

/* ===== 0x0001 比赛状态 11 字节 1Hz ===== */
typedef struct
{
  uint8_t game_type : 4;      ///< 1 超级对抗赛，2 单项赛，3 ICRA，4 联盟赛 3V3，5 联盟赛步兵对抗
  uint8_t game_progress : 4;  ///< 0 未开始，1 准备，2 自检，3 倒计时，4 比赛中，5 结算
  uint16_t stage_remain_time; ///< 当前阶段剩余时间，单位秒
  uint64_t SyncTimeStamp;     ///< NTP 同步的 UNIX 时间
} __attribute__((packed)) game_status_t;
static_assert(sizeof(game_status_t) == 11, "0x0001 长度应为 11");

/* ===== 0x0002 比赛结果 1 字节 ===== */
typedef struct
{
  uint8_t winner; ///< 0 平局，1 红方胜，2 蓝方胜
} __attribute__((packed)) game_result_t;
static_assert(sizeof(game_result_t) == 1, "0x0002 长度应为 1");

/* ===== 0x0003 机器人血量 24 字节 3Hz ===== */
typedef struct
{
  uint16_t ally_1_robot_HP;      ///< 己方 1 号重装血量
  uint16_t reserved;             ///< 保留位
  uint16_t ally_3_robot_HP;      ///< 己方 3 号步兵血量
  uint16_t ally_4_robot_HP;      ///< 己方 4 号步兵血量
  int16_t damage_difference;     ///< 己方与对方全队总伤害之差
  uint16_t ally_7_robot_HP;      ///< 己方 7 号哨兵血量
  uint16_t outpost_R_HP;         ///< R 前哨血量
  uint16_t ally_base_HP;         ///< 己方基地血量
  uint16_t outpost_B_HP;         ///< B 前哨血量
  uint16_t enemy_base_HP;        ///< 对方基地血量
  uint16_t ally_control_degree;  ///< 己方累计控制度
  uint16_t enemy_control_degree; ///< 对方累计控制度
} __attribute__((packed)) game_robot_HP_t;
static_assert(sizeof(game_robot_HP_t) == 24, "0x0003 长度应为 24");

/* ===== 0x0201 机器人性能体系 17 字节 10Hz ===== */
typedef struct
{
  uint8_t robot_id;                      ///< 本机器人 ID
  uint8_t robot_level;                   ///< 机器人等级
  uint16_t current_HP;                   ///< 当前血量
  uint16_t maximum_HP;                   ///< 血量上限
  uint16_t shooter_barrel_cooling_value; ///< 射击热量每秒冷却值
  uint16_t shooter_barrel_heat_limit;    ///< 射击热量上限
  uint16_t chassis_power_limit;          ///< 底盘功率上限
  float bullet_speed_limit;              ///< 射击初速度上限，2027 新增
  uint8_t power_management_gimbal_output : 1;  ///< gimbal 口 24V 输出
  uint8_t power_management_chassis_output : 1; ///< chassis 口 24V 输出
  uint8_t power_management_shooter_output : 1; ///< shooter 口 24V 输出
} __attribute__((packed)) robot_status_t;
static_assert(sizeof(robot_status_t) == 17, "0x0201 长度应为 17");

/* ===== 0x0202 缓冲能量与射击热量 14 字节 10Hz ===== */
typedef struct
{
  uint16_t reserved_0;               ///< 保留位
  uint16_t reserved_1;               ///< 保留位
  float reserved_2;                  ///< 保留位
  uint16_t buffer_energy;            ///< 缓冲能量，单位 J
  uint16_t shooter_17mm_barrel_heat; ///< 17mm 射击热量
  uint16_t shooter_42mm_barrel_heat; ///< 42mm 射击热量
} __attribute__((packed)) power_heat_data_t;
static_assert(sizeof(power_heat_data_t) == 14, "0x0202 长度应为 14");

/* ===== 0x0206 伤害状态 7 字节 ===== */
typedef struct
{
  uint8_t armor_id : 4;            ///< 装甲或测速模块 ID
  uint8_t HP_deduction_reason : 4; ///< 掉血类型
  uint16_t hit_sequence;           ///< 命中事件序号，每发递增
  int16_t hit_position_x;          ///< 命中位置 x，单位 mm
  int16_t hit_position_y;          ///< 命中位置 y，单位 mm
} __attribute__((packed)) hurt_data_t;
static_assert(sizeof(hurt_data_t) == 7, "0x0206 长度应为 7");

/* ===== 0x0207 实时射击 7 字节 ===== */
typedef struct
{
  uint8_t bullet_type;         ///< 1 为 17mm，2 为 42mm
  uint8_t shooter_number;      ///< 1 为 17mm 发射机构，3 为 42mm 发射机构
  uint8_t launching_frequency; ///< 弹丸射速，单位 Hz
  float initial_speed;         ///< 弹丸初速度，单位 m/s
} __attribute__((packed)) shoot_data_t;
static_assert(sizeof(shoot_data_t) == 7, "0x0207 长度应为 7");

/* ===== 0x0208 允许发弹量 8 字节 10Hz ===== */
typedef struct
{
  uint16_t projectile_allowance_17mm;     ///< 17mm 允许发弹量
  uint16_t projectile_allowance_42mm;     ///< 42mm 允许发弹量
  uint16_t remaining_gold_coin;           ///< 剩余金币
  uint16_t projectile_allowance_fortress; ///< 堡垒储备允许发弹量，2027 新增
} __attribute__((packed)) projectile_allowance_t;
static_assert(sizeof(projectile_allowance_t) == 8, "0x0208 长度应为 8");

/* ===== 0x020D 哨兵自主决策信息 14 字节 1Hz ===== */
typedef struct
{
  uint32_t sentry_info;   ///< 兑换与复活状态位域
  uint16_t sentry_info_2; ///< 姿态与脱战状态位域，2027 新增
  uint64_t sentry_info_3; ///< 姿态剩余时长位域，2027 新增
} __attribute__((packed)) sentry_info_t;
static_assert(sizeof(sentry_info_t) == 14, "0x020D 长度应为 14");

/**
 * @brief 2027 赛季解包实现。
 * @details 每个命令都按官方数据段长度严格校验，长度不符即丢弃该帧，
 * 未知命令静默忽略，保证新旧固件在同一总线上互不干扰。
 * 并发：OnCommand 在接收线程中调用，跨线程读取状态须由调用方保证一致性。
 */
class Protocol final : public ::referee_comm::Protocol
{
public:
  bool OnCommand(const Frame & frame) override;

  /* 未收到的命令保持零值 */
  game_status_t game_status{};
  game_result_t game_result{};
  game_robot_HP_t game_robot_hp{};
  robot_status_t robot_status{};
  power_heat_data_t power_heat_data{};
  hurt_data_t hurt_data{};
  shoot_data_t shoot_data{};
  projectile_allowance_t projectile_allowance{};
  sentry_info_t sentry_info{};

  /** @brief 比赛是否已进入进行中阶段。 */
  bool IsInBattle() const;
  /** @brief 本机器人是否已阵亡。 */
  bool IsDead() const;
  /** @brief 底盘剩余缓冲能量，单位 J。 */
  uint16_t BufferEnergy() const;
  /** @brief 17mm 枪口当前热量。 */
  uint16_t BarrelHeat17mm() const;

private:
  /**
   * @brief 校验长度并整体拷贝到目标状态。
   * @param[in] frame 完整帧视图。
   * @param[out] target 目标状态。
   * @param[in] expected 官方数据段长度。
   * @return 长度匹配并已拷贝时为 true。
   */
  template <typename T>
  bool Assign(const Frame & frame, T & target, size_t expected);
};

}  // namespace v2027
}  // namespace referee_comm

#endif  // IO__SERIAL__PROTOCOL__SEASONS__2027__PROTOCOL_HPP
