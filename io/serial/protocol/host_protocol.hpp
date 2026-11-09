#ifndef IO__SERIAL__PROTOCOL__HOST_PROTOCOL_HPP
#define IO__SERIAL__PROTOCOL__HOST_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>

#include "io/serial/protocol/referee.hpp"

/*
 * UICRM Host 协议 v1.0 — MCU → minipc（上位机）上行字段。
 * 与下位机 boards/communication/host/include/host/protocol.h 逐字段对应，
 * 改动必须两边同步。
 *
 * 线格式与裁判系统一致（见 referee.hpp）：A5 帧头 + CRC8 + CRC16。
 * 两条流共用同一链路时，minipc 按 cmd_id 号段区分：
 *   裁判 0x0001~0x0A06（转发原始帧）  下行 0x04xx  上行 0x05xx
 *
 * 设计原则（沿用固件）：
 *   1. 裁判转发已覆盖的数据（功率/热量/血量/弹量/机器人 ID/弹速）不在此重复上传；
 *   2. 只传 MCU 独有或 MCU 为权威来源的数据；
 *   3. 演进遵守 append-only：字段只加不改不删，未知 cmd_id 接收端静默跳过；
 *   4. 帧长随字段变化即版本失配，须同步提升 HOST_PROTOCOL_VERSION。
 */

namespace host_comm
{

/** @brief 协议版本，字段布局变更时递增。 */
constexpr uint8_t HOST_PROTOCOL_VERSION = 1;

/**
 * @brief 上行命令码（MCU → minipc）。
 * @details 号段 0x05xx。删除的号位不重用，避免新旧固件混跑时语义错位。
 */
enum Cmd : uint16_t
{
  HOST_VERSION = 0x0500,   ///< 协议版本握手与硬件构型，1Hz，兼作 MCU 心跳
  ROBOT_STATUS = 0x0501,   ///< 机器人状态汇总，1Hz + 变化即发
  CHASSIS_STATUS = 0x0502, ///< 底盘速度反馈，100Hz
  GIMBAL_STATUS = 0x0503,  ///< 云台姿态与自瞄使能，500Hz，延迟敏感
  SHOOT_STATUS = 0x0504,   ///< 发射模块状态，50Hz
  HOST_LOG = 0x0505,       ///< 调试日志通道，事件驱动，可选
};

/* ===== 0x0500 协议版本与硬件构型 10 字节 1Hz ===== */
/**
 * @brief 链路握手与心跳。
 * @details 收到首帧视为链路建立；超过 2s 未收到判 MCU 失联。
 * uptime_ms 回落视为 MCU 重启，也可用于估算跨设备时钟偏移。
 */
typedef struct
{
  uint8_t host_version;    ///< 协议版本，见 HOST_PROTOCOL_VERSION
  uint16_t referee_season; ///< 2026/2027，提示 minipc 按哪套协议解析裁判转发帧
  uint8_t robot_type;      ///< 1 英雄，3/4 步兵，7 哨兵，9 雷达
  uint8_t chassis_type;    ///< 0 无底盘，1 mecanum，2 omni，3 steering，4 swerve，5 legs
  uint8_t gimbal_type;     ///< 0 无云台，1 gimbal，2 gimbal_DoubleYaw，3 gimbal_Fold
  uint32_t uptime_ms;      ///< 本板运行时间，ms
} __attribute__((packed)) host_version_t;
static_assert(sizeof(host_version_t) == 10, "0x0500 长度应为 10");

/* ===== 0x0501 机器人状态 6 字节 1Hz + 变化即发 ===== */
/**
 * @brief 机器人状态汇总。
 * @details 使能分两类：本帧 enable 是 MCU 状态机的软件使能（指挥意图）；
 * 裁判系统各电源口使能在 0x0201 的 power_management_* 位中，随裁判转发帧到达。
 * 血量与击杀信息同样由裁判 0x0201 推导，不在此重复。
 */
typedef struct
{
  uint8_t enable;            ///< 软件使能位域：bit0 底盘，bit1 云台，bit2 发射，bit3 超电，bit4~7 保留
  uint8_t controller_online; ///< 遥控激活状态，0 关闭，1 已激活
  uint8_t robot_mode;        ///< 0 无，1 跟随，2 小陀螺，3 全自动哨兵，4 坦克模式，5 高级调试
  uint8_t fric_mode;         ///< 摩擦轮，0 停，1 preparing，2 就绪
  uint8_t shoot_mode;        ///< 发射，0 stop，1 single，2 burst
  uint8_t module_online;     ///< 在线位域：bit0 底盘，bit1 云台，bit2 发射，bit3 IMU，bit4 裁判，bit5 超电
} __attribute__((packed)) host_robot_status_t;
static_assert(sizeof(host_robot_status_t) == 6, "0x0501 长度应为 6");

/* ===== 0x0502 底盘状态 17 字节 100Hz ===== */
/**
 * @brief 底盘实测速度与航线角，导航闭环用。
 * @details 速度取体坐标系：x 前、y 左、turn 逆时针为正。
 * feedback_angle 为底盘航向角反馈（世界系，IMU 零位），不是 IMU 的 yaw：
 * 小陀螺模式下两者相差云台相对角，导航须用本字段。
 */
typedef struct
{
  uint8_t chassis_enabled; ///< 底盘使能，0/1
  float speed_x;           ///< 实测前进速度，m/s，体坐标
  float speed_y;           ///< 实测左移速度，m/s，体坐标
  float speed_turn;        ///< 实测转向角速度，rad/s，逆时针为正
  float feedback_angle;    ///< 底盘航向角反馈，rad，世界系（IMU 零位），逆时针为正
} __attribute__((packed)) host_chassis_status_t;
static_assert(sizeof(host_chassis_status_t) == 17, "0x0502 长度应为 17");

/* ===== 0x0503 云台姿态与自瞄使能 22 字节 500Hz ===== */
/**
 * @brief 云台姿态、关节角与自瞄裁决。
 * @details pitch_angle/yaw_angle/roll_angle 为 IMU 世界系姿态（相机/枪管指向），
 * 自瞄闭环与端到端延迟测量用；数据源 INS_angle[1]/[0]/[2]，符号约定沿用旧 host 协议。
 * upper_yaw_angle 仅 gimbal_type=2（大小 yaw）有效，fold_yaw_angle 仅 gimbal_type=3
 * （折叠 yaw）有效，其余构型两者置 0。
 * gimbal_enable 为云台软件使能，0 时电机不驱动、角度仍会更新。
 * autoaim_enabled 是 MCU 的最终裁决（遥控开关、模式与激活状态共同决定），
 * minipc 只服从不自裁；该位随姿态流 500Hz 下发，翻转即时送达。
 */
typedef struct
{
  uint8_t gimbal_enable;   ///< 云台使能，0 关闭，1 使能
  uint8_t autoaim_enabled; ///< 1 自瞄激活，0 未激活
  float pitch_angle;       ///< 俯仰角，rad，IMU 世界系
  float yaw_angle;         ///< 偏航角，rad，IMU 世界系
  float roll_angle;        ///< 横滚角，rad，IMU 世界系
  float upper_yaw_angle;   ///< 小 yaw 关节角，rad，仅 gimbal_type=2 有效，其余置 0
  float fold_yaw_angle;    ///< 折叠关节角，rad，仅 gimbal_type=3 有效，其余置 0
} __attribute__((packed)) host_gimbal_status_t;
static_assert(sizeof(host_gimbal_status_t) == 22, "0x0503 长度应为 22");

/* ===== 0x0504 发射模块状态 5 字节 50Hz ===== */
/**
 * @brief 发射机构状态。
 * @details 热量、弹量、实测弹速由裁判 0x0202/0x0208/0x0207 随转发帧到达，不在此重复。
 * fric_state 是实测达速状态，与 0x0501 的 fric_mode（控制意图）不同。
 */
typedef struct
{
  uint8_t shoot_enable; ///< 发射模块使能，0/1
  uint8_t fric_state;   ///< 摩擦轮实际状态：0 停，1 加速中，2 已达速
  uint8_t load_state;   ///< 拨弹轮：0 停，1 正常，2 卡弹
  uint16_t shot_count;  ///< 累计发弹数，上电清零
} __attribute__((packed)) host_shoot_status_t;
static_assert(sizeof(host_shoot_status_t) == 5, "0x0504 长度应为 5");

/* ===== 0x0505 调试日志 62 字节，事件驱动，可选 ===== */
/** @brief 文本日志通道，突发限速建议不超过 50 帧/秒。 */
typedef struct
{
  uint8_t level;  ///< 0 debug，1 info，2 warn，3 error
  uint8_t length; ///< text 有效字节数，0~60
  char text[60];  ///< UTF-8 文本，非 NUL 结尾
} __attribute__((packed)) host_log_t;
static_assert(sizeof(host_log_t) == 62, "0x0505 长度应为 62");

/**
 * @brief 上行 host 帧解包实现（0x0500~0x0505）。
 * @details 每个命令都按固件结构体长度严格校验，长度不符即丢弃该帧，
 * 未知命令静默忽略，保证新旧固件在同一总线上互不干扰。
 * 并发：OnCommand 在接收线程中调用，跨线程读取状态须由调用方保证一致性
 * （本项目的用法是：接收线程解完帧后在自己的锁内拷走需要的字段）。
 */
class Protocol final : public referee_comm::Protocol
{
public:
  bool OnCommand(const referee_comm::Frame & frame) override;

  /* 未收到的命令保持零值 */
  host_version_t version{};
  host_robot_status_t robot{};
  host_chassis_status_t chassis{};
  host_gimbal_status_t gimbal{};
  host_shoot_status_t shoot{};
  host_log_t log{};

  /** @brief 0x0500 是否已收到（之后的 gimbal_type 才有意义）。 */
  bool version_valid() const { return version_valid_; }
  /** @brief 0x0503 的累计帧数，用于统计上报频率。 */
  uint64_t gimbal_count() const { return gimbal_count_; }

private:
  /**
   * @brief 校验长度并整体拷贝到目标状态。
   * @param[in] frame 完整帧视图。
   * @param[out] target 目标状态。
   * @param[in] expected 固件结构体长度。
   * @return 长度匹配并已拷贝时为 true。
   */
  template <typename T>
  bool Assign(const referee_comm::Frame & frame, T & target, size_t expected);

  bool version_valid_ = false;
  uint64_t gimbal_count_ = 0;
};

}  // namespace host_comm

#endif  // IO__SERIAL__PROTOCOL__HOST_PROTOCOL_HPP
