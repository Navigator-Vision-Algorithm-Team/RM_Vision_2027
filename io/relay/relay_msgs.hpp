#ifndef IO__RELAY__RELAY_MSGS_HPP
#define IO__RELAY__RELAY_MSGS_HPP

/*
 * 视觉侧的线格式副本：与 info_relay 仓库的 msg/relay_msgs.hpp **字节兼容**。
 *
 * 维护约定：两份文件必须同步修改，并同时递增 kSchemaVersion；
 *           只改一侧会让运行期握手失败（Health.schema_version 不符）。
 *           本文件不允许被 info_relay 仓库引用，反之亦然。
 *
 * 时间基准：所有 t_ns 均为本机 steady_clock 纳秒计数，禁止在订阅侧重新打时间戳。
 */

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace info_relay::msg
{
// 契约版本：字段或线格式任何变化都必须递增；订阅端收到首个 Health 后比对，不符即判链路不可用。
// 版本 3：与中转同步；Health 尾部填充位为 schema_version，用于运行期握手（尺寸仍 48 字节）。
inline constexpr uint16_t kSchemaVersion = 3;

// 主题名前缀，实际主题为 "<prefix>/<topic>"；前缀由配置文件给出，便于同机跑多套实例
inline constexpr const char * kDefaultPrefix = "rm_vision";

// 主题名后缀：relay 发布
inline constexpr const char * kTopicImu = "imu";          // 云台姿态，500Hz
inline constexpr const char * kTopicChassis = "chassis";  // 底盘反馈，100Hz
inline constexpr const char * kTopicRobot = "robot";      // 机器人状态，1Hz 及变化即发
inline constexpr const char * kTopicGame = "game";        // 比赛状态，1Hz
inline constexpr const char * kTopicHealth = "health";    // 链路健康，10Hz
// 主题名后缀：视觉发布
inline constexpr const char * kTopicCmdAim = "cmd_aim";   // 云台瞄准指令
inline constexpr const char * kTopicCmdNav = "cmd_nav";   // 导航速度指令

// 通信链路类型，用于 Health 对外说明当前链路形态
enum class SourceKind : uint8_t
{
  Serial = 0,  // 串口链路
  Can = 1,     // CAN 链路
};

// 云台姿态与自瞄裁决：源 0x0503（500Hz），gimbal_type / uptime_ms 取自 0x0500（1Hz）缓存
struct alignas(8) Imu
{
  uint64_t t_ns;             // relay 收到该帧的时刻，ns
  uint64_t seq;              // 递增序号，从 1 开始；用于统计丢样
  uint32_t uptime_ms;        // MCU 上电时间，ms；数值回落表示 MCU 重启
  uint8_t gimbal_type;       // 0 无云台 / 1 gimbal / 2 双yaw / 3 折叠
  uint8_t gimbal_enable;     // 云台软件使能，0 关 / 1 开；0 时电机不驱动但角度仍更新
  uint8_t autoaim_enabled;   // MCU 对自瞄的最终裁决，0/1；minipc 只服从、不自裁
  uint8_t reserved0;         // 对齐填充，恒为 0
  float pitch_rad;           // 俯仰角，rad，IMU 世界系
  float yaw_rad;             // 偏航角，rad，IMU 世界系
  float roll_rad;            // 横滚角，rad，IMU 世界系
  float upper_yaw_rad;       // 小 yaw 关节角，rad；仅 gimbal_type=2 有效，其余为 0
  float fold_yaw_rad;        // 折叠关节角，rad；仅 gimbal_type=3 有效，其余为 0
};

// 底盘实测速度与航向：源 0x0502（100Hz）
struct alignas(8) Chassis
{
  uint64_t t_ns;             // relay 收到该帧的时刻，ns
  uint64_t seq;              // 递增序号
  uint8_t chassis_enabled;   // 底盘使能，0/1
  uint8_t reserved0[3];      // 对齐填充，恒为 0
  float speed_x;             // 实测前进速度，m/s，体坐标（前为正）
  float speed_y;             // 实测左移速度，m/s，体坐标（左为正）
  float speed_turn;          // 实测转向角速度，rad/s，逆时针为正
  float feedback_angle;      // 底盘航向角，rad，世界系（IMU 零位）；不是 IMU 的 yaw
};

// 机器人状态汇总：源 0x0201（10Hz）+ 0x0202（10Hz）+ 0x0501（1Hz）
struct alignas(8) Robot
{
  uint64_t t_ns;                 // 最近一次更新的时刻，ns
  uint64_t seq;                  // 递增序号；任一来源更新都会递增
  uint8_t robot_id;              // 本机 ID；红方 1~11，蓝方 101~111，0 表示未收到
  uint8_t robot_level;           // 机器人等级，来自 0x0201
  uint8_t own_team_red;          // 己方是否红方：1 红 / 0 蓝 / 2 未知（ID 非法）
  uint8_t valid_mask;            // 有效位：bit0 0x0201，bit1 0x0202，bit2 0x0501
  uint16_t current_hp;           // 当前血量，来自 0x0201
  uint16_t maximum_hp;           // 血量上限，来自 0x0201
  uint16_t barrel_heat_17mm;     // 当前 17mm 热量，来自 0x0202
  uint16_t heat_limit;           // 热量上限，来自 0x0201
  uint16_t cooling_value;        // 每秒冷却值，来自 0x0201
  uint16_t buffer_energy;        // 底盘缓冲能量，J，来自 0x0202
  float bullet_speed_limit;      // 弹速上限，m/s，来自 0x0201（2027 新增字段）
  uint8_t enable_bits;           // 软件使能位域：bit0 底盘 bit1 云台 bit2 发射 bit3 超电
  uint8_t robot_mode;            // 0 无 1 跟随 2 小陀螺 3 全自动哨兵 4 坦克 5 高级调试
  uint8_t module_online;         // 在线位域：bit0 底盘 bit1 云台 bit2 发射 bit3 IMU bit4 裁判
  uint8_t reserved0;             // 对齐填充，恒为 0
};

// 比赛状态：源 0x0001（1Hz）
struct alignas(8) Game
{
  uint64_t t_ns;            // relay 收到该帧的时刻，ns
  uint64_t seq;             // 递增序号
  uint8_t game_type;        // 1 超级对抗 2 单项 3 ICRA 4 联盟3V3 5 步兵对抗
  uint8_t game_progress;    // 0 未开始 1 准备 2 自检 3 倒计时 4 比赛中 5 结算
  uint8_t in_battle;        // 1 表示 game_progress == 4，视觉侧据此决定是否自瞄
  uint8_t valid;            // 是否收到过 0x0001；0 时其余字段无意义
  uint16_t stage_remain_s;  // 当前阶段剩余时间，s
  uint16_t reserved0;       // 对齐填充，恒为 0
};

// 链路健康：relay 自身统计，10Hz 定时发布；链路异常时照常发布，靠字段表达状态
struct alignas(8) Health
{
  uint64_t t_ns;            // 本帧发布时刻，ns
  uint64_t seq;             // 递增序号
  uint64_t last_rx_ns;      // 最后一次收到合法上行帧的时刻，ns；0 表示从未收到
  uint32_t rx_frames;       // 累计解析成功的上行帧数
  uint32_t crc_errors;      // 累计校验失败被丢弃的帧数
  uint32_t tx_commands;     // 累计写入串口的下行指令数
  uint32_t rx_hz_x100;      // 最近 1s 上行帧率 ×100（定点表示，10 表示 0.10Hz）
  uint8_t transport_ok;     // 1 表示链路健康（recent_timeout_ms 内有合法帧）
  uint8_t source_kind;      // 当前通信链路：0 serial / 1 can
  uint16_t last_latency_us; // 最近一帧 AimCmd 从视觉发出到写入串口的耗时，us
  uint16_t schema_version;  // 契约版本，取 kSchemaVersion；订阅端据此做运行期握手
  uint16_t reserved1;       // 对齐填充，恒为 0；保留给后续追加字段
};

// 视觉下发的云台瞄准指令：打包成 0x0402 写入串口
struct alignas(8) AimCmd
{
  uint64_t t_ns;         // 视觉侧发出时刻，ns；relay 用它计算端到端延迟
  float yaw_rad;         // 世界系绝对 yaw，rad
  float pitch_rad;       // 世界系绝对 pitch，rad，向上为负
  uint8_t shoot;         // 1 表示请求发弹
  uint8_t reserved0[3];  // 对齐填充，恒为 0；保证 AimCmd 仍为 24 字节
};

// 导航速度指令：打包成 0x0405 写入串口
struct alignas(8) NavCmd
{
  uint64_t t_ns;          // 产生时刻，ns
  float vx;               // 体坐标系前进速度，m/s
  float vy;               // 体坐标系左移速度，m/s；写报文前会取反，见 io::NavCommand
  float wz;               // 转向角速度，rad/s，逆时针为正
  uint8_t source;         // 0 导航节点 / 1 视觉进程
  uint8_t reserved0[3];   // 对齐填充，恒为 0
};

// ---------------------------------------------------------------------------
// 契约固化：跨进程共享内存按固定尺寸映射，尺寸变化必须同步递增 kSchemaVersion
// ---------------------------------------------------------------------------
static_assert(std::is_trivially_copyable_v<Imu>, "Imu 必须是 trivially copyable");
static_assert(std::is_trivially_copyable_v<Chassis>, "Chassis 必须是 trivially copyable");
static_assert(std::is_trivially_copyable_v<Robot>, "Robot 必须是 trivially copyable");
static_assert(std::is_trivially_copyable_v<Game>, "Game 必须是 trivially copyable");
static_assert(std::is_trivially_copyable_v<Health>, "Health 必须是 trivially copyable");
static_assert(std::is_trivially_copyable_v<AimCmd>, "AimCmd 必须是 trivially copyable");
static_assert(std::is_trivially_copyable_v<NavCmd>, "NavCmd 必须是 trivially copyable");

static_assert(std::is_standard_layout_v<Imu>, "Imu 必须是标准布局");
static_assert(std::is_standard_layout_v<Chassis>, "Chassis 必须是标准布局");
static_assert(std::is_standard_layout_v<Robot>, "Robot 必须是标准布局");
static_assert(std::is_standard_layout_v<Game>, "Game 必须是标准布局");
static_assert(std::is_standard_layout_v<Health>, "Health 必须是标准布局");
static_assert(std::is_standard_layout_v<AimCmd>, "AimCmd 必须是标准布局");
static_assert(std::is_standard_layout_v<NavCmd>, "NavCmd 必须是标准布局");

static_assert(sizeof(Imu) == 48, "Imu 尺寸变化即破坏性变更，需递增 kSchemaVersion");
static_assert(sizeof(Chassis) == 40, "Chassis 尺寸变化即破坏性变更，需递增 kSchemaVersion");
static_assert(sizeof(Robot) == 40, "Robot 尺寸变化即破坏性变更，需递增 kSchemaVersion");
static_assert(sizeof(Game) == 24, "Game 尺寸变化即破坏性变更，需递增 kSchemaVersion");
static_assert(sizeof(Health) == 48, "Health 尺寸变化即破坏性变更，需递增 kSchemaVersion");
static_assert(sizeof(AimCmd) == 24, "AimCmd 尺寸变化即破坏性变更，需递增 kSchemaVersion");
static_assert(sizeof(NavCmd) == 24, "NavCmd 尺寸变化即破坏性变更，需递增 kSchemaVersion");

static_assert(alignof(Imu) == 8, "Imu 必须 8 字节对齐");
static_assert(alignof(AimCmd) == 8, "AimCmd 必须 8 字节对齐");

}  // namespace info_relay::msg

#endif  // IO__RELAY__RELAY_MSGS_HPP
