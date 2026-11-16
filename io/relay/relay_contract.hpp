#ifndef IO__RELAY__RELAY_CONTRACT_HPP
#define IO__RELAY__RELAY_CONTRACT_HPP

/*
 * 视觉侧与中转进程之间的字段契约：订阅什么、发布什么，逐字段写明来源、单位与用途。
 *
 * 数据流：
 *   中转进程（唯一持有串口/CAN 的进程）解析下位机协议 → 发布 imu / robot / game / health
 *   → 本工程的 RelayClient 订阅并转换为 io::ImuState 等进程内结构；
 *   本工程只发布 cmd_aim，由中转进程落地成下行报文。
 *
 * 依赖与边界：
 *   线格式定义在 io/relay/relay_msgs.hpp（与中转仓库逐字节兼容的副本），本文件只做
 *   声明与断言，不引入任何实现；
 *   info_relay 仓库对外提供 7 个主题，本工程**只消费其中 4 个**，不订阅 chassis，
 *   也不发布 cmd_nav —— 本项目不参与导航。
 *
 * 使用注意：
 *   任何字段增删都必须同时改中转仓库的 msg/relay_msgs.hpp 并递增 kSchemaVersion，
 *   否则运行期握手会失败；本文件中的 static_assert 只是本地防呆，不能替代两侧同步。
 */

#include <cstddef>
#include <cstdint>

#include "io/relay/relay_msgs.hpp"

namespace io::relay
{

// ---------------------------------------------------------------------------
// 订阅集合：只打开项目逻辑真正消费的主题
// ---------------------------------------------------------------------------

// chassis（底盘速度与航向）不订阅：它只服务导航闭环，本工程没有消费者，
// 订阅后只会增加唤醒次数与共享内存占用。
inline constexpr bool kSubscribeChassis = false;

// cmd_nav（导航速度指令）不发布：本工程不产生导航意图。
inline constexpr bool kPublishNavCmd = false;

// ---------------------------------------------------------------------------
// 从中转获取的字段（4 个主题）
//
// imu —— 云台姿态与自瞄裁决，500Hz，唯一的时间基准来源
//   t_ns            ns   姿态插值的时间基准
//   seq             —    丢样统计
//   uptime_ms       ms   数值回落判定 MCU 重启，触发 reset_pending()
//   gimbal_type     —    0 无云台 / 1 单yaw / 2 双yaw / 3 折叠
//   gimbal_enable   0/1  云台软件使能；0 时不应期望云台动作
//   autoaim_enabled 0/1  MCU 对自瞄的最终裁决，视觉只服从不自裁
//   pitch/yaw/roll  rad  合成 R_gimbal2world 送入 Solver
//   upper_yaw_rad   rad  小yaw 关节角；lower_yaw = yaw_rad - upper_yaw_rad
//   fold_yaw_rad    rad  本赛制恒为 0，仅保留解析
//
// robot —— 机器人状态，1Hz
//   robot_id          —    红方 1~11 / 蓝方 101~111，0 表示未收到
//   own_team_red      0/1/2  敌我颜色判定的权威来源（2 = 未知）
//   current_hp         —    当前血量
//   maximum_hp         —    血量上限
//   barrel_heat_17mm   热量单位  当前 17mm 热量，can_shoot() 的判据之一
//   heat_limit         热量单位  热量上限
//   valid_mask        位域  bit0 0x0201 / bit1 0x0202 / bit2 0x0501
//
// game —— 比赛状态，1Hz
//   in_battle      0/1   是否允许自瞄的门控条件
//   game_progress  —     0 未开始 / 4 比赛中，仅用于日志
//   valid          0/1   未收到 0x0001 时该帧不参与门控
//
// health —— 链路健康，10Hz
//   transport_ok     0/1  中转侧链路判定，与本地超时判定互相印证
//   last_rx_ns       ns   定位"是中转没收到还是视觉没收到"
//   schema_version   —    契约握手，不符即判链路失效
//   last_latency_us  us   端到端延迟观测，用于开火时序标定
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 向外发送的字段（仅 cmd_aim）
//
//   t_ns        ns   本工程发出时刻，中转用它计算端到端延迟
//   yaw_rad     rad  世界系绝对 yaw，不再叠加任何关节偏移
//   pitch_rad   rad  世界系绝对 pitch，向上为负
//   control     0/1  0 表示下位机忽略角度，1 表示接管云台
//   shoot       0/1  开火请求；链路失效或热量超限时强制置 0
//
// 下发策略：不做排队，每次只保留最新一条——旧指令没有意义，重发只会让云台晚到。
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 本地防呆：尺寸必须与中转的线格式一致（与其 msg 里的断言同值）
// ---------------------------------------------------------------------------
static_assert(sizeof(info_relay::msg::Imu) == 48, "Imu 线格式尺寸与中转不一致");
static_assert(sizeof(info_relay::msg::Robot) == 40, "Robot 线格式尺寸与中转不一致");
static_assert(sizeof(info_relay::msg::Game) == 24, "Game 线格式尺寸与中转不一致");
static_assert(sizeof(info_relay::msg::Health) == 48, "Health 线格式尺寸与中转不一致");
static_assert(sizeof(info_relay::msg::AimCmd) == 24, "AimCmd 线格式尺寸与中转不一致");

// 契约版本：与中转的 kSchemaVersion 必须相同，运行时还会在首个 Health 上再核一次
static_assert(info_relay::msg::kSchemaVersion == 3, "契约版本与中转不一致，需同步两侧头文件");

}  // namespace io::relay

#endif  // IO__RELAY__RELAY_CONTRACT_HPP
