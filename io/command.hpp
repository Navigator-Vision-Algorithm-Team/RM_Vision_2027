#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

#include <cstdint>

/*
 * 视觉算法与通信层之间的公共数据结构：
 *   下行命令（Command）与上行裁判状态（RefereeState / ImuState）。
 * 通信由 io/relay 的 RelayClient 经 iceoryx2 完成，本文件不含任何设备或协议细节。
 */

namespace io
{
/*
 * 视觉下发给下位机的自瞄指令（0x0402）。
 *
 * yaw / pitch 单位弧度，是世界系绝对角。报文里**没有**"是否接管"的位，下位机对这两个
 * 角度无条件执行；所以"保持当前角度"必须由视觉侧显式下发当前姿态，不能靠留空或填 0。
 *
 * shoot 只是「请求开火」，真正是否发弹还要由下位机侧的热量/弹量保护最终裁定。
 *
 * 字段：shoot 是否请求开火；yaw / pitch 目标角；
 *       horizon_distance 无人机专有水平距离，其他兵种恒为 0。
 */
struct Command
{
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;
};

/*
 * 下位机上报、导航决策节点需要的那部分裁判状态（上行 0x0001 / 0x0201 帧）。
 *
 * 字段来源：
 *   game_progress  0x0001 的 game_info 高 4 位（0~15，按赛制 4 = 比赛中）
 *   current_hp     0x0201 的 remain_hp
 *   maximum_hp     0x0201 的 max_hp
 *
 * game_status_valid / robot_status_valid 表示「是否收到过对应帧」，与数据是否新鲜无关。
 * 转发侧必须按有效位决定发不发：导航侧把 maximum_hp == 0 解释为「满血值未知」并
 * 永久关闭低血返航判定，因此收到 0x0201 之前绝不能发一个 0 占位。
 */
struct RefereeState
{
  uint8_t game_progress = 0;
  int32_t current_hp = 0;
  int32_t maximum_hp = 0;
  bool game_status_valid = false;
  bool robot_status_valid = false;
};

/*
 * 一帧云台姿态（弧度），全部是「上电时车头朝向为 0」的世界系绝对角：
 *   大yaw(lower) 只有 yaw；pitch/roll 属于小yaw(upper)，即主相机所在的那块板。
 * 数据来源是下位机 0x0503，双yaw 的关节角由 SMALL_YAW - LOWER_YAW 复合得到。
 *
 * 放在本文件是因为它同时是「串口解析的输出」与「跨进程转发的载荷」，
 * 视觉侧（io::relay::RelayClient）与中转进程（info_relay）都只依赖这一个定义。
 */
struct ImuState
{
  double pitch = 0.0;      // 小yaw 板俯仰，低头为正
  double roll = 0.0;       // 小yaw 板横滚
  double upper_yaw = 0.0;  // 小yaw 世界系绝对 yaw，[0, 2π)
  double lower_yaw = 0.0;  // 大yaw 世界系绝对 yaw，[0, 2π)
};

}  // namespace io

#endif  // IO__COMMAND_HPP
