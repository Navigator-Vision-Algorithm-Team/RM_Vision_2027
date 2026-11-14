#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

#include <cstdint>

/*
 * io 层与下位机之间的公共数据结构：
 *   下行命令（Command / NavCommand）与上行裁判状态（RefereeState）。
 * 串口收发（io/serial）与 ROS 转发（io/ros2）都只依赖本文件，互不依赖对方的头文件。
 */

namespace io
{
/*
 * 视觉下发给下位机的自瞄指令（0x0402）。
 *
 * yaw / pitch 单位弧度，含义（绝对角还是相对角）由下位机协议约定，本层不做坐标变换；
 * control 为 false 时下位机应忽略 yaw / pitch；shoot 只是「请求开火」，
 * 真正是否发弹还要由下位机侧的热量/弹量保护最终裁定。
 *
 * 字段：control 是否接管云台；shoot 是否请求开火；yaw / pitch 目标角；
 *       horizon_distance 无人机专有水平距离，其他兵种恒为 0。
 */
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;
};

/*
 * 视觉下发给下位机的导航指令（0x0405），只表达底盘速度意图。
 *
 * vy 在本结构体中按「左移为正」填写，SerialBoard::send(NavCommand) 写入报文前会取反；
 * vx / wz 的正方向由下位机协议约定。
 */
struct NavCommand
{
  double vx = 0;
  double vy = 0;
  double wz = 0;
};

/*
 * 下位机上报、导航决策节点与本地日志共用的裁判状态（上行 0x0001 / 0x0201 帧）。
 *
 * 字段来源：
 *   game_type         0x0001 的 game_info 低 4 位
 *   game_progress     0x0001 的 game_info 高 4 位（0~15，按赛制 4 = 比赛中）
 *   stage_remain_time 0x0001 的阶段剩余时间（秒）
 *   current_hp        0x0201 的 remain_hp
 *   maximum_hp        0x0201 的 max_hp
 *
 * game_status_valid / robot_status_valid 表示「是否收到过对应帧」，与数据是否新鲜无关。
 * 转发侧必须按有效位决定发不发：导航侧把 maximum_hp == 0 解释为「满血值未知」并
 * 永久关闭低血返航判定，因此收到 0x0201 之前绝不能发一个 0 占位。
 */
struct RefereeState
{
  uint8_t game_type = 0;
  uint8_t game_progress = 0;
  uint16_t stage_remain_time = 0;
  int32_t current_hp = 0;
  int32_t maximum_hp = 0;
  bool game_status_valid = false;
  bool robot_status_valid = false;
};

}  // namespace io

#endif  // IO__COMMAND_HPP
