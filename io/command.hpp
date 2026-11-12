#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

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

}  // namespace io

#endif  // IO__COMMAND_HPP
