#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有
};

struct NavCommand
{
  double vx = 0;            // 体坐标系前进速度 m/s
  double vy = 0;            // 体坐标系左移速度 m/s
  double wz = 0;            // 转向角速度 rad/s
  double target_angle = 0;  // 目标航向角 rad（0x0405 的 target_angle，已确认暂时固定 0）
};


}  // namespace io

#endif  // IO__COMMAND_HPP