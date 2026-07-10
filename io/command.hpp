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
  double vx;
  double vy;
  double wz;
};


}  // namespace io

#endif  // IO__COMMAND_HPP