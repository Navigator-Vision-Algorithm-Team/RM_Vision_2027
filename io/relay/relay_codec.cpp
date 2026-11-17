/*
 * 线格式与进程内结构之间的映射实现。
 *
 * 注意：契约见 relay_codec.hpp，本文件只写实现要点。
 *       两个方向的字段一一对应，不做任何计算性处理（插值、限幅、限速都在客户端或算法侧）。
 */
#include "io/relay/relay_codec.hpp"

namespace io::relay
{

ImuState from_wire(const info_relay::msg::Imu & sample)
{
  ImuState state;
  state.pitch = sample.pitch_rad;
  state.roll = sample.roll_rad;
  state.upper_yaw = sample.yaw_rad;
  // 关节角是"小yaw 相对大yaw"的差值，因此大yaw 世界角 = 小yaw 世界角 - 关节角
  state.lower_yaw = static_cast<double>(sample.yaw_rad) - static_cast<double>(sample.upper_yaw_rad);
  return state;
}

info_relay::msg::AimCmd to_wire(const Command & cmd, int64_t now_ns)
{
  info_relay::msg::AimCmd wire{};
  wire.t_ns = static_cast<uint64_t>(now_ns);
  wire.yaw_rad = static_cast<float>(cmd.yaw);
  wire.pitch_rad = static_cast<float>(cmd.pitch);
  wire.shoot = cmd.shoot ? 1 : 0;
  return wire;
}

}  // namespace io::relay
