#ifndef IO__RELAY__RELAY_CODEC_HPP
#define IO__RELAY__RELAY_CODEC_HPP

/*
 * 线格式 <-> 进程内结构的显式映射：一个方向一个函数，逐字段赋值。
 *
 * 数据流：
 *   收：共享内存里的 info_relay::msg::Imu → io::ImuState（本工程算法规格）
 *   发：io::Command（世界系绝对角）→ info_relay::msg::AimCmd（线格式）
 *
 * 依赖与边界：
 *   只做字段搬运与单位/极角约定转换，不含插值、滤波或任何决策；
 *   不依赖结构体布局巧合——刻意不用 memcpy 解释整块内存，避免任一侧加字段后静默错位。
 *
 * 使用注意：
 *   所有时间戳由调用方给出，本层不读时钟（便于测试与回放）；
 *   aim 的 shoot 位由调用方按链路与热量状态决定，本层不擅自清零。
 */

#include <cstdint>

#include "io/command.hpp"
#include "io/relay/relay_msgs.hpp"

namespace io::relay
{

// 功能：把共享内存里的姿态样本还原成进程内姿态
// 输入：
//   sample  中转发布的 Imu 样本只读视图
// 输出：ImuState，角度单位 rad；lower_yaw 由 yaw 与关节角合成
// 备注：双yaw 构型下 upper_yaw_rad 是小yaw 相对大yaw 的关节角，
//       因此 lower_yaw = yaw_rad - upper_yaw_rad；单yaw 构型该字段为 0，两者相等
ImuState from_wire(const info_relay::msg::Imu & sample);

// 功能：把视觉侧指令打包成线格式
// 输入：
//   cmd     云台指令，yaw/pitch 单位 rad（世界系绝对角）
//   now_ns  本侧发出时刻，ns；写入 t_ns 供中转统计端到端延迟
// 输出：AimCmd 线格式载荷
// 备注：control/shoot 由调用方给定，本函数不修改其语义
info_relay::msg::AimCmd to_wire(const Command & cmd, int64_t now_ns);

}  // namespace io::relay

#endif  // IO__RELAY__RELAY_CODEC_HPP
