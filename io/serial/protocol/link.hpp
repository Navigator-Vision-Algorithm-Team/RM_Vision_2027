#ifndef IO__SERIAL__PROTOCOL__LINK_HPP
#define IO__SERIAL__PROTOCOL__LINK_HPP

#include <cstdint>
#include <functional>

#include "io/serial/protocol/host_protocol.hpp"
#include "io/serial/protocol/referee.hpp"
#include "io/serial/protocol/seasons/2027/protocol.hpp"

namespace io
{

/**
 * @brief 上位机侧链路解包：按 cmd_id 号段把帧分给对应解码器。
 * @details 下位机把裁判转发帧与自己的 host 帧放在同一条链路上，号段约定：
 *   裁判 0x0001~0x0A06（原样转发）  下行 0x04xx  上行 0x05xx
 * 因此 0x05xx 交给 host_comm，其余交给赛季协议；0x04xx 是上位机自己发的方向，
 * 正常不会收到，落到赛季协议后被静默忽略。
 * 并发：OnCommand 在接收线程中调用，on_frame 也在同一线程内同步回调。
 */
class MinipcLink final : public referee_comm::Protocol
{
public:
  bool OnCommand(const referee_comm::Frame & frame) override;

  /** @brief MCU 上行帧（0x0500~0x0505）。 */
  host_comm::Protocol host;
  /** @brief 下位机转发的裁判系统帧（按赛季解析）。 */
  referee_comm::v2027::Protocol referee;

  /**
   * @brief 每解出一帧就回调一次，参数为刚处理完的 cmd_id。
   * @note 指向的载荷在 host / referee 里，回调内直接读即可。
   */
  std::function<void(uint16_t cmd_id)> on_frame;

  /** @brief 云台构型（0 无，1 gimbal，2 gimbal_DoubleYaw，3 gimbal_Fold），来自 0x0500。 */
  uint8_t gimbal_type() const { return host.version.gimbal_type; }
};

}  // namespace io

#endif  // IO__SERIAL__PROTOCOL__LINK_HPP
