#include "io/serial/protocol/link.hpp"

namespace io
{

bool MinipcLink::OnCommand(const referee_comm::Frame & frame)
{
  // 号段 0x05xx 是 MCU 上行 host 帧，其余（含裁判 0x0xxx 与下行 0x04xx）走赛季协议。
  const uint16_t segment = static_cast<uint16_t>(frame.cmd_id & 0xFF00);
  const bool handled = (segment == 0x0500) ? host.OnCommand(frame) : referee.OnCommand(frame);

  // 只有被识别并成功解出的帧才回调，避免未知命令反复触发上层逻辑。
  if (handled && on_frame) on_frame(frame.cmd_id);
  return handled;
}

}  // namespace io
