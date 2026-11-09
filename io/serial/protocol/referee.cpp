#include "io/serial/protocol/referee.hpp"

#include <cstring>

namespace referee_comm
{

size_t Receiver::Encode(
  uint8_t * out, size_t capacity, uint8_t seq, uint16_t cmd_id, const void * payload,
  uint16_t length)
{
  if (out == nullptr || length > MAX_PAYLOAD) return 0;
  const size_t total = HEADER_SIZE + CMD_ID_SIZE + static_cast<size_t>(length) + TAIL_SIZE;
  if (total > capacity) return 0;

  out[0] = SOF;
  out[1] = static_cast<uint8_t>(length & 0xFF);
  out[2] = static_cast<uint8_t>((length >> 8) & 0xFF);
  out[3] = seq;
  Append_CRC8_Check_Sum(out, HEADER_SIZE);

  out[HEADER_SIZE] = static_cast<uint8_t>(cmd_id & 0xFF);
  out[HEADER_SIZE + 1] = static_cast<uint8_t>((cmd_id >> 8) & 0xFF);
  if (length > 0 && payload != nullptr)
    std::memcpy(out + HEADER_SIZE + CMD_ID_SIZE, payload, length);

  Append_CRC16_Check_Sum(out, static_cast<uint32_t>(total));
  return total;
}

void Receiver::Feed(const uint8_t * data, size_t size)
{
  if (data == nullptr || size == 0) return;

  // 与下位机一致：整段输入即视为对端存活证据，与帧本身是否有效无关。
  has_rx_ = true;
  last_rx_ = std::chrono::steady_clock::now();

  for (size_t i = 0; i < size; ++i) {
    const uint8_t byte = data[i];
    if (used_ == 0 && byte != SOF) {
      stats_.garbage_bytes++;
      continue;
    }

    // 等待最长候选帧的分支已保证上界，此处仅作防御。
    if (used_ >= MAX_PACKET) Discard(1);
    buffer_[used_++] = byte;

    while (used_ > 0) {
      if (buffer_[0] != SOF) {
        Resync();
        continue;
      }

      if (used_ < HEADER_SIZE) break;
      const size_t payload =
        static_cast<size_t>(buffer_[1]) | (static_cast<size_t>(buffer_[2]) << 8);
      if (!Verify_CRC8_Check_Sum(buffer_, HEADER_SIZE) || payload > MAX_PAYLOAD) {
        if (payload > MAX_PAYLOAD)
          stats_.payload_errors++;
        else
          stats_.crc8_errors++;
        Resync();
        continue;
      }
      const size_t total = HEADER_SIZE + CMD_ID_SIZE + payload + TAIL_SIZE;
      if (used_ < total) break;
      if (!Verify_CRC16_Check_Sum(buffer_, static_cast<uint32_t>(total))) {
        stats_.crc16_errors++;
        Resync();
        continue;
      }

      Frame frame{};
      frame.cmd_id = static_cast<uint16_t>(buffer_[5]) | (static_cast<uint16_t>(buffer_[6]) << 8);
      frame.data = buffer_ + HEADER_SIZE + CMD_ID_SIZE;
      frame.length = static_cast<uint16_t>(payload);
      stats_.frames++;
      protocol_.OnCommand(frame);
      Discard(total);
    }
  }
}

void Receiver::Reset()
{
  used_ = 0;
}

void Receiver::Discard(size_t size)
{
  if (size > used_) size = used_;
  used_ -= size;
  if (size > 0 && used_ > 0) std::memmove(buffer_, buffer_ + size, used_);
}

void Receiver::Resync()
{
  size_t next = 1;
  while (next < used_) {
    if (buffer_[next] == SOF) break;
    ++next;
  }
  stats_.garbage_bytes += next;
  Discard(next);
}

}  // namespace referee_comm
