#ifndef IO__SERIAL__PROTOCOL__REFEREE_HPP
#define IO__SERIAL__PROTOCOL__REFEREE_HPP

#include <chrono>
#include <cstddef>
#include <cstdint>

#include "io/serial/gimbal/protocol_crc.hpp"

namespace referee_comm
{

/*
 * 裁判系统线格式（2018~2027 各赛季一致，改动需先核实官方规则）：
 *
 *   frame_header            cmd_id   data      frame_tail
 *   SOF(1) len(2) seq(1)    (2)      (n)       CRC16(2)
 *   其中 len 为 data 字节数，frame_header 尾部含 CRC8(1)。
 *
 * 与下位机 boards/communication/referee 逐字节对应：下位机把裁判帧原样转发给上位机，
 * MCU 自己产生的 host 帧（0x05xx）复用同一套线格式，上位机按 cmd_id 号段区分
 * （见 io/serial/protocol/link.hpp）。
 */

/** @brief 数据帧起始字节。 */
constexpr uint8_t SOF = 0xA5;

/** @brief 帧头长度：SOF(1) + len(2) + seq(1) + CRC8(1)。 */
constexpr size_t HEADER_SIZE = 5;
/** @brief 命令码长度。 */
constexpr size_t CMD_ID_SIZE = 2;
/** @brief 帧尾长度：CRC16(2)。 */
constexpr size_t TAIL_SIZE = 2;

/**
 * @brief 单帧最大容量，单位字节。
 * @details 需容纳 2027 赛季图传链路的 0x0310（300 字节数据段），
 * 因此不小于 300 + HEADER_SIZE + CMD_ID_SIZE + TAIL_SIZE。
 */
constexpr size_t MAX_PACKET = 320;
/** @brief data 段最大字节数。 */
constexpr size_t MAX_PAYLOAD = MAX_PACKET - HEADER_SIZE - CMD_ID_SIZE - TAIL_SIZE;

// 校验算法与固件共用同一份查表实现（io/serial/gimbal/protocol_crc.cpp），
// 命名与下位机保持一致，方便两边逐行对照。
// 注意：这不是"标准" CRC16-CCITT 位运算版本，不可替换。
using crc8::Append_CRC8_Check_Sum;
using crc8::Verify_CRC8_Check_Sum;
using crc16::Append_CRC16_Check_Sum;
using crc16::Verify_CRC16_Check_Sum;

/** @brief 已通过 CRC16 校验的完整数据帧视图，生命周期止于 OnCommand 返回。 */
struct Frame
{
  uint16_t cmd_id = 0;            ///< 命令码。
  const uint8_t * data = nullptr; ///< data 段首地址，指向分帧器内部缓存。
  uint16_t length = 0;            ///< data 段字节数。
};

/**
 * @brief 载荷解码与编码接口。
 * @details 线格式由 Receiver 保证，本接口只承担 cmd_id 到字段的映射。
 * 每个赛季 / 每条链路独立实现一份，互不影响。
 */
class Protocol
{
public:
  virtual ~Protocol() = default;

  /**
   * @brief 处理一帧已通过校验的数据。
   * @param[in] frame 完整帧视图，不得在返回后保留指针。
   * @return 命令码被本实现识别并处理时为 true。
   * @note 返回 false 表示未知或已废弃命令，不得据此丢弃字节流。
   */
  virtual bool OnCommand(const Frame & frame) = 0;

  /**
   * @brief 按命令码生成待发送的 data 段（本项目暂未使用，发送侧仍是固定结构体）。
   * @return 写入的字节数；不发送该命令时返回 -1。
   */
  virtual int OnBuild(uint16_t /*cmd_id*/, uint8_t * /*out*/, size_t /*capacity*/) { return -1; }
};

/** @brief 分帧统计，用于运行期自检与日志。 */
struct Stats
{
  uint64_t frames = 0;         ///< 校验通过并已分发的帧数。
  uint64_t payload_errors = 0; ///< len 超过 MAX_PAYLOAD 的帧数。
  uint64_t crc8_errors = 0;    ///< 帧头 CRC8 校验失败次数。
  uint64_t crc16_errors = 0;   ///< 整帧 CRC16 校验失败次数。
  uint64_t garbage_bytes = 0;  ///< 重同步过程中丢弃的字节数。
};

/**
 * @brief 裁判系统 / 上位机链路的接收分帧器。
 * @details 对应下位机 referee_comm::Referee 的分帧部分，只是不绑串口与线程：
 * 调用方把任意长度字节流喂进 Feed()，本类负责半包、粘包、连续帧及校验失败后的
 * 后缀恢复，再把通过 CRC 的 data 段交给 Protocol 解包。
 * 并发：Feed 只允许单一线程调用（本项目是 SerialBoard 的接收线程）。
 */
class Receiver
{
public:
  explicit Receiver(Protocol & protocol) : protocol_(protocol) {}

  /**
   * @brief 输入任意长度字节流，完成切帧、校验与分发。
   * @param[in] data 接收字节地址，空指针直接返回。
   * @param[in] size 字节数。
   * @note 支持半包、粘包、连续帧及校验失败后的后缀恢复。
   */
  void Feed(const uint8_t * data, size_t size);

  /** @brief 丢弃暂存半包（串口重连 / 回放跳转时调用）。 */
  void Reset();
  /** @brief 是否已经收到过任何字节（用于判断链路是否建立）。 */
  bool has_rx() const { return has_rx_; }
  /** @brief 最近一次收到字节的时间。 */
  std::chrono::steady_clock::time_point last_rx() const { return last_rx_; }
  /** @brief 分帧统计（只增不减，重连不清零）。 */
  const Stats & stats() const { return stats_; }

  /**
   * @brief 将 data 段封装为完整帧。
   * @param[out] out 目标缓冲，至少 capacity 字节。
   * @param[in] capacity 目标缓冲容量。
   * @param[in] seq 包序号，由调用方维护。
   * @param[in] cmd_id 命令码。
   * @param[in] payload data 段地址，length 为 0 时可为空。
   * @param[in] length data 段字节数。
   * @return 完整帧字节数；参数非法时返回 0。
   */
  static size_t Encode(
    uint8_t * out, size_t capacity, uint8_t seq, uint16_t cmd_id, const void * payload,
    uint16_t length);

private:
  /** @brief 从缓存头部丢弃 size 字节。 */
  void Discard(size_t size);
  /** @brief 丢弃到下一个可能的帧头，用于校验失败后的边界恢复。 */
  void Resync();

  Protocol & protocol_;

  uint8_t buffer_[MAX_PACKET] = {}; ///< 接收与解帧共用缓存。
  size_t used_ = 0;                 ///< 缓存中待处理字节数。
  Stats stats_{};
  bool has_rx_ = false;
  std::chrono::steady_clock::time_point last_rx_{};
};

}  // namespace referee_comm

#endif  // IO__SERIAL__PROTOCOL__REFEREE_HPP
