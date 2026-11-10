// ============================================================================
//  time_delay_test — 视觉 -> 下位机 串口延迟探针（只发不收，不回环）
//
//  用途：上位机按固定频率把带「发送时刻」的探针包发给下位机，延迟由下位机侧
//  自己的代码用该时间戳解算。本工具不接收、不回环、不等待应答，只保证：
//    · 按给定频率稳定发送（绝对时间调度，不累积漂移）
//    · 每帧带上单调递增的发送时间戳，并打印本次运行的时间基准
//    · 统计实际发送频率、发送周期抖动、写串口耗时，方便判断「发包本身」是否可信
//
//  ---- 帧格式（与 0x0402 / 0x0502 等现有帧完全一致）----
//   偏移 0    1      2        3      4      5      6      7 ... 26  27  28
//        A5 | len_lo len_hi | seq | crc8 | cmd_lo cmd_hi | data(20B) | crc16(2B)
//    · cmd_id = 0x0410（固件按此指令号解析；若与已有指令冲突，改 kDefaultCmd）
//    · len    = 20，只算 data 段（不含 7B header 与 2B crc16），整帧 29 字节
//    · crc8   = CRC8(前 4 字节)，位于偏移 4
//    · crc16  = CRC16(整帧 29 字节)，位于帧尾
//    · 多字节整数一律小端（x86_64 / aarch64 都是小端）
//
//  ---- data 段（20 字节）----
//   偏移 0   uint32 seq       发送序号，每帧 +1（下位机可用它统计丢帧/乱序）
//   偏移 4   uint64 tx_us     发送时刻：steady_clock 微秒计数，自本工具启动为 0，
//                             单调递增、不回绕。只取低 32 位使用时回绕周期 71.6 min，
//                             差值用 uint32 无符号减法即可（不要在 MCU 上做符号比较）
//   偏移 12  uint32 boot_id   本次启动的随机标识：值变化 = 上位机重启过，tx_us
//                             会重新从 0 开始，下位机必须重置本地时间基准
//   偏移 16  uint32 magic     固定 0x4C415445（ASCII "LATE"），用于确认包完整/对齐
//
//  ---- 下位机侧等价结构体（29 字节，可直接 memcpy 映射）----
//   typedef struct __attribute__((packed)) {
//     uint8_t  sof;           // 0xA5
//     uint16_t data_length;   // 20
//     uint8_t  header_seq;    // 本工具填发送序号低 8 位，仅供日志
//     uint8_t  crc8;
//     uint16_t cmd_id;        // 0x0410
//     uint32_t seq;
//     uint64_t tx_us;         // 小端
//     uint32_t boot_id;
//     uint32_t magic;         // 0x4C415445
//     uint16_t crc16;
//   } vision_delay_probe_t;
//
//  ---- 下位机侧（固件）怎么用 ----
//    · 在串口「收完整帧」那一刻记 MCU tick，配合 tx_us 即可解算延迟；
//    · 只关心「串口传输 + 中断响应」时，标定一次固定 offset 即可；
//    · 要从视觉出帧算到执行时刻，用 (t_mcu - tx_us) 再减去一次标定出的 offset。
//    具体算法由下位机侧实现；本工具只负责稳定发送并给出时间基准（启动时会打印
//    tx_us=0 对应的墙钟时间与 boot_id，便于两侧日志对齐）。
//
//  ---- 用法 ----
//    直接运行，不需要任何参数（固定配置见 main 里的 Options 默认值）：
//        ./time_delay_test
//    等价于 /dev/ttyACM0 921600 100Hz 发 30s，逐帧写 ./delay.csv。
//    Ctrl+C 提前结束；退出时打印发送统计。
//
//  ---- 注意 ----
//    · 本工具不读串口：下位机持续上报的数据会在 PC 端被丢弃，不影响发包；
//      若固件侧有流控（RTS/CTS），本工具显式关闭，不使用硬件流控。
//    · 写串口是阻塞调用，如果下位机那边长时间不取数据导致发送缓冲满，写耗时会
//      明显变大——统计里的「写串口耗时」就是用来发现这种情况的。
//    · 921600 8N1 下 29 字节帧的纯线路时间 ≈ 0.32 ms，可作为延迟的下限参考。
// ============================================================================

#include <serial/serial.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "io/serial/gimbal/packet.hpp"
#include "io/serial/gimbal/protocol_crc.hpp"

namespace
{

using Clock = std::chrono::steady_clock;

std::atomic<bool> g_quit{false};

void on_signal(int /*sig*/) { g_quit = true; }

// ---------------------------------------------------------------------------
// 协议常量（与下位机约定，详见文件头）
// ---------------------------------------------------------------------------
constexpr uint8_t kSof = 0xA5;
constexpr uint16_t kDefaultCmd = 0x0410;  // 视觉 -> 下位机：延迟探针
constexpr uint32_t kMagic = 0x4C415445;   // ASCII 'LATE'
constexpr uint16_t kPayloadSize = 20;     // data 段长度

// 帧头直接复用 io::Header（A5 | data_length(2) | seq | crc8 | cmd_id(2)），
// 保证与 0x0402 / 0x0502 等现有帧逐字节一致。
struct DelayProbe
{
  io::Header header;
  uint32_t seq;
  uint64_t tx_us;
  uint32_t boot_id;
  uint32_t magic;
  uint16_t crc16;
} __attribute__((packed));

static_assert(sizeof(io::Header) == 7, "帧头必须是 7 字节，否则帧内偏移全部失效");
static_assert(
  sizeof(DelayProbe) == sizeof(io::Header) + kPayloadSize + 2,
  "DelayProbe 必须是 29 字节（7 + 20 + 2），改动需同步下位机固件");

uint64_t absolute_us()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch())
      .count());
}

std::string to_hex(const uint8_t * data, size_t size)
{
  static const char * digits = "0123456789ABCDEF";
  std::string out;
  out.reserve(size * 3);
  for (size_t i = 0; i < size; ++i) {
    out.push_back(' ');
    out.push_back(digits[data[i] >> 4]);
    out.push_back(digits[data[i] & 0x0F]);
  }
  return out;
}

std::string wall_time_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&seconds, &local);

  char date[32];
  std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &local);
  char out[64];
  std::snprintf(out, sizeof(out), "%s.%03lld", date, static_cast<long long>(ms.count()));
  return out;
}

// ---------------------------------------------------------------------------
// 参数与统计
// ---------------------------------------------------------------------------
// 无需命令行参数：直接 ./time_delay_test 就按下面的固定配置跑
struct Options
{
  std::string port = "/dev/ttyACM0";
  uint32_t baud = 921600;
  double hz = 100.0;                  // 探针发送频率
  double duration_s = 30.0;           // 运行 30s 后自动结束（Ctrl+C 可提前）
  std::string csv_path = "delay.csv"; // 逐帧记录，留空则不写
  uint16_t cmd = kDefaultCmd;
};

double percentile_ms(const std::vector<uint32_t> & sorted_us, double p)
{
  if (sorted_us.empty()) return 0.0;
  const double idx = p * static_cast<double>(sorted_us.size() - 1);
  return sorted_us[static_cast<size_t>(idx + 0.5)] / 1000.0;
}

// ---------------------------------------------------------------------------
// 发送主体
// ---------------------------------------------------------------------------
class ProbeSender
{
public:
  ProbeSender(serial::Serial & port, Options opt)
  : port_(port), opt_(std::move(opt)), boot_id_(make_boot_id()), t0_us_(absolute_us())
  {
  }

  int run();

private:
  static uint32_t make_boot_id()
  {
    std::random_device rd;
    return static_cast<uint32_t>(rd());
  }

  uint64_t now_us() const { return absolute_us() - t0_us_; }

  DelayProbe make_probe(uint32_t seq, uint64_t tx_us) const;
  bool send_one();
  void report();
  void print_summary() const;

  serial::Serial & port_;
  const Options opt_;
  const uint32_t boot_id_;
  const uint64_t t0_us_;

  uint32_t seq_ = 0;
  uint64_t sent_ = 0;

  bool has_last_tx_ = false;
  uint64_t last_tx_us_ = 0;
  uint64_t first_tx_us_ = 0;

  // 发送周期、写串口耗时（us）
  std::vector<uint32_t> periods_us_;
  uint64_t write_sum_us_ = 0;
  uint32_t write_min_us_ = UINT32_MAX;
  uint32_t write_max_us_ = 0;

  // 每秒统计用的游标
  uint64_t window_first_seq_ = 0;
  uint64_t window_start_us_ = 0;
  uint32_t window_max_period_us_ = 0;
  uint64_t window_write_sum_us_ = 0;
  uint64_t window_write_count_ = 0;

  std::ofstream csv_;
};

DelayProbe ProbeSender::make_probe(uint32_t seq, uint64_t tx_us) const
{
  DelayProbe pkt{};
  pkt.header.header = kSof;
  pkt.header.data_length = kPayloadSize;
  pkt.header.seq = static_cast<uint8_t>(seq);
  pkt.header.cmd_id = opt_.cmd;
  crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&pkt.header), sizeof(io::Header) - 2);

  pkt.seq = seq;
  pkt.tx_us = tx_us;
  pkt.boot_id = boot_id_;
  pkt.magic = kMagic;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&pkt), sizeof(DelayProbe));
  return pkt;
}

bool ProbeSender::send_one()
{
  const uint64_t tx_us = now_us();
  const uint64_t wall_ms = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count());
  const DelayProbe pkt = make_probe(seq_, tx_us);

  const uint64_t w0 = absolute_us();
  try {
    port_.write(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[ERR] 下发失败: %s\n", e.what());
    return false;
  }
  const uint32_t write_us = static_cast<uint32_t>(absolute_us() - w0);

  // 发送周期：相邻两帧时间戳之差（第一帧没有参照，跳过）
  if (has_last_tx_) {
    const uint32_t period_us = static_cast<uint32_t>(tx_us - last_tx_us_);
    if (periods_us_.size() < 4'000'000) periods_us_.push_back(period_us);
    window_max_period_us_ = std::max(window_max_period_us_, period_us);
  } else {
    first_tx_us_ = tx_us;
    window_start_us_ = tx_us;
    window_first_seq_ = seq_;
  }
  has_last_tx_ = true;
  last_tx_us_ = tx_us;

  write_min_us_ = std::min(write_min_us_, write_us);
  write_max_us_ = std::max(write_max_us_, write_us);
  write_sum_us_ += write_us;
  window_write_sum_us_ += write_us;
  window_write_count_++;

  const uint32_t period_us = periods_us_.empty() ? 0 : periods_us_.back();
  if (csv_.is_open()) {
    csv_ << seq_ << ',' << tx_us << ',' << wall_ms << ',' << period_us << ',' << write_us << '\n';
  }

  seq_++;
  sent_++;
  return true;
}

void ProbeSender::report()
{
  const uint64_t delta = sent_ - window_first_seq_;
  const uint64_t dt_us = last_tx_us_ - window_start_us_;
  // delta 是窗口内的帧数，相邻帧间隔共 delta-1 个，频率按间隔数算
  const double actual_hz =
    (dt_us > 0 && delta > 1)
      ? static_cast<double>(delta - 1) * 1e6 / static_cast<double>(dt_us)
      : 0.0;
  const double write_avg_ms =
    window_write_count_ > 0
      ? static_cast<double>(window_write_sum_us_) / static_cast<double>(window_write_count_) /
          1000.0
      : 0.0;

  char line[320];
  std::snprintf(
    line, sizeof(line),
    "[STAT] t=%6.1fs  累计 %llu 帧（本周期 %llu 帧，实际 %.1f Hz / 目标 %.1f Hz）"
    "  最大周期 %.2f ms（目标 %.2f ms）  写串口 avg %.3f ms",
    static_cast<double>(last_tx_us_) / 1e6, static_cast<unsigned long long>(sent_),
    static_cast<unsigned long long>(delta), actual_hz, opt_.hz, window_max_period_us_ / 1000.0,
    1000.0 / opt_.hz, write_avg_ms);
  std::fputs(line, stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);

  // 重置窗口
  window_first_seq_ = sent_;
  window_start_us_ = last_tx_us_;
  window_max_period_us_ = 0;
  window_write_sum_us_ = 0;
  window_write_count_ = 0;
}

void ProbeSender::print_summary() const
{
  std::vector<uint32_t> periods = periods_us_;
  std::sort(periods.begin(), periods.end());

  const double elapsed_s =
    sent_ > 1 ? static_cast<double>(last_tx_us_ - first_tx_us_) / 1e6 : 0.0;
  const double actual_hz =
    elapsed_s > 0.0 ? static_cast<double>(sent_ - 1) / elapsed_s : 0.0;
  const double wire_ms =
    static_cast<double>(sizeof(DelayProbe)) * 10.0 * 1000.0 / static_cast<double>(opt_.baud);

  char line[512];
  std::fputs("\n================ 发送统计 ================\n", stdout);
  std::snprintf(
    line, sizeof(line), "发送 %llu 帧 / %.2f s，实际 %.1f Hz（目标 %.1f Hz）  boot_id=0x%08X",
    static_cast<unsigned long long>(sent_), elapsed_s, actual_hz, opt_.hz, boot_id_);
  std::fputs(line, stdout);
  std::fputc('\n', stdout);

  if (!periods.empty()) {
    double sum = 0.0;
    double jitter = 0.0;
    for (size_t i = 0; i < periods.size(); ++i) {
      sum += periods[i];
      if (i > 0) {
        jitter += std::fabs(static_cast<double>(periods[i]) - static_cast<double>(periods[i - 1]));
      }
    }
    const double avg_ms = sum / static_cast<double>(periods.size()) / 1000.0;
    double var = 0.0;
    for (uint32_t v : periods) {
      const double d = v / 1000.0 - avg_ms;
      var += d * d;
    }
    std::snprintf(
      line, sizeof(line),
      "发送周期 (ms)  min %.3f  avg %.3f  p50 %.3f  p95 %.3f  p99 %.3f  max %.3f  std %.3f  "
      "抖动 %.3f",
      periods.front() / 1000.0, avg_ms, percentile_ms(periods, 0.50), percentile_ms(periods, 0.95),
      percentile_ms(periods, 0.99), periods.back() / 1000.0,
      std::sqrt(var / static_cast<double>(periods.size())),
      (periods.size() > 1 ? jitter / static_cast<double>(periods.size() - 1) : 0.0) / 1000.0);
    std::fputs(line, stdout);
    std::fputc('\n', stdout);
  }

  std::snprintf(
    line, sizeof(line), "写串口 (ms)    min %.3f  avg %.3f  max %.3f",
    sent_ > 0 ? write_min_us_ / 1000.0 : 0.0,
    sent_ > 0 ? static_cast<double>(write_sum_us_) / static_cast<double>(sent_) / 1000.0 : 0.0,
    write_max_us_ / 1000.0);
  std::fputs(line, stdout);
  std::fputc('\n', stdout);

  std::snprintf(
    line, sizeof(line),
    "帧长 %zuB（7B 帧头 + %uB data + 2B CRC16），%u bps 下单帧线路时间约 %.3f ms",
    sizeof(DelayProbe), kPayloadSize, opt_.baud, wire_ms);
  std::fputs(line, stdout);
  std::fputc('\n', stdout);
  std::fputs("=========================================\n", stdout);
  std::fflush(stdout);
}

int ProbeSender::run()
{
  if (!opt_.csv_path.empty()) {
    csv_.open(opt_.csv_path, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      std::fprintf(stderr, "[WARN] 无法写入 CSV 文件: %s\n", opt_.csv_path.c_str());
    } else {
      csv_ << "seq,tx_us,unix_ms,period_us,write_us\n";
    }
  }

  const DelayProbe sample = make_probe(0, 0);
  char line[512];

  std::snprintf(
    line, sizeof(line), "[INFO] 串口=%s 波特率=%u  探针 0x%03X @ %.1f Hz  时长 %.1fs  CSV=%s",
    opt_.port.c_str(), opt_.baud, opt_.cmd, opt_.hz, opt_.duration_s,
    opt_.csv_path.empty() ? "(不写)" : opt_.csv_path.c_str());
  std::fputs(line, stdout);
  std::fputc('\n', stdout);

  std::snprintf(
    line, sizeof(line), "[INFO] 帧长 %zuB（7B 帧头 + %uB data + 2B CRC16）", sizeof(DelayProbe),
    kPayloadSize);
  std::fputs(line, stdout);
  std::fputc('\n', stdout);

  std::snprintf(
    line, sizeof(line),
    "[INFO] tx_us 基准（=0）对应墙钟 %s，boot_id=0x%08X（上位机重启后 tx_us 归零）",
    wall_time_now().c_str(), boot_id_);
  std::fputs(line, stdout);
  std::fputc('\n', stdout);

  const std::string hex = "[HEX] 探针样例（seq=0, tx_us=0）:" +
                          to_hex(reinterpret_cast<const uint8_t *>(&sample), sizeof(sample));
  std::fputs(hex.c_str(), stdout);
  std::fputc('\n', stdout);
  std::fputs("[INFO] 只发不收：延迟由下位机侧解算；Ctrl+C 结束\n", stdout);
  std::fflush(stdout);

  const auto period =
    std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / opt_.hz));
  auto next_tx = Clock::now();
  auto t_start = next_tx;
  auto last_report = next_tx;

  while (!g_quit) {
    if (!send_one()) break;

    const auto now = Clock::now();
    if (opt_.duration_s > 0.0 &&
        std::chrono::duration<double>(now - t_start).count() >= opt_.duration_s) {
      break;
    }
    if (std::chrono::duration<double>(now - last_report).count() >= 1.0) {
      last_report = now;
      report();
    }

    next_tx += period;
    std::this_thread::sleep_until(next_tx);
    // 系统卡顿错过多个周期时重新对齐，避免瞬间连发一串探针
    if (Clock::now() - next_tx > std::chrono::milliseconds(100)) next_tx = Clock::now();
  }

  if (csv_.is_open()) csv_.close();
  print_summary();
  return 0;
}

}  // namespace

int main()
{
  // 固定配置，直接运行即可：/dev/ttyACM0 921600 100Hz，跑 30s，逐帧写 delay.csv
  const Options opt;

  serial::Serial port;
  try {
    port.setPort(opt.port);
    port.setBaudrate(opt.baud);
    port.setBytesize(serial::eightbits);
    port.setParity(serial::parity_none);
    port.setStopbits(serial::stopbits_one);
    port.setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(20);
    port.setTimeout(timeout);
    port.open();
  } catch (const std::exception & e) {
    std::fprintf(
      stderr, "[ERR] 打开串口 %s @ %u 失败: %s\n", opt.port.c_str(), opt.baud, e.what());
    return 1;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  ProbeSender sender(port, opt);
  const int ret = sender.run();

  try {
    if (port.isOpen()) port.close();
  } catch (...) {
  }
  return ret;
}
