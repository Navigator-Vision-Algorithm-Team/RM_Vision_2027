// ============================================================================
//  time_delay_test — 视觉 <-> 下位机 串口通信延迟测试（回环 RTT）
//
//  发送帧与单向版完全一致（不改动），下位机把收到的 data 段原样回传即可（怎么
//  回传、用什么 cmd_id 由固件自己决定），上位机用「收到回传的时刻 - 本地发送
//  时刻」算往返时延 RTT，单向延迟 ≈ RTT / 2。
//
//  ---- 发送帧（29 字节，与之前逐字节一致）----
//   偏移 0    1      2        3      4      5      6      7 ... 26  27  28
//        A5 | len_lo len_hi | seq | crc8 | cmd_lo cmd_hi | data(20B) | crc16(2B)
//    · cmd_id = 0x0410
//    · len    = 20，只算 data 段（不含 7B header 与 2B crc16），整帧 29 字节
//    · crc8   = CRC8(前 4 字节)，位于偏移 4
//    · crc16  = CRC16(整帧 29 字节)，位于帧尾
//    · 多字节整数一律小端（x86_64 / aarch64 都是小端）
//
//  ---- data 段（20 字节，发送与回传统一）----
//   偏移 0   uint32 seq       发送序号，每帧 +1
//   偏移 4   uint64 tx_us     发送时刻：steady_clock 微秒计数，自本工具启动为 0，
//                             单调递增、不回绕
//   偏移 12  uint32 boot_id   本次启动的随机标识：值变化 = 上位机重启过
//   偏移 16  uint32 magic     固定 0x4C415445（ASCII "LATE"）
//
//  ---- 回传怎么认（不需要固定回传指令号）----
//    收到的帧只要同时满足下面三条，就按「本测试的回传」处理：
//      · header.data_length == 20
//      · data[16..19] == 0x4C415445（magic 没被改写）
//      · data[12..15] == 本次运行的 boot_id（排除上一轮运行残留的回包）
//    cmd_id 不限：原样回 0x0410 或固件自定义 0x0411 都能识别，日志里会打印实际值。
//
//  ---- 下位机侧最小实现（伪代码）----
//    收到 cmd_id == 0x0410 且 CRC16 正确后（时间戳打在「收完整帧」那一刻）：
//      reply = rx;                    // 整帧复制，data 段 20B 原样不动
//      reply.header.cmd_id = 0x0411;  // 或者不改，原样回 0x0410
//      Append_CRC8_Check_Sum(&reply, 5);
//      Append_CRC16_Check_Sum(&reply, 29);
//      send(reply);
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
//  ---- 用法 ----
//    直接运行，不需要任何参数（固定配置见 main 里的 Options 默认值）：
//        ./time_delay_test
//    等价于 /dev/ttyACM0 921600 100Hz 跑 30s，逐帧写 ./delay.csv。
//    Ctrl+C 提前结束；退出时打印统计。
//
//  ---- 结果怎么读 ----
//    · 921600 8N1 下 29 字节帧的纯线路时间 ≈ 0.32 ms，来回约 0.64 ms；USB 转串口
//      （ttyACM）另有约 1 ms 的 USB 帧周期，RTT 正常在 1~3 ms。
//    · 看 p95 / p99 / max 判断抖动；--csv（默认 delay.csv）可以出曲线画图。
// ============================================================================

#include <serial/serial.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
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

// data 段内偏移
constexpr size_t kOffSeq = 0;
constexpr size_t kOffTxUs = 4;
constexpr size_t kOffBootId = 12;
constexpr size_t kOffMagic = 16;

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

// 整行一次性输出，避免发送 / 接收两个线程的打印互相穿插
void emit(const std::string & line)
{
  std::fputs(line.c_str(), stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
}

// 读取最多 size 字节：收满或超时返回实际收到的字节数
size_t read_exact(serial::Serial & port, uint8_t * buffer, size_t size, int timeout_ms)
{
  size_t got = 0;
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  while (got < size) {
    size_t n = 0;
    try {
      n = port.read(buffer + got, size - got);
    } catch (const std::exception &) {
      return got;
    }
    got += n;
    if (got < size && Clock::now() >= deadline) break;
  }
  return got;
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
  double timeout_ms = 200.0;          // 超过该时间没回传就按丢包算
  uint16_t cmd = kDefaultCmd;
};

struct Sample
{
  uint32_t seq = 0;
  uint32_t rtt_us = 0;      // 权威往返时延：收到时刻 - 本地记录的发包时刻
  uint32_t pkt_rtt_us = 0;  // 用包里回传的 tx_us 算的往返时延，正常应与 rtt_us 相同
};

struct Summary
{
  size_t n = 0;
  double min_ms = 0.0;
  double avg_ms = 0.0;
  double p50_ms = 0.0;
  double p95_ms = 0.0;
  double p99_ms = 0.0;
  double max_ms = 0.0;
  double std_ms = 0.0;
  double jitter_ms = 0.0;  // 相邻样本 RTT 差的绝对值平均
  double oneway_ms = 0.0;  // 单向估计 RTT/2
};

double percentile_ms(const std::vector<uint32_t> & sorted_us, double p)
{
  if (sorted_us.empty()) return 0.0;
  const double idx = p * static_cast<double>(sorted_us.size() - 1);
  return sorted_us[static_cast<size_t>(idx + 0.5)] / 1000.0;
}

Summary summarize(const std::vector<Sample> & samples, size_t begin)
{
  Summary s;
  if (begin >= samples.size()) return s;

  s.n = samples.size() - begin;
  std::vector<uint32_t> rtt_us;
  rtt_us.reserve(s.n);

  double sum = 0.0;
  double jitter = 0.0;
  for (size_t i = begin; i < samples.size(); ++i) {
    rtt_us.push_back(samples[i].rtt_us);
    sum += samples[i].rtt_us;
    if (i > begin) {
      jitter += std::fabs(
        static_cast<double>(samples[i].rtt_us) - static_cast<double>(samples[i - 1].rtt_us));
    }
  }

  std::vector<uint32_t> sorted = rtt_us;
  std::sort(sorted.begin(), sorted.end());

  s.min_ms = sorted.front() / 1000.0;
  s.max_ms = sorted.back() / 1000.0;
  s.p50_ms = percentile_ms(sorted, 0.50);
  s.p95_ms = percentile_ms(sorted, 0.95);
  s.p99_ms = percentile_ms(sorted, 0.99);
  s.avg_ms = sum / static_cast<double>(s.n) / 1000.0;

  double var = 0.0;
  for (uint32_t v : rtt_us) {
    const double d = v / 1000.0 - s.avg_ms;
    var += d * d;
  }
  s.std_ms = std::sqrt(var / static_cast<double>(s.n));
  s.jitter_ms = (s.n > 1 ? jitter / static_cast<double>(s.n - 1) : 0.0) / 1000.0;
  s.oneway_ms = s.avg_ms / 2.0;
  return s;
}

// ---------------------------------------------------------------------------
// 测试主体
// ---------------------------------------------------------------------------
class RttTester
{
public:
  RttTester(serial::Serial & port, Options opt)
  : port_(port),
    opt_(std::move(opt)),
    timeout_us_(static_cast<uint64_t>(opt_.timeout_ms * 1000.0)),
    boot_id_(make_boot_id()),
    t0_us_(absolute_us())
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
  void receive_loop();
  void handle_echo(const uint8_t * data, uint16_t cmd_id, uint8_t header_seq);
  void evict_timeouts(uint64_t now);
  void report();
  void print_summary() const;
  void write_csv(const Sample & sample);

  serial::Serial & port_;
  const Options opt_;
  const uint64_t timeout_us_;
  const uint32_t boot_id_;
  const uint64_t t0_us_;

  std::thread rx_thread_;
  uint32_t seq_ = 0;

  // 已发出、等待回传的 seq -> 本地发送时刻（us）
  std::mutex pending_mutex_;
  std::unordered_map<uint32_t, uint64_t> pending_;

  // 统计（接收线程写，主线程读）
  mutable std::mutex stats_mutex_;
  std::vector<Sample> samples_;
  size_t window_begin_ = 0;

  std::atomic<uint64_t> sent_{0};
  std::atomic<uint64_t> echoed_{0};
  std::atomic<uint64_t> lost_{0};
  std::atomic<uint64_t> late_echo_{0};    // 判丢之后才回来 / 重复回包
  std::atomic<uint64_t> bad_crc8_{0};
  std::atomic<uint64_t> bad_crc16_{0};
  std::atomic<uint64_t> short_frames_{0};
  std::atomic<uint64_t> garbage_{0};      // 帧头之前的杂散字节
  std::atomic<uint64_t> other_frames_{0}; // 非本测试帧（0x0503 等）
  std::atomic<uint64_t> stale_frames_{0}; // boot_id 不是本次运行的残留回包
  std::atomic<uint64_t> bad_timestamp_{0};
  std::atomic<uint64_t> write_us_sum_{0};
  std::atomic<uint64_t> write_count_{0};
  std::atomic<uint32_t> write_max_us_{0};

  // 发送周期（us）
  std::vector<uint32_t> periods_us_;
  bool has_last_tx_ = false;
  uint64_t last_tx_us_ = 0;
  uint64_t first_tx_us_ = 0;

  std::mutex csv_mutex_;
  std::ofstream csv_;

  // 一次性提示，避免刷屏（只在接收线程里置位）
  bool timestamp_warned_ = false;
  // 只在主线程里置位
  bool no_echo_warned_ = false;
};

DelayProbe RttTester::make_probe(uint32_t seq, uint64_t tx_us) const
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

bool RttTester::send_one()
{
  const uint64_t tx_us = now_us();
  const DelayProbe pkt = make_probe(seq_, tx_us);

  {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    pending_[seq_] = tx_us;
  }

  const uint64_t w0 = absolute_us();
  try {
    port_.write(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[ERR] 下发失败: %s\n", e.what());
    std::lock_guard<std::mutex> lk(pending_mutex_);
    pending_.erase(seq_);
    return false;
  }
  const uint32_t write_us = static_cast<uint32_t>(absolute_us() - w0);
  write_us_sum_ += write_us;
  write_count_++;
  const uint32_t prev_max = write_max_us_.load();
  if (write_us > prev_max) write_max_us_.store(write_us);

  if (has_last_tx_) {
    const uint32_t period_us = static_cast<uint32_t>(tx_us - last_tx_us_);
    if (periods_us_.size() < 4'000'000) periods_us_.push_back(period_us);
  } else {
    first_tx_us_ = tx_us;
  }
  has_last_tx_ = true;
  last_tx_us_ = tx_us;

  seq_++;
  sent_++;
  return true;
}

void RttTester::receive_loop()
{
  std::vector<uint8_t> frame;

  while (!g_quit) {
    // ---- 1) 逐字节找帧头 0xA5 ----
    uint8_t first = 0;
    size_t got = 0;
    try {
      got = port_.read(&first, 1);
    } catch (const std::exception &) {
      break;
    }
    if (got == 0) {
      evict_timeouts(now_us());
      continue;
    }
    if (first != kSof) {
      garbage_++;
      continue;
    }

    // ---- 2) 收 Header 剩余 6 字节，校验 CRC8（前 4 字节 -> 第 5 字节）----
    uint8_t head[sizeof(io::Header)];
    head[0] = kSof;
    if (read_exact(port_, head + 1, sizeof(head) - 1, 50) != sizeof(head) - 1) {
      short_frames_++;
      continue;
    }
    if (!crc8::Verify_CRC8_Check_Sum(head, sizeof(head) - 2)) {
      bad_crc8_++;
      continue;
    }

    io::Header header{};
    std::memcpy(&header, head, sizeof(header));

    // ---- 3) 只收 data_length==20 的帧，其余（0x0503 等上报）整帧读掉丢弃 ----
    if (header.data_length != kPayloadSize) {
      if (header.data_length > 512) {
        continue;
      }
      uint8_t discard[514];
      if (read_exact(port_, discard, static_cast<size_t>(header.data_length) + 2, 50) !=
          static_cast<size_t>(header.data_length) + 2) {
        short_frames_++;
      } else {
        other_frames_++;
      }
      continue;
    }

    // ---- 4) 收 data + CRC16，校验通过才处理 ----
    frame.assign(head, head + sizeof(io::Header));
    frame.resize(sizeof(DelayProbe));
    if (read_exact(port_, frame.data() + sizeof(io::Header), kPayloadSize + 2, 50) !=
        kPayloadSize + 2) {
      short_frames_++;
      continue;
    }
    if (!crc16::Verify_CRC16_Check_Sum(frame.data(), static_cast<uint32_t>(frame.size()))) {
      bad_crc16_++;
      continue;
    }

    handle_echo(frame.data() + sizeof(io::Header), header.cmd_id, header.seq);
    evict_timeouts(now_us());
  }
}

void RttTester::handle_echo(const uint8_t * data, uint16_t cmd_id, uint8_t header_seq)
{
  uint32_t seq = 0;
  uint64_t tx_us = 0;
  uint32_t boot_id = 0;
  uint32_t magic = 0;
  std::memcpy(&seq, data + kOffSeq, sizeof(seq));
  std::memcpy(&tx_us, data + kOffTxUs, sizeof(tx_us));
  std::memcpy(&boot_id, data + kOffBootId, sizeof(boot_id));
  std::memcpy(&magic, data + kOffMagic, sizeof(magic));

  // magic 不对：不是本测试的回传帧（比如恰好 20 字节的其它上报帧）
  if (magic != kMagic) {
    other_frames_++;
    return;
  }
  // boot_id 不对：上一轮运行残留的回包，丢弃（本地待回包表里也不会有它）
  if (boot_id != boot_id_) {
    stale_frames_++;
    return;
  }

  const uint64_t t_now = now_us();
  const uint32_t pkt_rtt_us = static_cast<uint32_t>(t_now - tx_us);

  // 用本地记录的发包时刻配对：最可信，且不依赖下位机怎么处理 tx_us
  uint32_t rtt_us = pkt_rtt_us;
  bool matched = false;
  {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    auto it = pending_.find(seq);
    if (it != pending_.end()) {
      matched = true;
      rtt_us = static_cast<uint32_t>(t_now - it->second);
      pending_.erase(it);
    }
  }

  if (!matched) {
    late_echo_++;  // 已判丢包后才回来，或者同一 seq 重复回包
  } else if (pkt_rtt_us + 1000u < rtt_us || pkt_rtt_us > rtt_us + 1000u) {
    // 本地算出的 RTT 与包内时间戳算出的不一致（>1ms）：下位机改写过 tx_us
    if (!timestamp_warned_) {
      timestamp_warned_ = true;
      char warn[256];
      std::snprintf(
        warn, sizeof(warn),
        "[WARN] 包内 tx_us 与本地发送时刻不一致（本地 %.3f ms / 包内 %.3f ms）："
        "回传时 tx_us 应原样保留",
        rtt_us / 1000.0, pkt_rtt_us / 1000.0);
      emit(warn);
    }
  }

  if (rtt_us > 5'000'000u) {  // > 5 s 显然是时间戳错乱，不计入统计
    bad_timestamp_++;
    return;
  }

  const Sample sample{seq, rtt_us, pkt_rtt_us};
  {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    samples_.push_back(sample);
  }
  echoed_++;

  if (echoed_.load() == 1) {
    char info[256];
    std::snprintf(
      info, sizeof(info), "[INFO] 首个回传帧: cmd=0x%03X seq=%u header.seq=%u rtt=%.3f ms",
      cmd_id, seq, header_seq, rtt_us / 1000.0);
    emit(info);
  }

  write_csv(sample);
}

void RttTester::evict_timeouts(uint64_t now)
{
  std::lock_guard<std::mutex> lk(pending_mutex_);
  for (auto it = pending_.begin(); it != pending_.end();) {
    if (now - it->second < timeout_us_) {
      ++it;
      continue;
    }
    const uint32_t seq = it->first;
    it = pending_.erase(it);
    const uint64_t n = lost_.fetch_add(1) + 1;
    if (n <= 3) {
      char warn[192];
      std::snprintf(
        warn, sizeof(warn), "[WARN] seq=%u 超过 %.0f ms 没有回传，按丢包计数", seq,
        timeout_us_ / 1000.0);
      emit(warn);
    }
  }
}

void RttTester::write_csv(const Sample & sample)
{
  std::lock_guard<std::mutex> lk(csv_mutex_);
  if (!csv_.is_open()) return;
  csv_ << sample.seq << ',' << sample.rtt_us << ',' << sample.rtt_us / 2 << '\n';
}

void RttTester::report()
{
  Summary s;
  {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    s = summarize(samples_, window_begin_);
    window_begin_ = samples_.size();
  }

  const uint64_t sent = sent_.load();
  const uint64_t echoed = echoed_.load();
  const uint64_t lost = lost_.load();
  const double loss_pct =
    sent > 0 ? 100.0 * static_cast<double>(lost) / static_cast<double>(sent) : 0.0;

  char buf[512];
  if (s.n == 0) {
    std::snprintf(
      buf, sizeof(buf), "[STAT] 发 %llu 收 %llu 丢 %llu (%.1f%%)  |  本周期没有收到回传帧",
      static_cast<unsigned long long>(sent), static_cast<unsigned long long>(echoed),
      static_cast<unsigned long long>(lost), loss_pct);
  } else {
    std::snprintf(
      buf, sizeof(buf),
      "[STAT] 发 %llu 收 %llu 丢 %llu (%.1f%%)  |  RTT min %.2f avg %.2f p95 %.2f max %.2f ms"
      "  抖动 %.2f ms  单向≈%.2f ms  (n=%zu)",
      static_cast<unsigned long long>(sent), static_cast<unsigned long long>(echoed),
      static_cast<unsigned long long>(lost), loss_pct, s.min_ms, s.avg_ms, s.p95_ms, s.max_ms,
      s.jitter_ms, s.oneway_ms, s.n);
  }
  emit(buf);

  if (echoed == 0 && !no_echo_warned_) {
    no_echo_warned_ = true;
    emit(
      "[WARN] 还没有收到任何回传帧：确认下位机把收到的 data 段原样回传（data_length=20、"
      "magic 不变、boot_id 原样带回），链路本身可先用 serial_communication_test 确认");
  }
}

void RttTester::print_summary() const
{
  std::vector<Sample> all;
  {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    all = samples_;
  }
  const Summary s = summarize(all, 0);

  const uint64_t sent = sent_.load();
  const uint64_t echoed = echoed_.load();
  const uint64_t lost = lost_.load();
  const uint64_t in_flight = (sent > echoed + lost) ? sent - echoed - lost : 0;
  const double loss_pct =
    sent > 0 ? 100.0 * static_cast<double>(lost) / static_cast<double>(sent) : 0.0;
  const double elapsed_s =
    sent > 1 ? static_cast<double>(last_tx_us_ - first_tx_us_) / 1e6 : 0.0;
  const double actual_hz = elapsed_s > 0.0 ? static_cast<double>(sent - 1) / elapsed_s : 0.0;
  const double wire_ms =
    static_cast<double>(sizeof(DelayProbe)) * 10.0 * 1000.0 / static_cast<double>(opt_.baud);

  char line[512];
  emit("");
  emit("================ 延迟测试结果 ================");
  std::snprintf(
    line, sizeof(line), "发送 %llu 帧 / %.2f s，实际 %.1f Hz（目标 %.1f Hz）  boot_id=0x%08X",
    static_cast<unsigned long long>(sent), elapsed_s, actual_hz, opt_.hz, boot_id_);
  emit(line);
  std::snprintf(
    line, sizeof(line), "收到回传 %llu 帧 / 超时丢失 %llu 帧 (%.2f%%) / 在途 %llu",
    static_cast<unsigned long long>(echoed), static_cast<unsigned long long>(lost), loss_pct,
    static_cast<unsigned long long>(in_flight));
  emit(line);

  if (s.n > 0) {
    std::snprintf(
      line, sizeof(line),
      "RTT (ms)   min %.3f  avg %.3f  p50 %.3f  p95 %.3f  p99 %.3f  max %.3f  std %.3f  抖动 %.3f",
      s.min_ms, s.avg_ms, s.p50_ms, s.p95_ms, s.p99_ms, s.max_ms, s.std_ms, s.jitter_ms);
    emit(line);
    std::snprintf(
      line, sizeof(line), "单向延迟估计 %.3f ms（RTT/2）", s.oneway_ms);
    emit(line);
  } else {
    emit("未收到有效回传，无法计算 RTT");
  }

  if (!periods_us_.empty()) {
    std::vector<uint32_t> periods = periods_us_;
    std::sort(periods.begin(), periods.end());
    double sum = 0.0;
    for (uint32_t v : periods) sum += v;
    std::snprintf(
      line, sizeof(line), "发送周期 (ms)  min %.3f  avg %.3f  p95 %.3f  max %.3f",
      periods.front() / 1000.0, sum / static_cast<double>(periods.size()) / 1000.0,
      percentile_ms(periods, 0.95), periods.back() / 1000.0);
    emit(line);
  }

  const uint64_t writes = write_count_.load();
  std::snprintf(
    line, sizeof(line), "写串口 (ms)    avg %.3f  max %.3f",
    writes > 0 ? static_cast<double>(write_us_sum_.load()) / static_cast<double>(writes) / 1000.0
               : 0.0,
    write_max_us_.load() / 1000.0);
  emit(line);

  std::snprintf(
    line, sizeof(line),
    "帧长 %zuB（7B 帧头 + %uB data + 2B CRC16），%u bps 下单程线路时间约 %.3f ms",
    sizeof(DelayProbe), kPayloadSize, opt_.baud, wire_ms);
  emit(line);

  std::snprintf(
    line, sizeof(line),
    "异常: crc8=%llu crc16=%llu 短帧=%llu 垃圾字节=%llu 其它帧=%llu 过期回包=%llu "
    "迟到回包=%llu 时间戳异常=%llu",
    static_cast<unsigned long long>(bad_crc8_.load()),
    static_cast<unsigned long long>(bad_crc16_.load()),
    static_cast<unsigned long long>(short_frames_.load()),
    static_cast<unsigned long long>(garbage_.load()),
    static_cast<unsigned long long>(other_frames_.load()),
    static_cast<unsigned long long>(stale_frames_.load()),
    static_cast<unsigned long long>(late_echo_.load()),
    static_cast<unsigned long long>(bad_timestamp_.load()));
  emit(line);
  emit("=============================================");
}

int RttTester::run()
{
  if (!opt_.csv_path.empty()) {
    csv_.open(opt_.csv_path, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      std::fprintf(stderr, "[WARN] 无法写入 CSV 文件: %s\n", opt_.csv_path.c_str());
    } else {
      csv_ << "seq,rtt_us,oneway_us\n";
    }
  }

  const DelayProbe sample = make_probe(0, 0);
  char line[512];
  std::snprintf(
    line, sizeof(line), "[INFO] 串口=%s 波特率=%u  探针 0x%03X @ %.1f Hz  时长 %.1fs  CSV=%s",
    opt_.port.c_str(), opt_.baud, opt_.cmd, opt_.hz, opt_.duration_s,
    opt_.csv_path.empty() ? "(不写)" : opt_.csv_path.c_str());
  emit(line);

  std::snprintf(
    line, sizeof(line), "[INFO] 帧长 %zuB（7B 帧头 + %uB data + 2B CRC16），回传不限 cmd_id",
    sizeof(DelayProbe), kPayloadSize);
  emit(line);

  std::snprintf(
    line, sizeof(line),
    "[INFO] tx_us 基准（=0）对应墙钟 %s，boot_id=0x%08X（上位机重启后 tx_us 归零）",
    wall_time_now().c_str(), boot_id_);
  emit(line);

  emit(
    "[HEX] 探针样例（seq=0, tx_us=0）:" +
    to_hex(reinterpret_cast<const uint8_t *>(&sample), sizeof(sample)));
  emit("[INFO] 下位机把 data 段原样回传即可；Ctrl+C 结束");
  std::fflush(stdout);

  rx_thread_ = std::thread(&RttTester::receive_loop, this);

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

  g_quit = true;
  if (rx_thread_.joinable()) rx_thread_.join();

  report();  // 最后不足 1s 的残余样本
  print_summary();

  {
    std::lock_guard<std::mutex> lk(csv_mutex_);
    if (csv_.is_open()) csv_.close();
  }
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

  RttTester tester(port, opt);
  const int ret = tester.run();

  try {
    if (port.isOpen()) port.close();
  } catch (...) {
  }
  return ret;
}
