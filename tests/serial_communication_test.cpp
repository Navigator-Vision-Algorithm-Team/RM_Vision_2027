// ============================================================================
//  serial_communication_test — 视觉 <-> 下位机串口通信自检
//
//  1) 读取：持续解析下位机上报帧并把字段 print 出来
//       0x502 IMU(双yaw) / 0x503 底盘速度 / 0x504 按键 / 0x505 机器人状态 /
//       0x0001·0x0100 比赛状态 / 0x0201·0x0102 机器人状态
//     每帧都过 CRC8(头) + CRC16(整帧)，校验失败的帧只计数不打印；
//     未知 cmd_id 按 header.data_length 收完整帧后打印原始 hex，便于发现固件新增指令。
//  2) 下发：固定频率发 0x0402 自瞄指令，yaw 是世界系绝对角（弧度，wrap 到 (-pi, pi]），
//     按恒定角速度缓慢增大，云台因此慢慢旋转；pitch=0、shoot=0、accuracy=50，
//     id / robo_id / timeseries 均为 0。
//     加 --nav 时同时发 0x0405 导航指令（vx=vy=wz=0，不产生移动）。
//
//  用法:
//    ./serial_communication_test [串口] [波特率] [yaw角速度rad/s] [发送频率Hz] [--nav]
//    ./serial_communication_test                             # /dev/ttyACM0 921600 0.2 100
//    ./serial_communication_test /dev/gimbal                 # 本项目默认串口
//    ./serial_communication_test /dev/ttyUSB0 115200 0.1 50 --nav
//    Ctrl+C 退出
// ============================================================================

#include <serial/serial.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "io/serial/gimbal/packet.hpp"
#include "io/serial/gimbal/protocol_crc.hpp"

namespace
{

std::atomic<bool> g_quit{false};

void on_signal(int /*sig*/) { g_quit = true; }

constexpr double k_pi = 3.14159265358979323846;
constexpr double k_rad2deg = 180.0 / k_pi;

// 与 tools::limit_rad 一致：wrap 到 (-pi, pi]，与下发的世界系绝对角约定相同
double limit_rad(double angle)
{
  while (angle > k_pi) angle -= 2 * k_pi;
  while (angle <= -k_pi) angle += 2 * k_pi;
  return angle;
}

struct Options
{
  std::string port = "/dev/ttyACM0";
  uint32_t baud = 921600;
  double yaw_rate = 0.2;   // rad/s，云台目标角的增长角速度（0.2 ≈ 11°/s）
  double hz = 100.0;       // 0x0402 下发频率
  double print_hz = 10.0;  // 同类帧的最大打印频率（避免 0x502 500Hz 刷屏）
  bool send_nav = false;   // 是否同时下发 0x0405（全零速度）
};

struct Stats
{
  std::atomic<uint64_t> rx_frames{0};
  std::atomic<uint64_t> rx_bytes{0};
  std::atomic<uint64_t> tx_frames{0};
  std::atomic<uint64_t> garbage_bytes{0};
  std::atomic<uint64_t> bad_crc8{0};
  std::atomic<uint64_t> bad_crc16{0};
  std::atomic<uint64_t> short_frames{0};
  std::atomic<uint64_t> bad_length{0};
};

// 已知上报帧：cmd_id -> 全帧字节数（含 Header 与 CRC16），与下面解析用的结构体一致
struct KnownCommand
{
  uint16_t cmd_id;
  size_t frame_size;
  const char * name;
};

const KnownCommand k_known_commands[] = {
  {0x0502, sizeof(io::IMUPacket), "IMU"},
  {0x0503, sizeof(io::classisPacket), "CHASSIS"},
  {0x0504, sizeof(io::bottonPacket), "BUTTON"},
  {0x0505, sizeof(io::bassPacket), "BASE"},
  {0x0001, sizeof(io::GameStatusPacket), "GAME"},
  {0x0100, sizeof(io::GameStatusPacket), "GAME"},
  {0x0201, sizeof(io::GameRobotStatusPacket), "ROBOT"},
  {0x0102, sizeof(io::GameRobotStatusPacket), "ROBOT"},
};

const KnownCommand * find_known(uint16_t cmd_id)
{
  for (const auto & k : k_known_commands)
    if (k.cmd_id == cmd_id) return &k;
  return nullptr;
}

// 整行一次性输出，避免 RX/TX 两个线程的打印互相穿插
void emit(const std::string & line)
{
  std::fputs(line.c_str(), stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
}

std::string to_hex(const std::vector<uint8_t> & frame)
{
  static const char * digits = "0123456789ABCDEF";
  std::string out;
  out.reserve(frame.size() * 3);
  for (uint8_t b : frame) {
    out.push_back(' ');
    out.push_back(digits[b >> 4]);
    out.push_back(digits[b & 0x0F]);
  }
  return out;
}

// 读取最多 size 字节：等到收满或超时（串口本身已设 20ms 超时，允许部分读取）
size_t read_exact(serial::Serial & port, uint8_t * buffer, size_t size, int timeout_ms)
{
  size_t got = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (got < size) {
    size_t n = 0;
    try {
      n = port.read(buffer + got, size - got);
    } catch (const std::exception &) {
      return got;
    }
    got += n;
    if (got < size && std::chrono::steady_clock::now() >= deadline) break;
  }
  return got;
}

// 把一帧解析成对应结构体并打印字段
void print_frame(const std::vector<uint8_t> & frame, const io::Header & header)
{
  char buf[512];
  const size_t n = frame.size();

  switch (header.cmd_id) {
    case 0x0502: {
      if (n < sizeof(io::IMUPacket)) break;
      io::IMUPacket p{};
      std::memcpy(&p, frame.data(), sizeof(p));
      std::snprintf(
        buf, sizeof(buf),
        "[RX] 0x502 IMU      pitch=%+.4f roll=%+.4f upper_yaw=%+.4f lower_yaw=%+.4f rad "
        "(yaw %+.1f°/ %+.1f°)  roboid=%u  seq=%u len=%u",
        p.pitch, p.roll, p.upper_yaw, p.lower_yaw, p.upper_yaw * k_rad2deg,
        p.lower_yaw * k_rad2deg, p.roboid, header.seq, header.data_length);
      emit(buf);
      return;
    }
    case 0x0503: {
      if (n < sizeof(io::classisPacket)) break;
      io::classisPacket p{};
      std::memcpy(&p, frame.data(), sizeof(p));
      std::snprintf(
        buf, sizeof(buf), "[RX] 0x503 CHASSIS  vx=%+.3f vy=%+.3f vz=%+.3f enable=%u  seq=%u len=%u",
        p.vx, p.vy, p.vz, p.enable, header.seq, header.data_length);
      emit(buf);
      return;
    }
    case 0x0504: {
      if (n < sizeof(io::bottonPacket)) break;
      io::bottonPacket p{};
      std::memcpy(&p, frame.data(), sizeof(p));
      std::snprintf(buf, sizeof(buf), "[RX] 0x504 BUTTON   start=%d  seq=%u len=%u",
                    static_cast<int>(p.start), header.seq, header.data_length);
      emit(buf);
      return;
    }
    case 0x0505: {
      if (n < sizeof(io::bassPacket)) break;
      io::bassPacket p{};
      std::memcpy(&p, frame.data(), sizeof(p));
      std::snprintf(
        buf, sizeof(buf),
        "[RX] 0x505 BASE     roboid=%u reset=%d pos=(%u,%u) HP=%u mode=%u canshoot=%d "
        "shootmode=%u super=%d online=%d two=%d  seq=%u len=%u",
        p.roboid, static_cast<int>(p.reset), p.position[0], p.position[1], p.HP, p.robomode,
        static_cast<int>(p.canshoot), p.shootmode, static_cast<int>(p.superpower),
        static_cast<int>(p.online), static_cast<int>(p.two), header.seq, header.data_length);
      emit(buf);
      return;
    }
    case 0x0001:
    case 0x0100: {
      if (n < sizeof(io::GameStatusPacket)) break;
      io::GameStatusPacket p{};
      std::memcpy(&p, frame.data(), sizeof(p));
      std::snprintf(
        buf, sizeof(buf),
        "[RX] 0x%03X GAME     type=%u progress=%u remain=%us sync=%llu  seq=%u len=%u",
        header.cmd_id, p.game_info & 0x0F, (p.game_info >> 4) & 0x0F, p.stage_remain_time,
        static_cast<unsigned long long>(p.sync_timestamp), header.seq, header.data_length);
      emit(buf);
      return;
    }
    case 0x0201:
    case 0x0102: {
      if (n < sizeof(io::GameRobotStatusPacket)) break;
      io::GameRobotStatusPacket p{};
      std::memcpy(&p, frame.data(), sizeof(p));
      std::snprintf(
        buf, sizeof(buf),
        "[RX] 0x%03X ROBOT    id=%u lv=%u HP=%u/%u cool=%u heat_limit=%u power_limit=%u "
        "mains(g%d c%d s%d)  seq=%u len=%u",
        header.cmd_id, p.robot_id, p.robot_level, p.remain_hp, p.max_hp, p.shooter_cooling_rate,
        p.shooter_heat_limit, p.chassis_power_limit, p.mains_power_state & 0x01,
        (p.mains_power_state >> 1) & 0x01, (p.mains_power_state >> 2) & 0x01, header.seq,
        header.data_length);
      emit(buf);
      return;
    }
    default:
      break;
  }

  // 未知 / 长度不符：原样打印 hex
  std::snprintf(buf, sizeof(buf), "[RX] 0x%03X UNKNOWN  seq=%u data_length=%u bytes=%zu",
                header.cmd_id, header.seq, header.data_length, n);
  emit(std::string(buf) + "  raw:" + to_hex(frame));
}

void receive_loop(serial::Serial & port, const Options & opt, Stats & stats)
{
  std::unordered_map<uint16_t, std::chrono::steady_clock::time_point> last_print;
  std::unordered_map<uint16_t, uint64_t> per_cmd;
  std::unordered_map<uint16_t, bool> first_seen;
  std::unordered_map<uint16_t, bool> length_warned;
  std::vector<uint8_t> frame;

  auto last_stat = std::chrono::steady_clock::now();

  while (!g_quit) {
    // ---- 1) 逐字节找帧头 0xA5 ----
    uint8_t first = 0;
    if (read_exact(port, &first, 1, 100) != 1) continue;
    if (first != 0xA5) {
      stats.garbage_bytes++;
      continue;
    }

    // ---- 2) 收 Header 剩余 6 字节，校验 CRC8（前 4 字节 -> 第 5 字节）----
    frame.assign(sizeof(io::Header), 0);
    frame[0] = first;
    const size_t head_rest = sizeof(io::Header) - 1;
    if (read_exact(port, frame.data() + 1, head_rest, 50) != head_rest) {
      stats.short_frames++;
      continue;
    }
    if (!crc8::Verify_CRC8_Check_Sum(frame.data(), sizeof(io::Header) - 2)) {
      stats.bad_crc8++;
      continue;
    }

    io::Header header{};
    std::memcpy(&header, frame.data(), sizeof(header));

    // ---- 3) 确定整帧长度：已知 cmd 用固件结构体长度，未知 cmd 用 data_length ----
    const KnownCommand * known = find_known(header.cmd_id);
    size_t frame_size = 0;
    if (known) {
      frame_size = known->frame_size;
      const size_t declared = sizeof(io::Header) + header.data_length + 2;
      if (declared != frame_size && !length_warned[header.cmd_id]) {
        length_warned[header.cmd_id] = true;
        char warn[256];
        std::snprintf(
          warn, sizeof(warn),
          "[WARN] cmd 0x%03X (%s) data_length=%u 与结构体长度 %zu 不一致，"
          "已按结构体长度解析，请与固件核对",
          header.cmd_id, known->name, header.data_length, frame_size - sizeof(io::Header) - 2);
        emit(warn);
      }
    } else {
      frame_size = sizeof(io::Header) + header.data_length + 2;
      if (header.data_length > 256) {
        stats.bad_length++;
        continue;
      }
    }

    // ---- 4) 收 body（payload + CRC16）并校验 ----
    frame.resize(frame_size);
    const size_t body = frame_size - sizeof(io::Header);
    if (read_exact(port, frame.data() + sizeof(io::Header), body, 50) != body) {
      stats.short_frames++;
      continue;
    }
    if (!crc16::Verify_CRC16_Check_Sum(frame.data(), static_cast<uint32_t>(frame_size))) {
      stats.bad_crc16++;
      continue;
    }

    stats.rx_frames++;
    stats.rx_bytes += frame_size;
    per_cmd[header.cmd_id]++;

    // ---- 5) 打印：同一 cmd 限频，另对每类帧首次出现时打一份完整 hex ----
    const auto now = std::chrono::steady_clock::now();
    auto & last = last_print[header.cmd_id];
    if (std::chrono::duration<double>(now - last).count() >= 1.0 / opt.print_hz) {
      last = now;
      print_frame(frame, header);
    }
    if (!first_seen[header.cmd_id]) {
      first_seen[header.cmd_id] = true;
      char tag[64];
      std::snprintf(tag, sizeof(tag), "[HEX] 0x%03X 首帧原始字节:", header.cmd_id);
      emit(tag + to_hex(frame));
    }

    // ---- 6) 每秒一次统计，方便判断"收不到 / 收错 / 收得对" ----
    if (std::chrono::duration<double>(now - last_stat).count() >= 1.0) {
      last_stat = now;
      std::string line = "[STAT] rx=" + std::to_string(stats.rx_frames.load()) +
                         " frames (" + std::to_string(stats.rx_bytes.load()) + " B)  tx=" +
                         std::to_string(stats.tx_frames.load()) + "  err: crc8=" +
                         std::to_string(stats.bad_crc8.load()) + " crc16=" +
                         std::to_string(stats.bad_crc16.load()) + " short=" +
                         std::to_string(stats.short_frames.load()) + " garbage=" +
                         std::to_string(stats.garbage_bytes.load()) + "  |  ";
      for (const auto & kv : per_cmd)
        line += "0x" + std::to_string(kv.first) + ":" + std::to_string(kv.second) + " ";
      if (stats.rx_frames.load() == 0)
        line += "| 未收到任何合法帧：检查接线/TX-RX 交叉/波特率/下位机是否上电";
      emit(line);
    }
  }
}

void append_header(io::Header & header, uint16_t cmd_id, uint16_t data_length, uint8_t seq)
{
  header.header = 0xA5;
  header.data_length = data_length;
  header.seq = seq;
  header.cmd_id = cmd_id;
  crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&header), sizeof(io::Header) - 2);
}

// 0x0402 自瞄指令：只让 yaw 缓慢变化，其余保持默认值
void send_aim(serial::Serial & port, double yaw, uint8_t seq)
{
  io::SendPacket packet{};
  append_header(
    packet.header, 0x0402, sizeof(io::SendPacket) - sizeof(io::Header) - 2, seq);

  packet.id = 0;
  packet.robo_id = 0;
  packet.pitch = 0.0f;
  packet.yaw = static_cast<float>(yaw);
  packet.accuracy = 50;
  packet.shoot = 0;
  packet.timeseries = 0;

  crc16::Append_CRC16_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet), sizeof(io::SendPacket));
  port.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(io::SendPacket));
}

// 0x0405 导航指令：全零速度（不移动，仅验证链路）
void send_nav(serial::Serial & port, uint8_t seq)
{
  io::NavigationPacket packet{};
  append_header(
    packet.header, 0x0405, sizeof(io::NavigationPacket) - sizeof(io::Header) - 2, seq);

  packet.vx = 0.0f;
  packet.vy = 0.0f;
  packet.wz = 0.0f;

  crc16::Append_CRC16_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet), sizeof(io::NavigationPacket));
  port.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(io::NavigationPacket));
}

void print_usage(const char * argv0)
{
  std::printf(
    "用法: %s [串口] [波特率] [yaw角速度 rad/s] [发送频率 Hz] [--nav]\n"
    "示例: %s                              # /dev/ttyACM0 921600 0.2 100\n"
    "      %s /dev/gimbal                  # 本项目默认串口\n"
    "      %s /dev/ttyUSB0 115200 0.1 50 --nav\n"
    "说明: yaw 为世界系绝对角，按给定角速度缓慢增大 -> 云台慢慢旋转；\n"
    "      pitch/shoot/accuracy 等保持默认值；--nav 额外下发 0x0405 全零速度。\n",
    argv0, argv0, argv0, argv0);
}

bool parse_args(int argc, char * argv[], Options & opt)
{
  int positional = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") return false;
    if (arg == "--nav" || arg == "-n") {
      opt.send_nav = true;
      continue;
    }
    try {
      switch (positional++) {
        case 0:
          opt.port = arg;
          break;
        case 1:
          opt.baud = static_cast<uint32_t>(std::stoul(arg));
          break;
        case 2:
          opt.yaw_rate = std::stod(arg);
          break;
        case 3:
          opt.hz = std::stod(arg);
          break;
        default:
          std::fprintf(stderr, "多余的参数: %s\n", arg.c_str());
          return false;
      }
    } catch (const std::exception &) {
      std::fprintf(stderr, "参数无法解析: %s\n", arg.c_str());
      return false;
    }
  }

  if (opt.hz <= 0.0 || opt.print_hz <= 0.0) {
    std::fprintf(stderr, "发送频率必须为正\n");
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char * argv[])
{
  Options opt;
  if (!parse_args(argc, argv, opt)) {
    print_usage(argv[0]);
    return 1;
  }

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

  std::printf(
    "[INFO] 串口=%s 波特率=%u  下发: 0x0402 @ %.1fHz, yaw 以 %.3f rad/s (%.1f°/s) 缓慢增大%s\n",
    opt.port.c_str(), opt.baud, opt.hz, opt.yaw_rate, opt.yaw_rate * k_rad2deg,
    opt.send_nav ? ", 以及 0x0405 (全零速度)" : "");
  std::printf(
    "[INFO] 0x0402 载荷: pitch=0 shoot=0 accuracy=50 id=0 robo_id=0 timeseries=0 ; "
    "Ctrl+C 退出\n");
  std::fflush(stdout);

  Stats stats;
  std::thread rx_thread(receive_loop, std::ref(port), std::cref(opt), std::ref(stats));

  uint8_t seq = 0;
  double yaw = 0.0;
  auto t_prev = std::chrono::steady_clock::now();
  auto next_tx = t_prev;
  auto last_tx_print = t_prev;
  const auto period = std::chrono::duration<double>(1.0 / opt.hz);

  while (!g_quit) {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - t_prev).count();
    t_prev = now;

    // 世界系绝对角 + 恒定角速度；wrap 到 (-pi, pi] 后仍是同一个物理角度，旋转连续
    yaw = limit_rad(yaw + opt.yaw_rate * dt);

    try {
      send_aim(port, yaw, seq);
      if (opt.send_nav) send_nav(port, seq);
    } catch (const std::exception & e) {
      std::fprintf(stderr, "[ERR] 下发失败: %s\n", e.what());
      break;
    }
    seq++;
    stats.tx_frames++;

    if (std::chrono::duration<double>(now - last_tx_print).count() >= 1.0 / opt.print_hz) {
      last_tx_print = now;
      char buf[256];
      std::snprintf(
        buf, sizeof(buf),
        "[TX] 0x0402 AIM      yaw=%+.4f rad (%+.1f°) pitch=%+.4f accuracy=50 shoot=0 seq=%u",
        yaw, yaw * k_rad2deg, 0.0, seq);
      emit(buf);
    }

    next_tx += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next_tx);
    // 系统卡顿导致错过多个周期时重新对齐，避免瞬间连发一串旧指令
    if (std::chrono::steady_clock::now() - next_tx > std::chrono::milliseconds(100))
      next_tx = std::chrono::steady_clock::now();
  }

  g_quit = true;
  if (rx_thread.joinable()) rx_thread.join();
  try {
    if (port.isOpen()) port.close();
  } catch (...) {
  }

  std::printf(
    "[INFO] 退出。累计: 收 %llu 帧 / %llu B, 发 %llu 帧; 错误: crc8=%llu crc16=%llu "
    "short=%llu garbage=%llu\n",
    static_cast<unsigned long long>(stats.rx_frames.load()),
    static_cast<unsigned long long>(stats.rx_bytes.load()),
    static_cast<unsigned long long>(stats.tx_frames.load()),
    static_cast<unsigned long long>(stats.bad_crc8.load()),
    static_cast<unsigned long long>(stats.bad_crc16.load()),
    static_cast<unsigned long long>(stats.short_frames.load()),
    static_cast<unsigned long long>(stats.garbage_bytes.load()));
  return 0;
}
