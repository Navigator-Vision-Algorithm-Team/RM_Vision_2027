// ============================================================================
//  serial_communication_test — 视觉 <-> 下位机串口通信自检
//
//  1) 读取：用与业务完全相同的解包器（io/serial/protocol/）解析下位机上报帧
//       host 上行 0x0500 协议版本 / 0x0501 机器人状态 / 0x0502 底盘 / 0x0503 云台 /
//                 0x0504 发射 / 0x0505 日志
//       裁判转发 0x0001 比赛状态 / 0x0003 血量 / 0x0201 机器人性能 / 0x0202 热量 /
//                 0x0207 实时射击 / 0x0208 允许发弹量 / 0x020D 哨兵信息
//     分帧、CRC8、CRC16 全部由 referee_comm::Receiver 负责，未知 cmd_id 打印原始 data。
//  2) 下发：固定频率发 0x0402 自瞄指令，yaw 是世界系绝对角（弧度，wrap 到 (-pi, pi]），
//     按恒定角速度缓慢增大，云台因此慢慢旋转；pitch=0、shoot=0，data 段只发 pitch/yaw/shoot。
//     加 --nav 时同时发 0x0405 导航指令（vx=vy=wz=0、target_angle=0，不产生移动）。
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
#include "io/serial/protocol/host_protocol.hpp"
#include "io/serial/protocol/referee.hpp"
#include "io/serial/protocol/seasons/2027/protocol.hpp"

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
  double yaw_rate = 0.2;  // rad/s，云台目标角的增长角速度（0.2 ≈ 11°/s）
  double hz = 100.0;      // 0x0402 下发频率
  double print_hz = 10.0; // 同类帧的最大打印频率（避免 0x0503 500Hz 刷屏）
  bool send_nav = false;  // 是否同时下发 0x0405（全零速度）
};

struct Stats
{
  std::atomic<uint64_t> rx_frames{0};
  std::atomic<uint64_t> rx_bytes{0};
  std::atomic<uint64_t> tx_frames{0};
};

// 整行一次性输出，避免 RX/TX 两个线程的打印互相穿插
void emit(const std::string & line)
{
  std::fputs(line.c_str(), stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
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

// 把解出来的载荷打成一行可读文本；同一 cmd_id 限频
class ConsoleDecoder final : public referee_comm::Protocol
{
public:
  explicit ConsoleDecoder(double print_hz) : print_hz_(print_hz) {}

  bool OnCommand(const referee_comm::Frame & frame) override
  {
    const uint16_t segment = static_cast<uint16_t>(frame.cmd_id & 0xFF00);
    const bool handled =
      (segment == 0x0500) ? host.OnCommand(frame) : referee.OnCommand(frame);

    per_cmd_[frame.cmd_id]++;
    const auto now = std::chrono::steady_clock::now();
    auto & last = last_print_[frame.cmd_id];
    const bool due = std::chrono::duration<double>(now - last).count() >= 1.0 / print_hz_;

    if (due) {
      last = now;
      if (handled)
        print_known(frame.cmd_id);
      else
        print_unknown(frame);
    }
    if (!first_seen_[frame.cmd_id]) {
      first_seen_[frame.cmd_id] = true;
      char tag[64];
      std::snprintf(tag, sizeof(tag), "[DATA] 0x%03X 首帧 data 段:", frame.cmd_id);
      emit(tag + to_hex(frame.data, frame.length));
    }
    return handled;
  }

  std::string per_cmd_line() const
  {
    std::string line;
    for (const auto & kv : per_cmd_) {
      char item[32];
      std::snprintf(item, sizeof(item), "0x%03X:%llu ", kv.first,
                    static_cast<unsigned long long>(kv.second));
      line += item;
    }
    return line;
  }

  host_comm::Protocol host;
  referee_comm::v2027::Protocol referee;

private:
  void print_known(uint16_t cmd_id)
  {
    char buf[512];
    using namespace referee_comm::v2027;

    switch (cmd_id) {
      case host_comm::HOST_VERSION: {
        const auto & p = host.version;
        std::snprintf(
          buf, sizeof(buf),
          "[RX] 0x500 VERSION  host_v=%u season=%u robot_type=%u chassis_type=%u "
          "gimbal_type=%u uptime=%ums",
          p.host_version, p.referee_season, p.robot_type, p.chassis_type, p.gimbal_type,
          p.uptime_ms);
        break;
      }
      case host_comm::ROBOT_STATUS: {
        const auto & p = host.robot;
        std::snprintf(
          buf, sizeof(buf),
          "[RX] 0x501 ROBOT    enable=0x%02X ctrl=%u mode=%u fric=%u shoot=%u modules=0x%02X",
          p.enable, p.controller_online, p.robot_mode, p.fric_mode, p.shoot_mode,
          p.module_online);
        break;
      }
      case host_comm::CHASSIS_STATUS: {
        const auto & p = host.chassis;
        std::snprintf(
          buf, sizeof(buf),
          "[RX] 0x502 CHASSIS  enabled=%u vx=%+.3f vy=%+.3f wz=%+.3f heading=%+.4f rad",
          p.chassis_enabled, p.speed_x, p.speed_y, p.speed_turn, p.feedback_angle);
        break;
      }
      case host_comm::GIMBAL_STATUS: {
        const auto & p = host.gimbal;
        // 双yaw：yaw_angle 是 IMU(相机) 世界角，小yaw 关节角由 upper_yaw_angle 给出
        const double lower = limit_rad(p.yaw_angle - p.upper_yaw_angle);
        std::snprintf(
          buf, sizeof(buf),
          "[RX] 0x503 GIMBAL   pitch=%+.4f yaw=%+.4f roll=%+.4f (%.1f°) upper_joint=%+.4f "
          "-> lower=%+.4f (%.1f°) enable=%u autoaim=%u",
          p.pitch_angle, p.yaw_angle, p.roll_angle, p.yaw_angle * k_rad2deg,
          p.upper_yaw_angle, lower, lower * k_rad2deg, p.gimbal_enable, p.autoaim_enabled);
        break;
      }
      case host_comm::SHOOT_STATUS: {
        const auto & p = host.shoot;
        std::snprintf(
          buf, sizeof(buf), "[RX] 0x504 SHOOT    enable=%u fric=%u load=%u count=%u",
          p.shoot_enable, p.fric_state, p.load_state, p.shot_count);
        break;
      }
      case host_comm::HOST_LOG: {
        const auto & p = host.log;
        std::snprintf(
          buf, sizeof(buf), "[RX] 0x505 LOG      level=%u text=%.*s", p.level,
          static_cast<int>(p.length > sizeof(p.text) ? sizeof(p.text) : p.length), p.text);
        break;
      }
      case GAME_STATUS: {
        const auto & p = referee.game_status;
        std::snprintf(
          buf, sizeof(buf), "[RX] 0x001 GAME     type=%u progress=%u remain=%us sync=%llu",
          static_cast<unsigned>(p.game_type), static_cast<unsigned>(p.game_progress),
          p.stage_remain_time, static_cast<unsigned long long>(p.SyncTimeStamp));
        break;
      }
      case GAME_ROBOT_HP: {
        const auto & p = referee.game_robot_hp;
        std::snprintf(
          buf, sizeof(buf),
          "[RX] 0x003 HP       ally1=%u ally3=%u ally4=%u ally7=%u base=%u enemy_base=%u "
          "dmg_diff=%d",
          p.ally_1_robot_HP, p.ally_3_robot_HP, p.ally_4_robot_HP, p.ally_7_robot_HP,
          p.ally_base_HP, p.enemy_base_HP, p.damage_difference);
        break;
      }
      case ROBOT_STATUS: {
        const auto & p = referee.robot_status;
        std::snprintf(
          buf, sizeof(buf),
          "[RX] 0x201 ROBOT    id=%u lv=%u HP=%u/%u cool=%u heat_limit=%u power_limit=%u "
          "speed_limit=%.1fm/s mains(g%u c%u s%u)",
          p.robot_id, p.robot_level, p.current_HP, p.maximum_HP,
          p.shooter_barrel_cooling_value, p.shooter_barrel_heat_limit, p.chassis_power_limit,
          p.bullet_speed_limit, p.power_management_gimbal_output,
          p.power_management_chassis_output, p.power_management_shooter_output);
        break;
      }
      case POWER_HEAT_DATA: {
        const auto & p = referee.power_heat_data;
        std::snprintf(
          buf, sizeof(buf), "[RX] 0x202 HEAT     buffer=%uJ heat17=%u heat42=%u",
          p.buffer_energy, p.shooter_17mm_barrel_heat, p.shooter_42mm_barrel_heat);
        break;
      }
      case SHOOT_DATA: {
        const auto & p = referee.shoot_data;
        std::snprintf(
          buf, sizeof(buf), "[RX] 0x207 SHOOT    bullet_type=%u shooter=%u freq=%uHz speed=%.2fm/s",
          p.bullet_type, p.shooter_number, p.launching_frequency, p.initial_speed);
        break;
      }
      case PROJECTILE_ALLOWANCE: {
        const auto & p = referee.projectile_allowance;
        std::snprintf(
          buf, sizeof(buf), "[RX] 0x208 ALLOW    17mm=%u 42mm=%u coin=%u fortress=%u",
          p.projectile_allowance_17mm, p.projectile_allowance_42mm, p.remaining_gold_coin,
          p.projectile_allowance_fortress);
        break;
      }
      case SENTRY_INFO: {
        const auto & p = referee.sentry_info;
        std::snprintf(
          buf, sizeof(buf), "[RX] 0x20D SENTRY    info=0x%08X info2=0x%04X info3=0x%016llX",
          p.sentry_info, p.sentry_info_2, static_cast<unsigned long long>(p.sentry_info_3));
        break;
      }
      default:
        std::snprintf(buf, sizeof(buf), "[RX] 0x%03X OK", cmd_id);
        break;
    }
    emit(buf);
  }

  void print_unknown(const referee_comm::Frame & frame)
  {
    char buf[128];
    std::snprintf(
      buf, sizeof(buf), "[RX] 0x%03X UNKNOWN  bytes=%u  raw:", frame.cmd_id, frame.length);
    emit(std::string(buf) + to_hex(frame.data, frame.length));
  }

  double print_hz_;
  std::unordered_map<uint16_t, std::chrono::steady_clock::time_point> last_print_;
  std::unordered_map<uint16_t, bool> first_seen_;
  std::unordered_map<uint16_t, uint64_t> per_cmd_;
};

void receive_loop(serial::Serial & port, const Options & opt, Stats & stats, ConsoleDecoder & dec)
{
  referee_comm::Receiver receiver(dec);
  std::vector<uint8_t> buffer(1024);
  auto last_stat = std::chrono::steady_clock::now();

  while (!g_quit) {
    size_t n = 0;
    try {
      n = port.read(buffer.data(), buffer.size());
    } catch (const std::exception &) {
      continue;
    }
    if (n == 0) continue;

    stats.rx_bytes += n;
    receiver.Feed(buffer.data(), n);
    stats.rx_frames = receiver.stats().frames;

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_stat).count() >= 1.0) {
      last_stat = now;
      std::string line = "[STAT] rx=" + std::to_string(stats.rx_frames.load()) + " frames (" +
                         std::to_string(stats.rx_bytes.load()) + " B)  tx=" +
                         std::to_string(stats.tx_frames.load()) + "  err: crc8=" +
                         std::to_string(receiver.stats().crc8_errors) + " crc16=" +
                         std::to_string(receiver.stats().crc16_errors) + " len=" +
                         std::to_string(receiver.stats().payload_errors) + " garbage=" +
                         std::to_string(receiver.stats().garbage_bytes) + "  |  " +
                         dec.per_cmd_line();
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

  packet.pitch = 0.0f;
  packet.yaw = static_cast<float>(yaw);
  packet.shoot = 0;

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
  packet.target_angle = 0.0f;

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
    "[INFO] 0x0402 载荷: pitch=0 shoot=0（data 段 9 字节）; "
    "Ctrl+C 退出\n");
  std::fflush(stdout);

  Stats stats;
  ConsoleDecoder decoder(opt.print_hz);
  std::thread rx_thread(
    receive_loop, std::ref(port), std::cref(opt), std::ref(stats), std::ref(decoder));

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
        "[TX] 0x0402 AIM      yaw=%+.4f rad (%+.1f°) pitch=%+.4f shoot=0 seq=%u",
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
    "[INFO] 退出。累计: 收 %llu 帧 / %llu B, 发 %llu 帧\n",
    static_cast<unsigned long long>(stats.rx_frames.load()),
    static_cast<unsigned long long>(stats.rx_bytes.load()),
    static_cast<unsigned long long>(stats.tx_frames.load()));
  return 0;
}
