// ============================================================================
//  can_com_debug — 上位机 -> 下位机 CAN 运动指令例程（步兵 Mini-PC <-> C 板）
//
//  复用 io/can/ 的正式协议栈（常量见 io/can/soldier_can_config.hpp，CAN 2.0 / 1 Mbps）：
//    上位机 -> 下位机 0x510 指令帧（DLC=8）：
//      byte0 序号 | byte1 版本0x10|bit0跟踪|bit1开火建议 |
//      byte2-3 yaw(mrad,i16) | byte4-5 pitch_raw(mrad,i16) |
//      byte6-7 姿态参考时刻回显(ms)
//      跟踪时 yaw ∈ [-pi, pi]、pitch_raw ∈ [-0.25, 0.4] rad，且必须回显所用
//      0x511 姿态样本的 reference_ms（100ms 内有效），否则协议类自动降级为空闲帧
//    下位机 -> 上位机 0x511 姿态 / 0x512 底盘 / 0x513 机器人状态 / 0x514 热量
//
//  例程内容：
//    1) 后台线程按固定频率（默认 50Hz，协议上限 100Hz）发送 0x510：
//       无目标时发空闲帧保持链路，有目标时携带最新姿态参考进入跟踪
//    2) 周期打印 0x511~0x514 遥测与发送结果，电控重启自动检测（会话切换）
//    3) 两种运动方式：
//       交互模式：输入绝对 yaw pitch（度），云台转到位
//       --sweep 模式：yaw 恒角速度缓慢旋转、pitch 固定 0°，验证连续跟踪指令
//
//  用法：
//    ./can_com_debug [config.yaml] [发送频率Hz] [--sweep] [--sweep-rate=角速度deg/s]
//    ./can_com_debug                                # 交互模式，50Hz
//    ./can_com_debug configs/soldier_debug.yaml 50 --sweep --sweep-rate=5
//  交互命令：yaw pitch(度) 跟踪 / s 停止 / p 立即打印遥测 / q 退出，Ctrl+C 也可退出
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <iostream>
#include <mutex>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fmt/format.h>

#include "io/can/soldier_can_board.hpp"
#include "tools/exiter.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr double kRadToDeg = 180.0 / std::numbers::pi;

// 交互线程写入、发送线程读取的运动目标
struct Target
{
  std::mutex mutex;
  double yaw_rad = 0.0;        // 世界系绝对 yaw
  double pitch_raw_rad = 0.0;  // 已换算的编码器原始 pitch
  bool tracking = false;
};

const char * send_result_name(io::SoldierCanBoard::SendResult result)
{
  switch (result) {
    case io::SoldierCanBoard::SendResult::sent: return "已入队";
    case io::SoldierCanBoard::SendResult::stale_reference: return "姿态参考过期(等0x511)";
    case io::SoldierCanBoard::SendResult::invalid_command: return "角度非法,已改发空闲帧";
    case io::SoldierCanBoard::SendResult::control_unavailable: return "电控条件不满足,已改发空闲帧";
    case io::SoldierCanBoard::SendResult::transport_error: return "CAN发送失败";
  }
  return "未知结果";
}

std::string payload_hex(const io::soldier_can::Payload & payload)
{
  std::string hex;
  for (const auto byte : payload) hex += fmt::format("{:02X} ", byte);
  return hex;
}

// 下位机 -> 上位机 遥测打印（快照内各状态只在姿态序号相同时合并，可能缺项）
void print_telemetry(const io::SoldierCanBoard & board)
{
  const auto snapshot = board.snapshot();
  if (!snapshot) {
    fmt::print("[遥测] 未收到 0x511 姿态帧（检查下位机供电、CAN 接线与通道）\n");
    return;
  }
  const auto & pose = snapshot->pose.value;
  fmt::print(
    "[遥测] 姿态 yaw={:+7.2f}° pitch_raw={:+7.2f}° seq={:3d} ref={:5d}ms "
    "valid={:d} autoaim={:d} 展开={:d}",
    pose.yaw_rad * kRadToDeg, pose.pitch_raw_rad * kRadToDeg, pose.sequence, pose.reference_ms,
    pose.pose_valid ? 1 : 0, pose.autoaim_active ? 1 : 0, pose.unfolded ? 1 : 0);
  if (snapshot->robot) {
    const auto & robot = *snapshot->robot;
    fmt::print(
      " | 状态 id={} 弹速={:.2f} 使能={:d} 自瞄请求={:d}", robot.robot_id,
      robot.bullet_speed_m_s, robot.enabled ? 1 : 0, robot.autoaim_requested ? 1 : 0);
  }
  if (snapshot->heat) {
    const auto & heat = *snapshot->heat;
    fmt::print(" | 热量 {:.0f}/{:.0f} 冷却 {:.1f}", heat.heat, heat.heat_limit, heat.cooling_rate);
  }
  fmt::print("\n");
}

}  // namespace

int main(int argc, char * argv[]) try
{
  std::string config_path = "configs/soldier_debug.yaml";
  int rate_hz = 50;
  bool sweep = false;
  double sweep_rate_deg_s = 10.0;
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--sweep") sweep = true;
    else if (arg.rfind("--sweep-rate=", 0) == 0) sweep_rate_deg_s = std::stod(arg.substr(12));
    else positional.push_back(arg);
  }
  if (!positional.empty()) config_path = positional[0];
  if (positional.size() > 1) rate_hz = std::stoi(positional[1]);
  if (positional.size() > 2 || rate_hz < 1 || rate_hz > 100 || sweep_rate_deg_s <= 0 ||
      sweep_rate_deg_s > 180) {
    fmt::print(
      stderr,
      "用法: ./can_com_debug [config.yaml] [发送频率Hz<=100] [--sweep] [--sweep-rate=deg/s]\n");
    return 1;
  }

  // 与正式链路一致：pitch_raw = 编码器零点 - 方向 * (算法 pitch + 静态补偿)
  const auto yaml = tools::load(config_path);
  const double pitch_zero = yaml["pitch_encoder_zero_rad"].as<double>(0.0);
  const int pitch_direction = yaml["pitch_encoder_direction"].as<int>(1);
  const double yaw_offset = yaml["yaw_offset"].as<double>(0.0) * kDegToRad;
  const double pitch_offset = yaml["pitch_offset"].as<double>(0.0) * kDegToRad;
  if ((pitch_direction != -1 && pitch_direction != 1) || !std::isfinite(pitch_zero)) {
    fmt::print(stderr, "云台角度配置无效：pitch_encoder_direction 必须为 +1 或 -1\n");
    return 1;
  }

  // direct 模式：调试例程直接跟随输入角度，不要求电控自瞄请求等条件；
  // fire_advice 在非 ballistic 模式下强制为 0，调试安全。
  io::SoldierCanBoard board(config_path, io::SoldierCanBoard::AimMode::direct);

  Target target;
  const auto send_period = std::max(
    io::soldier_can::config::kMinimumSendPeriod, std::chrono::milliseconds(1000 / rate_hz));

  fmt::print(
    "CAN 指令例程：0x510@{:.0f}Hz，pitch_raw=零点({:+.3f})-方向({:+d})*pitch，遥测 0x511~0x514\n",
    1000.0 / std::chrono::duration<double, std::milli>(send_period).count(), pitch_zero,
    pitch_direction);

  // 发送线程：固定频率下发 0x510，打印结果变化与周期遥测
  std::thread sender([&] {
    auto next_send = io::SoldierCanBoard::Clock::now();
    auto next_log = next_send;
    std::optional<io::SoldierCanBoard::SendResult> last_result;
    std::uint64_t last_session = 0;
    while (!tools::exiting()) {
      std::this_thread::sleep_for(1ms);
      const auto now = io::SoldierCanBoard::Clock::now();
      if (now < next_send) continue;
      next_send = std::max(next_send + send_period, now);

      io::soldier_can::Command command;
      std::optional<io::SoldierCanBoard::PoseSample> reference;
      {
        const std::lock_guard lock(target.mutex);
        command.tracking = target.tracking;
        command.yaw_rad = tools::limit_rad(target.yaw_rad);
        command.pitch_raw_rad = std::clamp(
          target.pitch_raw_rad, io::soldier_can::config::kMinPitchRawRad,
          io::soldier_can::config::kMaxPitchRawRad);
      }
      if (command.tracking) {
        // 跟踪指令必须回显所用姿态样本；下位机未回传时协议类自动降级为空闲帧
        if (const auto snapshot = board.snapshot()) reference = snapshot->pose;
      }

      io::SoldierCanBoard::SendAttempt attempt{};
      const auto result = board.send(command, reference, &attempt);
      if (!last_result || result != *last_result) {
        last_result = result;
        fmt::print("[发送] {}  0x510 [{}]\n", send_result_name(result), payload_hex(attempt.payload));
      }

      if (now < next_log) continue;
      next_log = now + 500ms;
      const auto session = board.session_status();
      if (session.id != last_session) {
        last_session = session.id;
        fmt::print(
          "[会话] #{} {}（seq {}->{}, ref {}->{}ms，电控可能已重启）\n", session.id, session.reason,
          session.previous_seq, session.incoming_seq, session.previous_reference_ms,
          session.incoming_reference_ms);
      }
      print_telemetry(board);
    }
  });

  if (!sweep) {
    fmt::print("交互模式：输入绝对 yaw pitch（度，空格分隔）| s 停止 | p 遥测 | q 退出\n");
    std::string line;
    while (!tools::exiting() && std::cout << "yaw pitch (deg)> " && std::getline(std::cin, line)) {
      if (line == "q" || line == "Q") break;
      if (line == "s" || line == "S") {
        const std::lock_guard lock(target.mutex);
        target.tracking = false;
        fmt::print("已停止跟踪，下发空闲帧\n");
        continue;
      }
      if (line == "p" || line == "P") {
        print_telemetry(board);
        continue;
      }

      std::istringstream input(line);
      double yaw_deg = 0.0;
      double pitch_deg = 0.0;
      std::string extra;
      if (!(input >> yaw_deg >> pitch_deg) || (input >> extra) || !std::isfinite(yaw_deg) ||
          !std::isfinite(pitch_deg)) {
        fmt::print("输入无效：请输入两个有限数字（yaw pitch），或 s/p/q\n");
        continue;
      }

      const double pitch_raw = std::clamp(
        pitch_zero - pitch_direction * (pitch_deg * kDegToRad + pitch_offset),
        io::soldier_can::config::kMinPitchRawRad, io::soldier_can::config::kMaxPitchRawRad);
      {
        const std::lock_guard lock(target.mutex);
        target.yaw_rad = tools::limit_rad(yaw_deg * kDegToRad + yaw_offset);
        target.pitch_raw_rad = pitch_raw;
        target.tracking = true;
      }
      fmt::print(
        "跟踪目标 yaw={:+.2f}° pitch={:+.2f}°（raw={:+.4f} rad）\n", yaw_deg, pitch_deg, pitch_raw);
    }
  } else {
    fmt::print("扫掠模式：yaw 恒速 {:+.1f}°/s，pitch=0°，Ctrl+C 退出\n", sweep_rate_deg_s);
    const auto t0 = io::SoldierCanBoard::Clock::now();
    while (!tools::exiting()) {
      std::this_thread::sleep_for(10ms);
      const double elapsed_s =
        std::chrono::duration<double>(io::SoldierCanBoard::Clock::now() - t0).count();
      const std::lock_guard lock(target.mutex);
      target.yaw_rad = sweep_rate_deg_s * kDegToRad * elapsed_s;
      target.pitch_raw_rad = pitch_zero;  // 世界系 pitch=0 对应的编码器原始角
      target.tracking = true;
    }
  }

  // 退出前回空闲，让下位机明确收到"停止跟踪"
  {
    const std::lock_guard lock(target.mutex);
    target.tracking = false;
  }
  std::this_thread::sleep_for(3 * send_period);
  tools::request_exit();
  sender.join();
  fmt::print("can_com_debug 已退出\n");
  return 0;
}
catch (const std::exception & e) {
  fmt::print(stderr, "can_com_debug 异常退出: {}\n", e.what());
  return 1;
}
