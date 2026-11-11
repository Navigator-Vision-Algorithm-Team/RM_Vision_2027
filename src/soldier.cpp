#include "soldier.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <numbers>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>

#include "io/camera.hpp"
#include "io/can/soldier_can_board.hpp"
#include "tasks/auto_aim/debug_stream.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

int run_soldier(int argc, char * argv[], const std::string & default_config) try
{
  const std::string keys =
    "{help h usage ? |       | 输出命令行参数说明}"
    "{can-rx-only | false | 只接收 CAN 并统计 0x511 到达间隔}"
    "{can-geometry-only | true | 兼容旧启动命令，当前始终直接跟随}"
    "{@config-path | " + default_config + " | 位置参数，YAML 配置文件路径}";
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  const bool can_rx_only = cli.get<bool>("can-rx-only");
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }

  io::SoldierCanBoard can_board(config_path, io::SoldierCanBoard::AimMode::direct);
  if (can_rx_only) {
    tools::Exiter exiter;
    tools::logger()->info("[soldier] CAN receive-only diagnostics");
    while (!exiter.exit()) {
      std::this_thread::sleep_for(1s);
      const auto stats = can_board.take_pose_rx_stats();
      tools::logger()->info(
        "[soldier] 0x511 frames={} gaps_under_2ms={} gaps_at_least_100ms={} max_gap_ms={:.3f}",
        stats.frames, stats.gaps_under_2ms, stats.gaps_at_least_100ms, stats.max_gap_ms);
    }
    return 0;
  }

  const auto yaml = tools::load(config_path);
  const double pitch_zero = yaml["pitch_encoder_zero_rad"].as<double>(0.0);
  const int pitch_direction = yaml["pitch_encoder_direction"].as<int>(1);
  const double yaw_offset = yaml["yaw_offset"].as<double>(0.0) * std::numbers::pi / 180.0;
  const double pitch_offset = yaml["pitch_offset"].as<double>(0.0) * std::numbers::pi / 180.0;
  if ((pitch_direction != -1 && pitch_direction != 1) || !std::isfinite(pitch_zero) ||
      !std::isfinite(yaw_offset) || !std::isfinite(pitch_offset)) {
    tools::logger()->error("[soldier] 云台角度配置无效，pitch_encoder_direction 必须为 +1 或 -1");
    return 1;
  }

  io::Camera camera(config_path);
  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::DebugStream debug(config_path, "soldier");
  if (const auto debug_yaml = yaml["debug_ros2"];
      debug_yaml && debug_yaml["enable"].as<bool>(false) && !debug.enabled()) {
    tools::logger()->error("[soldier] ROS 2 调试流初始化失败");
    return 1;
  }
  // ROS 2 注册信号处理器后，再建立程序自己的退出处理器。
  tools::Exiter exiter;
  tools::logger()->info("[soldier] YOLO 直接跟随，fire_advice=0");

  cv::Mat img;
  io::SoldierCanBoard::TimePoint t;
  auto last_send = io::SoldierCanBoard::TimePoint::min();
  auto last_log = io::SoldierCanBoard::Clock::now();
  std::optional<bool> last_autoaim_active;
  std::optional<auto_aim::DebugCanTx> last_debug_tx;
  const auto monotonic_us = [](io::SoldierCanBoard::TimePoint stamp) {
    return std::chrono::duration_cast<std::chrono::microseconds>(stamp.time_since_epoch()).count();
  };

  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) continue;
    const bool debug_frame = debug.frame(img);
    debug.begin_yolo();
    auto armors = detector.detect(img);
    debug.end_yolo();

    const auto frame_pose = can_board.pose_at(t - 1ms);
    const auto status = can_board.snapshot();
    if (status && last_autoaim_active != status->pose.value.autoaim_active) {
      last_autoaim_active = status->pose.value.autoaim_active;
      tools::logger()->info("[soldier] C board autoaim_active={}", *last_autoaim_active);
    }

    io::soldier_can::Command command;
    std::optional<io::SoldierCanBoard::PoseSample> reference;
    Eigen::Vector3d gimbal_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
    const char * state = armors.empty() ? "no_target" : "waiting_pose";
    bool pnp_solved = false;
    if (frame_pose) {
      const double pitch = pitch_direction * (frame_pose->pitch_raw_rad - pitch_zero);
      gimbal_pos = {frame_pose->yaw_rad, pitch, 0.0};
      target_pos = gimbal_pos;
      const Eigen::Quaterniond q(
        Eigen::AngleAxisd(gimbal_pos[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
      solver.set_R_gimbal2world(q.toRotationMatrix());

      if (!armors.empty()) {
        auto & armor = armors.front();
        try {
          pnp_solved = solver.solve(armor) && armor.xyz_in_world.allFinite() &&
                       armor.xyz_in_world.squaredNorm() > 0.0;
        } catch (const cv::Exception &) {
          pnp_solved = false;
        }
        // PnP 缺少可用深度时仍可沿检测中心的视线跟随。
        const Eigen::Vector3d direction = pnp_solved ? armor.xyz_in_world : solver.world_ray(armor.center);
        const auto ypd = tools::xyz2ypd(direction);
        command.tracking = true;
        command.yaw_rad = tools::limit_rad(ypd[0] + yaw_offset);
        command.pitch_raw_rad = std::clamp(
          pitch_zero - pitch_direction * (ypd[1] + pitch_offset),
          io::soldier_can::config::kMinPitchRawRad, io::soldier_can::config::kMaxPitchRawRad);
        reference = frame_pose->reference;
        target_pos = {
          command.yaw_rad, pitch_direction * (command.pitch_raw_rad - pitch_zero), 0.0};
        state = "tracking";
      }
    }

    const auto now = io::SoldierCanBoard::Clock::now();
    if (last_send == io::SoldierCanBoard::TimePoint::min() ||
        now - last_send >= io::soldier_can::config::kMinimumSendPeriod) {
      last_send = now;
      io::SoldierCanBoard::SendAttempt attempt{};
      const auto result = can_board.send(command, reference, &attempt);
      last_debug_tx = auto_aim::DebugCanTx{
        attempt.payload, monotonic_us(t), monotonic_us(attempt.started_at), static_cast<int>(result),
        result != io::SoldierCanBoard::SendResult::transport_error};
    }

    if (debug_frame) {
      auto_aim::DebugStatus debug_status{
        gimbal_pos, target_pos, command.tracking, false, state, "YOLO DIRECT"};
      debug_status.frame_us = monotonic_us(t);
      debug_status.can_tx = last_debug_tx;
      if (pnp_solved) debug_status.observed_armor_world.push_back(armors.front().xyz_in_world);
      debug.publish(armors, debug_status, frame_pose ? &solver : nullptr);
    }

    if (now - last_log >= 500ms && last_debug_tx) {
      last_log = now;
      tools::logger()->info(
        "[soldier] state={} boxes={} yaw={:.1f} pitch={:.1f} tx_tracking={} tx_result={}",
        state, armors.size(), target_pos[0] * 180.0 / std::numbers::pi,
        target_pos[1] * 180.0 / std::numbers::pi, (last_debug_tx->payload[1] & 0x01) != 0,
        last_debug_tx->result);
    }
  }
  return 0;
}
catch (const std::exception & error)
{
  tools::logger()->error("[soldier] {}", error.what());
  return 1;
}
