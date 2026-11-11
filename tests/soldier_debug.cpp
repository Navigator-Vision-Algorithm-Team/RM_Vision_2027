// ============================================================================
//  【调试入口】soldier 的调试副本（正式比赛请用 src/ 下的目标）
//
//  保留 YOLO -> Tracker(EKF) -> Aimer 调试流程，云台数据通过达妙 CAN 2.0 收发。
//  auto_aim::DebugStream（ROS2 -> Foxglove）用于开发板上
//  没有桌面端时，在局域网内实时观测：
//      相机原图 / YOLO 预处理后的输入图 / 检测框 + 敌人数字编号 /
//      YOLO 处理帧率(FPS) / 当前云台位姿 / 下发给云台的目标位姿 /
//      当前装甲板的 PnP 距离与世界坐标
//  调试数据只在 DebugStream 内部流转，不参与任何控制指令的生成。
//  删除 yaml 里的 debug_ros2 段或置 enable: false 即可整体关闭。
// ============================================================================

#include <algorithm>
#include <chrono>
#include <exception>
#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>

#include "io/camera.hpp"
#include "io/can/soldier_can_board.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/debug_stream.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                           | 输出命令行参数说明}"
  "{can-geometry-only | false                  | 云台几何跟踪联调，始终关闭开火建议}"
  "{@config-path   | configs/soldier_debug.yaml | 位置参数，YAML 配置文件路径}";

int main(int argc, char * argv[]) try
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  const bool geometry_only = cli.get<bool>("can-geometry-only");
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }
  const auto aim_mode = geometry_only ? io::SoldierCanBoard::AimMode::geometric
                                     : io::SoldierCanBoard::AimMode::ballistic;
  auto yaml = tools::load(config_path);

  const double pitch_encoder_zero_rad = yaml["pitch_encoder_zero_rad"].as<double>(0.0);
  const int pitch_encoder_direction = yaml["pitch_encoder_direction"].as<int>(1);
  const bool pitch_encoder_calibrated = yaml["pitch_encoder_calibrated"].as<bool>(false);
  if (pitch_encoder_direction != -1 && pitch_encoder_direction != 1) {
    tools::logger()->error("[soldier_debug] pitch_encoder_direction 必须为 +1 或 -1");
    return 1;
  }
  if (!pitch_encoder_calibrated) {
    tools::logger()->warn("[soldier_debug] pitch 编码器未标定，CAN 仅发送空闲帧");
  }

  // YOLO 不做敌我过滤，颜色由配置决定
  auto enemy_color =
    (yaml["enemy_color"].as<std::string>("red") == "red") ? auto_aim::red : auto_aim::blue;

  io::Camera camera(config_path);  // 不带 sursign：读取 camera_name / exposure_ms / gain
  io::SoldierCanBoard can_board(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  // 【新增】ROS2 调试数据流（Foxglove 实时画面）：只读业务数据，不参与控制
  auto_aim::DebugStream debug(config_path, "soldier");
  if (const auto debug_yaml = yaml["debug_ros2"];
      debug_yaml && debug_yaml["enable"].as<bool>(false) && !debug.enabled()) {
    tools::logger()->error("[soldier_debug] ROS 2 调试流初始化失败");
    return 1;
  }

  // Exiter 必须在 DebugStream 之后构造：DebugStream 内部的 rclcpp::init 会用自己的
  // signal handler 接管 SIGINT/SIGTERM，后安装的 handler 才能让 Ctrl+C 重新置位退出标志。
  tools::Exiter exiter;

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  auto last_log_time = std::chrono::steady_clock::now();
  auto last_can_send_time = std::chrono::steady_clock::time_point::min();
  auto last_can_wait_log = std::chrono::steady_clock::time_point::min();
  auto last_session = can_board.session();
  std::optional<bool> last_autoaim_active;
  std::optional<auto_aim::DebugCanTx> last_debug_tx;
  const auto monotonic_us = [](std::chrono::steady_clock::time_point stamp) {
    return std::chrono::duration_cast<std::chrono::microseconds>(stamp.time_since_epoch()).count();
  };
  auto send_can = [&](io::soldier_can::Command command,
                      const std::optional<io::SoldierCanBoard::PoseSample> & reference = {}) {
    const auto now = std::chrono::steady_clock::now();
    if (last_can_send_time != std::chrono::steady_clock::time_point::min() &&
        now - last_can_send_time < io::soldier_can::config::kMinimumSendPeriod) return;
    last_can_send_time = now;
    io::SoldierCanBoard::SendAttempt attempt{};
    const auto result =
      can_board.send(command, reference, debug.enabled() ? &attempt : nullptr, aim_mode);
    if (debug.enabled()) {
      last_debug_tx = auto_aim::DebugCanTx{
        attempt.payload, monotonic_us(t), monotonic_us(attempt.started_at), static_cast<int>(result),
        result != io::SoldierCanBoard::SendResult::transport_error};
    }
  };

  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) continue;
    const bool debug_frame_requested = debug.frame(img);

    const auto frame_pose = can_board.pose_at(t - 1ms);
    const auto status = can_board.snapshot();
    if (status && last_autoaim_active != status->pose.value.autoaim_active) {
      last_autoaim_active = status->pose.value.autoaim_active;
      tools::logger()->info("[soldier_debug] C board autoaim_active={}", *last_autoaim_active);
    }
    // CAN 先发送新目标，C 板接受后通过 autoaim_active 回报接管。
    if (!frame_pose || !status || frame_pose->reference.session != status->pose.session ||
        !status->pose.value.pose_valid || !status->pose.value.unfolded ||
        !status->robot || !status->robot->robot_status_valid || !status->robot->enabled ||
        !status->robot->autoaim_requested) {
      send_can({});
      const auto now = std::chrono::steady_clock::now();
      if (last_can_wait_log == std::chrono::steady_clock::time_point::min() ||
          now - last_can_wait_log >= 1s) {
        last_can_wait_log = now;
        tools::logger()->info("[soldier_debug] 等待有效 CAN 姿态、整车使能、云台展开及自瞄请求");
      }
      if (debug_frame_requested) {
        debug.begin_yolo();
        auto armors = detector.detect(img);
        debug.end_yolo();
        armors.remove_if([&enemy_color](const auto_aim::Armor & armor) {
          return armor.color != enemy_color;
        });
        auto_aim::DebugStatus waiting;
        waiting.state = "waiting_can";
        waiting.frame_us = monotonic_us(t);
        waiting.can_tx = last_debug_tx;
        debug.publish(armors, waiting);
      }
      std::this_thread::sleep_for(1ms);
      continue;
    }

    // CAN 直接给出地面 yaw 和 pitch 编码器角度，标定后构造云台到世界系的旋转。
    const double pitch_world_rad =
      pitch_encoder_direction * (frame_pose->pitch_raw_rad - pitch_encoder_zero_rad);
    const Eigen::Quaterniond q(
      Eigen::AngleAxisd(frame_pose->yaw_rad, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch_world_rad, Eigen::Vector3d::UnitY()));
    solver.set_R_gimbal2world(q.toRotationMatrix());
    const Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // YOLO 检测。颜色先按步兵配置过滤，Tracker 仍负责连续帧目标关联与状态管理。
    debug.begin_yolo();  // 【新增】统计 YOLO 处理帧率与推理耗时
    auto armors = detector.detect(img);
    debug.end_yolo();  // 【新增】
    armors.remove_if([&enemy_color](const auto_aim::Armor & armor) {
      return armor.color != enemy_color;
    });

    // CAN 会话变化对应旧入口的下位机复位，清空旧跟踪状态。
    if (frame_pose->reference.session != last_session) {
      last_session = frame_pose->reference.session;
      tracker.reset();
    }

    // Tracker 内部完成 PnP、目标关联、EKF 预测与观测更新。
    // 使用相机帧时间戳 t，让滤波时间间隔对应真实帧间隔。
    const auto * target = tracker.track(armors, t, true, img.size());

    // 没有可用跟踪目标时保持当前云台角度；有效跟踪时由 Aimer 做未来位置和弹道预测。
    const auto & robot = *status->robot;
    const bool aim_ready = geometry_only || (robot.bullet_speed_valid && robot.bullet_speed_m_s > 0.0);
    io::Command command{false, false, gimbal_pos[0], gimbal_pos[1]};
    if (target != nullptr && aim_ready) {
      command = geometry_only ? aimer.aim_geometric(*target)
                              : aimer.aim_ballistic(*target, t, robot.bullet_speed_m_s);
      // 解算失败时，调试面板仍显示当前云台角。
      if (!command.control) {
        command.yaw = gimbal_pos[0];
        command.pitch = gimbal_pos[1];
      }
      command.shoot = false;  // 步兵程序不接管发射
    }

    // 0x510 发送地面 yaw 和编码器原始 pitch，回显本帧实际使用的姿态时间戳。
    io::soldier_can::Command can_command;
    can_command.tracking = command.control && pitch_encoder_calibrated;
    if (can_command.tracking) {
      can_command.yaw_rad = tools::limit_rad(command.yaw);
      can_command.pitch_raw_rad = pitch_encoder_zero_rad + pitch_encoder_direction * command.pitch;
    }
    send_can(can_command, frame_pose->reference);

    // 【新增】调试数据流：检测框 + YOLO FPS + 当前云台位姿 + 下发目标位姿
    std::string note;
    if (target != nullptr) {
      const auto observed = std::find_if(armors.begin(), armors.end(), [target](const auto & armor) {
        return armor.name == target->name && armor.type == target->armor_type &&
               armor.ypd_in_world[2] > 0 && armor.xyz_in_world.allFinite();
      });
      if (observed != armors.end()) {
        note = fmt::format(
          "pnp  {:.2f} m  world ({:+.2f}, {:+.2f}, {:+.2f})", observed->xyz_in_gimbal.norm(),
          observed->xyz_in_world.x(), observed->xyz_in_world.y(), observed->xyz_in_world.z());
      }
    }

    auto_aim::DebugStatus debug_status{
      gimbal_pos, Eigen::Vector3d(command.yaw, command.pitch, 0.0), can_command.tracking,
      command.shoot, aim_ready ? (target != nullptr ? "aim" : "no_target") : "waiting_bullet_speed",
      note};
    debug_status.frame_us = monotonic_us(t);
    debug_status.can_tx = last_debug_tx;
    debug.publish(armors, debug_status);

    // 画框与四个关键点（不做 imshow/imwrite，无显示器环境下安全）
    for (const auto & armor : armors) {
      cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);
      for (const auto & point : armor.points) {
        cv::circle(img, point, 2, cv::Scalar(0, 0, 255), -1);
      }
    }

    // 2Hz 打印解算与下发结果，便于无显示环境下调试
    const auto now = std::chrono::steady_clock::now();
    if (now - last_log_time > 500ms) {
      last_log_time = now;
      if (target != nullptr) {
        const Eigen::VectorXd x = target->ekf_x();
        tools::logger()->info(
          "[soldier] state={} {} | EKF center({:.2f}, {:.2f}, {:.2f}) velocity({:.2f}, {:.2f}, "
          "{:.2f}) | cmd yaw={:.1f} pitch={:.1f}",
          tracker.state(), auto_aim::ARMOR_NAMES[target->name], x[0], x[2], x[4], x[1], x[3], x[5],
          command.yaw * 57.3, command.pitch * 57.3);
      } else {
        tools::logger()->info(
          "[soldier] state={} 无有效跟踪目标 | 保持 yaw={:.1f} pitch={:.1f} | gimbal yaw={:.1f}",
          tracker.state(), command.yaw * 57.3, command.pitch * 57.3, gimbal_pos[0] * 57.3);
      }
    }
  }

  return 0;
}
catch (const std::exception & error)
{
  tools::logger()->error("[soldier_debug] {}", error.what());
  return 1;
}
