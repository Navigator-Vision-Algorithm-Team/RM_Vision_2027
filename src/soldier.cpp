#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <opencv2/opencv.hpp>
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
  "{help h usage ? |                      | 输出命令行参数说明}"
  "{@config-path   | configs/soldier.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[]) try
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto yaml = tools::load(config_path);

  const double pitch_encoder_zero_rad = yaml["pitch_encoder_zero_rad"].as<double>(0.0);
  const int pitch_encoder_direction = yaml["pitch_encoder_direction"].as<int>(1);
  const bool pitch_encoder_calibrated = yaml["pitch_encoder_calibrated"].as<bool>(false);
  if (pitch_encoder_direction != -1 && pitch_encoder_direction != 1) {
    tools::logger()->error("[soldier] pitch_encoder_direction 必须为 +1 或 -1");
    return 1;
  }
  if (!pitch_encoder_calibrated) {
    tools::logger()->warn(
      "[soldier] pitch encoder calibration is incomplete; tracking commands will stay idle");
  }

  io::SoldierCanBoard can_board(config_path);
  io::Camera camera(config_path);  // 不带 sursign：读取 camera_name / exposure_ms / gain

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  // 可选调试流直接读取 CAN 主链路的同帧结果，避免另起串口调试入口比对姿态。
  auto_aim::DebugStream debug(config_path, "soldier");
  if (const auto debug_yaml = yaml["debug_ros2"];
      debug_yaml && debug_yaml["enable"].as<bool>(false) && !debug.enabled()) {
    tools::logger()->error(
      "[soldier] ROS 2 debug stream unavailable; check the startup log and the ROS 2 build environment");
    return 1;
  }
  // ROS2 初始化会注册信号处理器，Exiter 需在它之后构造。
  tools::Exiter exiter;

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  auto last_log_time = std::chrono::steady_clock::now();
  auto last_can_send_time = std::chrono::steady_clock::time_point::min();
  auto last_can_warning_time = std::chrono::steady_clock::time_point::min();
  std::uint64_t last_session = can_board.session();
  bool waiting_logged = false;
  bool invalid_robot_logged = false;

  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) continue;
    const bool debug_frame_requested = debug.frame(img);

    const auto current_session = can_board.session();
    if (current_session != last_session) {
      tracker.reset();
      last_session = current_session;
      tools::logger()->info("[soldier] CAN session changed; cleared target tracker");
    }

    // 取与本帧图像时间相邻的姿态样本；之后把实际使用的参考时间戳回传给 C 板。
    const auto frame_pose = can_board.pose_at(t - 1ms);
    const auto status = can_board.snapshot();
    const bool can_control_ready =
      frame_pose && status && frame_pose->reference.session == status->pose.session &&
      status->pose.value.pose_valid &&
      status->pose.value.autoaim_active && status->pose.value.unfolded &&
      status->robot && status->robot->robot_status_valid && status->robot->enabled &&
      status->robot->autoaim_requested && status->robot->bullet_speed_valid;

    const auto now = std::chrono::steady_clock::now();
    const bool send_due =
      last_can_send_time == std::chrono::steady_clock::time_point::min() ||
      now - last_can_send_time >= io::soldier_can::config::kMinimumSendPeriod;
    auto send_idle = [&] {
      if (send_due) {
        const auto result = can_board.send({});
        if (result != io::SoldierCanBoard::SendResult::sent &&
            (last_can_warning_time == std::chrono::steady_clock::time_point::min() ||
             now - last_can_warning_time >= 1s)) {
          tools::logger()->warn("[soldier] CAN idle send status={}", static_cast<int>(result));
          last_can_warning_time = now;
        }
        last_can_send_time = now;
      }
    };
    auto publish_preview = [&](const char * state) {
      if (!debug_frame_requested) return;
      // 电控未就绪时，只在调试采样帧做检测，结果不送入 Tracker。
      debug.begin_yolo();
      const auto armors = detector.detect(img);
      debug.end_yolo();
      auto_aim::DebugStatus preview;
      preview.state = state;
      debug.publish(armors, preview);
    };

    if (!can_control_ready) {
      send_idle();
      if (!waiting_logged) {
        tools::logger()->info(
          "[soldier] Waiting for valid CAN pose, robot status, auto-aim request and bullet speed");
        waiting_logged = true;
      }
      publish_preview("waiting_can");
      std::this_thread::sleep_for(1ms);
      continue;
    }
    waiting_logged = false;

    // 协议回传地面 yaw 和 pitch 编码器角；按配置零点/方向构造算法所需姿态。
    const double pitch_world_rad = pitch_encoder_direction *
      (frame_pose->pitch_raw_rad - pitch_encoder_zero_rad);
    const Eigen::Quaterniond q(
      Eigen::AngleAxisd(frame_pose->yaw_rad, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch_world_rad, Eigen::Vector3d::UnitY()));
    solver.set_R_gimbal2world(q.toRotationMatrix());
    const Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    const auto robot_id = status->robot->robot_id;
    auto_aim::Color enemy_color;
    if (robot_id >= 1 && robot_id <= 11) {
      enemy_color = auto_aim::blue;
    } else if (robot_id >= 101 && robot_id <= 111) {
      enemy_color = auto_aim::red;
    } else {
      // 协议要求 robot_id 无效时不能猜测本机阵营。
      send_idle();
      if (!invalid_robot_logged) {
        tools::logger()->warn("[soldier] Invalid robot_id {}; keeping CAN command idle", robot_id);
        invalid_robot_logged = true;
      }
      publish_preview("invalid_robot_id");
      continue;
    }
    invalid_robot_logged = false;

    // YOLO 检测。敌方颜色来自 0x513.robot_id；Tracker 跳过 YAML 颜色过滤，只负责跟踪。
    debug.begin_yolo();
    auto armors = detector.detect(img);
    debug.end_yolo();
    armors.remove_if([&enemy_color](const auto_aim::Armor & armor) {
      return armor.color != enemy_color;
    });

    // Tracker 内部完成 PnP、目标关联、EKF 预测与观测更新。
    // 使用相机帧时间戳 t，让滤波时间间隔对应真实帧间隔。
    const auto * target = tracker.track(armors, t, false, img.size());

    // 有效跟踪时由 Aimer 做未来位置和弹道预测；无目标时 CAN 层发送 tracking=0。
    io::Command command{false, false, gimbal_pos[0], gimbal_pos[1]};
    if (target != nullptr) {
      command = aimer.aim(target, t, status->robot->bullet_speed_m_s);
      // 无有效瞄准解时，调试数据用当前云台角，CAN 发送空闲帧。
      if (!command.control) {
        command.yaw = gimbal_pos[0];
        command.pitch = gimbal_pos[1];
      }
      command.shoot = false;  // 步兵程序不接管发射
    }

    // 0x510：yaw 是地面角；pitch 转成编码器原始角。步兵当前不发送开火建议。
    if (send_due) {
      io::soldier_can::Command can_command;
      can_command.tracking = command.control && pitch_encoder_calibrated;
      can_command.fire_advice = false;
      if (can_command.tracking) {
        can_command.yaw_rad = tools::limit_rad(command.yaw);
        can_command.pitch_raw_rad = pitch_encoder_zero_rad +
          pitch_encoder_direction * command.pitch;
      }
      const auto send_result = can_board.send(can_command, frame_pose->reference);
      if (send_result != io::SoldierCanBoard::SendResult::sent &&
          (last_can_warning_time == std::chrono::steady_clock::time_point::min() ||
           now - last_can_warning_time >= 1s)) {
        tools::logger()->warn("[soldier] CAN send status={}", static_cast<int>(send_result));
        last_can_warning_time = now;
      }
      last_can_send_time = now;
    }

    if (debug_frame_requested) {
      auto_aim::DebugStatus debug_status{
        gimbal_pos, Eigen::Vector3d(command.yaw, command.pitch, 0.0),
        command.control && pitch_encoder_calibrated, false, tracker.state(), ""};
      if (target != nullptr) {
        const auto x = target->ekf_x();
        if (x.size() >= 11) {
          debug_status.has_ekf = true;
          debug_status.target_center = Eigen::Vector3d(x[0], x[2], x[4]);
          debug_status.target_vel = Eigen::Vector2d(x[1], x[3]);
          debug_status.target_w = x[7];
          debug_status.target_r = x[8];
          debug_status.model_prob = target->model_probabilities();
          debug_status.nis_ratio = target->nis_ratio();
          const auto convergence = target->debug_convergence_metrics();
          debug_status.has_ekf_convergence = true;
          debug_status.ekf_converged = convergence.converged;
          debug_status.ekf_convergence_score = convergence.score;
          debug_status.ekf_covariance_score = convergence.covariance_score;
          debug_status.ekf_consistency_score = convergence.consistency_score;
          debug_status.ekf_position_std = convergence.position_std;
          debug_status.ekf_velocity_std = convergence.velocity_std;
          debug_status.ekf_yaw_std = convergence.yaw_std;
          debug_status.ekf_angular_velocity_std = convergence.angular_velocity_std;
          debug_status.ekf_geometry_std = convergence.geometry_std;
          debug_status.ekf_nis_failure_rate = convergence.nis_failure_rate;
          debug_status.ekf_samples = convergence.samples;
          debug_status.ekf_min_samples = convergence.min_samples;
        }
      }
      if (target != nullptr && command.control && aimer.debug_aim_point.valid) {
        debug_status.has_aim_prediction = true;
        debug_status.aim_prediction_world = aimer.debug_aim_point.xyza.head<3>();
        const auto & p = debug_status.aim_prediction_world;
        const auto pixels = solver.world2pixel({cv::Point3f(p.x(), p.y(), p.z())});
        if (!pixels.empty()) {
          debug_status.has_aim_prediction_pixel = true;
          debug_status.aim_prediction_pixel = pixels.front();
        }
      }
      debug.publish(armors, debug_status, &solver);
    }

    // 2Hz 打印解算与下发结果，便于无显示环境下调试
    const auto log_now = std::chrono::steady_clock::now();
    if (log_now - last_log_time > 500ms) {
      last_log_time = log_now;
      if (target != nullptr) {
        const Eigen::VectorXd x = target->ekf_x();
        tools::logger()->info(
          "[soldier] state={} {} | EKF center({:.2f}, {:.2f}, {:.2f}) velocity({:.2f}, {:.2f}, {:.2f}) | cmd yaw={:.1f} pitch={:.1f}",
          tracker.state(), auto_aim::ARMOR_NAMES[target->name], x[0], x[2], x[4], x[1], x[3],
          x[5], command.yaw * 57.3, command.pitch * 57.3);
      } else {
        tools::logger()->info(
          "[soldier] state={} 无有效跟踪目标 | 保持 yaw={:.1f} pitch={:.1f} | gimbal yaw={:.1f}",
          tracker.state(),
          command.yaw * 57.3, command.pitch * 57.3, gimbal_pos[0] * 57.3);
      }
    }
  }

  return 0;
}
catch (const std::exception & error)
{
  tools::logger()->error("[soldier] {}", error.what());
  return 1;
}
