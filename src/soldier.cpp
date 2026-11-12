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
#include <tuple>

#include "io/camera.hpp"
#include "io/can/soldier_can_board.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/debug_stream.hpp"
#include "tasks/auto_aim/multithread/yolo_pipeline.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace
{
std::optional<auto_aim::Color> enemy_color_from(
  const std::optional<io::soldier_can::RobotStatus> & robot)
{
  if (!robot || !robot->robot_status_valid) return std::nullopt;
  const auto id = robot->robot_id;
  if (id >= 1 && id <= 11) return auto_aim::Color::blue;
  if (id >= 101 && id <= 111) return auto_aim::Color::red;
  return std::nullopt;
}
}  // namespace

int run_soldier(int argc, char * argv[], const std::string & default_config) try
{
  const std::string keys =
    "{help h usage ? |       | 输出命令行参数说明}"
    "{can-rx-only | false | 只接收 CAN 并统计 0x511 到达间隔}"
    "{can-geometry-only | false | 强制使用几何瞄准}"
    "{@config-path | " + default_config + " | 位置参数，YAML 配置文件路径}";
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  const bool can_rx_only = cli.get<bool>("can-rx-only");
  const bool geometry_only = cli.get<bool>("can-geometry-only");
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }

  io::SoldierCanBoard can_board(config_path, io::SoldierCanBoard::AimMode::tracking_with_fire_advice);
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
  if ((pitch_direction != -1 && pitch_direction != 1) || !std::isfinite(pitch_zero)) {
    tools::logger()->error("[soldier] 云台角度配置无效，pitch_encoder_direction 必须为 +1 或 -1");
    return 1;
  }
  io::Camera camera(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  omniperception::Decider decider(config_path);
  auto_aim::DebugStream debug(config_path, "soldier");
  if (const auto debug_yaml = yaml["debug_ros2"];
      debug_yaml && debug_yaml["enable"].as<bool>(false) && !debug.enabled()) {
    tools::logger()->error("[soldier] ROS 2 调试流初始化失败");
    return 1;
  }
  // ROS 2 注册信号处理器后，再建立程序自己的退出处理器。
  tools::Exiter exiter;
  using auto_aim::multithread::CameraResult;
  using auto_aim::multithread::WaitStatus;
  auto_aim::multithread::CameraJob main_job;
  main_job.camera = &camera;
  main_job.keep_image = true;
  main_job.pin_cpu = false;           // 解除步兵 YOLO 工作线程的绑核
  main_job.single_thread_cpu = false; // 解除 OpenVINO 的单线程限制
  auto_aim::multithread::YoloPipeline pipeline(config_path, {main_job}, false);
  tools::logger()->info("[soldier] Tracker + IMM/EKF + Aimer，视觉开火建议已启用");

  auto last_session = can_board.session();
  std::optional<auto_aim::Color> last_enemy_color;
  auto last_send = io::SoldierCanBoard::TimePoint::min();
  auto last_log = io::SoldierCanBoard::Clock::now();
  std::optional<bool> last_autoaim_active;
  std::optional<auto_aim::DebugCanTx> last_debug_tx;
  io::soldier_can::Command command;
  std::optional<io::SoldierCanBoard::PoseSample> reference;
  auto command_frame_time = io::SoldierCanBoard::Clock::now();
  const auto monotonic_us = [](io::SoldierCanBoard::TimePoint stamp) {
    return std::chrono::duration_cast<std::chrono::microseconds>(stamp.time_since_epoch()).count();
  };
  const auto send_latest = [&] {
    const auto now = io::SoldierCanBoard::Clock::now();
    if (last_send != io::SoldierCanBoard::TimePoint::min() &&
        now - last_send < io::soldier_can::config::kMinimumSendPeriod) return;
    last_send = now;
    io::SoldierCanBoard::SendAttempt attempt{};
    // 发送间隔与图像帧间隔不同。复用最新解算时保留原参考，过期由 CAN 层改发空闲帧。
    const auto result = can_board.send(command, reference, &attempt);
    last_debug_tx = auto_aim::DebugCanTx{
      attempt.payload, monotonic_us(command_frame_time), monotonic_us(attempt.started_at),
      static_cast<int>(result), result != io::SoldierCanBoard::SendResult::transport_error};
  };

  while (!exiter.exit()) {
    CameraResult frame;
    const auto frame_status =
      pipeline.wait_main(frame, io::soldier_can::config::kMinimumSendPeriod);
    if (frame_status == WaitStatus::Stopped) break;

    // 模式只读最新的 0x511 接管反馈，相机暂时没有出帧时也照常更新。
    const auto status = can_board.snapshot();
    const auto autoaim_active =
      status ? std::optional{status->pose.value.autoaim_active} : std::nullopt;
    if (!autoaim_active.value_or(false)) command.fire_advice = false;
    const char * mode = !autoaim_active ? "未知（无新鲜 0x511）" :
      *autoaim_active ? "自瞄模式（C 板已接管）" : "自瞄未接管";
    if (last_autoaim_active != autoaim_active) {
      last_autoaim_active = autoaim_active;
      tools::logger()->info(
        "[soldier] 当前模式：{} | 0x511 autoaim_active={}", mode,
        !autoaim_active ? "unknown" : *autoaim_active ? "true" : "false");
    }

    if (frame_status == WaitStatus::Timeout || frame.img.empty()) {
      send_latest();
      continue;
    }
    const auto t = frame.timestamp;
    auto & armors = frame.armors;
    const auto detected_count = armors.size();
    const bool debug_frame = debug.frame(frame.img);
    debug.yolo_sample(frame.inference_ms);

    const auto robot = can_board.latest_robot_status();
    const auto enemy_color = enemy_color_from(robot);
    const auto frame_pose = can_board.pose_at(t - 1ms);
    const auto session = frame_pose ? frame_pose->reference.session : can_board.session();
    if (session != last_session || enemy_color != last_enemy_color) {
      tracker.reset();
      last_session = session;
      last_enemy_color = enemy_color;
    }

    command = {};
    reference.reset();
    command_frame_time = t;
    Eigen::Vector3d gimbal_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
    const auto_aim::Target * target = nullptr;
    std::string state = "waiting_pose";
    std::string note;
    aimer.debug_aim_point.valid = false;
    if (frame_pose) {
      const double pitch = pitch_direction * (frame_pose->pitch_raw_rad - pitch_zero);
      gimbal_pos = {frame_pose->yaw_rad, pitch, 0.0};
      target_pos = gimbal_pos;
      const Eigen::Quaterniond q(
        Eigen::AngleAxisd(gimbal_pos[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
      solver.set_R_gimbal2world(q.toRotationMatrix());

      state = "waiting_enemy_color";
      if (enemy_color) {
        decider.armor_filter(armors, *enemy_color);
        // 复用哨兵的主相机跟踪入口。步兵只有主相机，全向队列为空。
        target = std::get<1>(tracker.track({}, armors, t, false, frame.img.size()));
        state = tracker.state();
        if (target) {
          // 对接协议允许无有效测速时发送非发射几何角。弹道预测只用有效实测值。
          bool ballistic_aim =
            !geometry_only && robot->bullet_speed_valid && robot->bullet_speed_m_s > 0.0;
          io::Command aim{};
          if (ballistic_aim) {
            aim = aimer.aim_ballistic(*target, t, robot->bullet_speed_m_s);
            ballistic_aim = aim.control && std::isfinite(aim.yaw) && std::isfinite(aim.pitch);
          }
          // 单次弹道无解时仍可按文档发非发射几何角。
          if (!ballistic_aim) aim = aimer.aim_geometric(*target, t);
          note = ballistic_aim ? "ballistic" : "geometric";
          if (aim.control && std::isfinite(aim.yaw) && std::isfinite(aim.pitch)) {
            command.tracking = true;
            command.fire_advice = autoaim_active.value_or(false) && state == "tracking" &&
              std::any_of(armors.begin(), armors.end(), [&](const auto & armor) {
                return armor.name == target->name && armor.type == target->armor_type &&
                       armor.xyz_in_world.allFinite() && armor.xyz_in_world.squaredNorm() > 0.0 &&
                       armor.ypd_in_world.allFinite() && armor.ypr_in_world.allFinite();
              });
            command.yaw_rad = tools::limit_rad(aim.yaw);
            // Aimer 的 pitch 向上为负，和 Solver 姿态的 Ry 方向相同。
            command.pitch_raw_rad = std::clamp(
              pitch_zero + pitch_direction * aim.pitch,
              io::soldier_can::config::kMinPitchRawRad, io::soldier_can::config::kMaxPitchRawRad);
            reference = frame_pose->reference;
            target_pos = {
              command.yaw_rad, pitch_direction * (command.pitch_raw_rad - pitch_zero), 0.0};
          }
        }
      }
    }

    send_latest();

    const bool tx_tracking = last_debug_tx && (last_debug_tx->payload[1] & 0x01) != 0;
    if (debug_frame) {
      auto_aim::DebugStatus debug_status{
        gimbal_pos, target_pos, tx_tracking && last_debug_tx->sdk_accepted, false, state, note};
      debug_status.frame_us = monotonic_us(t);
      debug_status.can_tx = last_debug_tx;
      if (frame_pose && enemy_color) {
        for (const auto & armor : armors) {
          if (!armor.xyz_in_world.allFinite() || armor.xyz_in_world.squaredNorm() <= 0.0) continue;
          debug_status.pnp_results.push_back({armor.box, armor.xyz_in_gimbal});
          debug_status.observed_armor_world.push_back(armor.xyz_in_world);
        }
      }
      if (command.tracking && aimer.debug_aim_point.valid) {
        debug_status.has_aim_prediction = true;
        debug_status.aim_prediction_world = aimer.debug_aim_point.xyza.head<3>();
        const auto & timing = aimer.debug_prediction_timing;
        debug_status.prediction = auto_aim::DebugPrediction{
          monotonic_us(timing.target_time), timing.extra_delay_s, timing.flight_time_s};
        const auto & point = debug_status.aim_prediction_world;
        const auto pixels = solver.world2pixel({cv::Point3f(point.x(), point.y(), point.z())});
        if (!pixels.empty()) {
          debug_status.has_aim_prediction_pixel = true;
          debug_status.aim_prediction_pixel = pixels.front();
        }
      }
      if (target) {
        const auto x = target->ekf_x();
        debug_status.has_ekf = true;
        debug_status.target_center = {x[0], x[2], x[4]};
        debug_status.target_vel = {x[1], x[3]};
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
      debug.publish(armors, debug_status, frame_pose && enemy_color ? &solver : nullptr);
    }

    const auto now = io::SoldierCanBoard::Clock::now();
    if (now - last_log >= 500ms && last_debug_tx) {
      last_log = now;
      tools::logger()->info(
        "[soldier] 当前模式：{} | state={} boxes={} candidates={} yaw={:.1f} pitch={:.1f} "
        "tx_tracking={} tx_result={}",
        mode, state, detected_count, enemy_color ? armors.size() : 0,
        target_pos[0] * 180.0 / std::numbers::pi,
        target_pos[1] * 180.0 / std::numbers::pi, tx_tracking,
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
