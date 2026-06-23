#include <fmt/core.h>

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder_main(30, "main");

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  auto yaml = tools::load(config_path);

  io::ROS2 ros2;
  io::CBoard cboard(config_path);
  io::Camera camera(config_path, "main");

  // 读取全向感知相机配置
  int omni_count = 0;
  if (yaml["omni_camera_count"]) omni_count = yaml["omni_camera_count"].as<int>();

  bool if_spin = false;
  if (omni_count == 0) if_spin = true;

  std::vector<std::unique_ptr<io::Camera>> omni_cameras;
  std::vector<omniperception::OmniCameraConfig> omni_configs;

  for (int i = 1; i <= omni_count; i++) {
    auto sursign = "omni" + std::to_string(i);
    auto cam = std::make_unique<io::Camera>(config_path, sursign);

    omniperception::OmniCameraConfig cfg;
    cfg.camera = cam.get();
    cfg.mount_yaw = yaml["omni_mount_yaw_" + std::to_string(i)]
                      ? yaml["omni_mount_yaw_" + std::to_string(i)].as<double>()
                      : 0.0;
    cfg.mount_pitch = yaml["omni_mount_pitch_" + std::to_string(i)]
                        ? yaml["omni_mount_pitch_" + std::to_string(i)].as<double>()
                        : 0.0;
    cfg.fov_h = yaml["omni_fov_h_" + std::to_string(i)]
                  ? yaml["omni_fov_h_" + std::to_string(i)].as<double>()
                  : 54.2;
    cfg.fov_v = yaml["omni_fov_v_" + std::to_string(i)]
                  ? yaml["omni_fov_v_" + std::to_string(i)].as<double>()
                  : 44.5;

    omni_configs.push_back(cfg);
    omni_cameras.push_back(std::move(cam));
  }

  // 为每个全向相机创建独立的 Recorder
  std::vector<std::unique_ptr<tools::Recorder>> omni_recorders;
  std::vector<tools::Recorder *> omni_recorder_ptrs;
  for (int i = 1; i <= omni_count; i++) {
    auto name = "omni" + std::to_string(i);
    omni_recorders.push_back(std::make_unique<tools::Recorder>(30, name));
    omni_recorder_ptrs.push_back(omni_recorders.back().get());
  }

  auto yolo = std::make_shared<auto_aim::YOLO>(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);
  omniperception::Perceptron perceptron(omni_configs, config_path, yolo, omni_recorder_ptrs);

  omniperception::DetectionResult switch_target;
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  tools::logger()->info("[Main] Initialization complete, entering main loop.");
  int frame_count = 0;
  auto last_status_time = std::chrono::steady_clock::now();
  std::string last_tracker_state;
  int last_armor_count = -1;

  while (!exiter.exit()) {
    frame_count++;
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo->detect(img);

    // --- draw detection boxes and record main camera ---
    {
      cv::Mat annotated = img.clone();
      for (const auto & armor : armors) {
        std::vector<cv::Point> pts;
        for (const auto & p : armor.points) pts.emplace_back(p);
        cv::polylines(annotated, pts, true, {0, 255, 0}, 2);
        auto label = fmt::format(
          "{} {:.2f}", auto_aim::ARMOR_NAMES[armor.name], armor.confidence);
        cv::putText(
          annotated, label, armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 1);
      }
      recorder_main.record(annotated, q, timestamp);
    }

    decider.get_invincible_armor(ros2.subscribe_enemy_status());

    decider.armor_filter(armors);

    decider.set_priority(armors);

    // --- armor count change event ---
    int armor_count = static_cast<int>(armors.size());
    if (armor_count != last_armor_count) {
      if (armor_count > 0 && last_armor_count <= 0)
        tools::logger()->info(
          "[Main] Armor detected! count={} color={} name={} conf={:.2f}",
          armor_count,
          auto_aim::COLORS[armors.front().color],
          auto_aim::ARMOR_NAMES[armors.front().name],
          armors.front().confidence);
      else if (armor_count == 0 && last_armor_count > 0)
        tools::logger()->info("[Main] Armor lost.");
      last_armor_count = armor_count;
    }

    auto detection_queue = perceptron.get_detection_queue();

    decider.sort(detection_queue);

    auto [switch_target, targets] = tracker.track(detection_queue, armors, timestamp);

    // --- tracker state change ---
    if (tracker.state() != last_tracker_state) {
      tools::logger()->info("[Main] tracker state: {} -> {}", last_tracker_state, tracker.state());
      last_tracker_state = tracker.state();
    }

    // --- debug status logging (1 Hz) ---
    auto now = std::chrono::steady_clock::now();
    auto elapsed = tools::delta_time(now, last_status_time);
    if (elapsed >= 1.0) {
      tools::logger()->info(
        "[Main] fps={:.0f} armors={} omni={} tracker={} bullet={:.1f}",
        frame_count / elapsed,
        armors.size(), detection_queue.size(), tracker.state(), cboard.bullet_speed);
      frame_count = 0;
      last_status_time = now;
    }
    // --- end debug ---

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    if (tracker.state() == "switching") {
      command.control = switch_target.armors.empty() ? false : true;
      command.shoot = false;
      command.pitch = tools::limit_rad(switch_target.delta_pitch);
      command.yaw = tools::limit_rad(switch_target.delta_yaw + gimbal_pos[0]);
    }

    else if (tracker.state() == "lost") {

      if (detection_queue.empty() && if_spin) {
        command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0] + 4);
      } else {
        command = decider.decide(detection_queue);
        command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0]);
      }
    }

    else {
      command = aimer.aim(targets, timestamp, cboard.bullet_speed);
      // --- aimer result output (0.5 Hz) ---
      if (!targets.empty()) {
        static auto last_aim_log = std::chrono::steady_clock::now();
        if (tools::delta_time(std::chrono::steady_clock::now(), last_aim_log) >= 0.5) {
          auto & tgt = targets.front();
          auto x = tgt.ekf_x();
          tools::logger()->info(
            "[Aimer] cmd_yaw={:.2f}deg cmd_pitch={:.2f}deg shoot={} ctrl={}",
            command.yaw * 57.3, command.pitch * 57.3, command.shoot, command.control);
          tools::logger()->info(
            "[Aimer] target xyz=({:.2f},{:.2f},{:.2f})m dist={:.2f}m yaw={:.1f}deg r={:.2f}",
            x[0], x[2], x[4], x[8], x[6] * 57.3, x[8]);
          last_aim_log = std::chrono::steady_clock::now();
        }
      }
    }

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
    // command.shoot = false;

    // --- 低通滤波：吸收云台机械抖动导致的高频振荡 ---
    {
      static io::Command last_cmd{false, false, 0, 0};
      static bool last_control = false;

      // 从不受控→受控时重置滤波状态，避免 auto-spin 的旧值污染跟踪指令
      if (!last_control && command.control) last_cmd = command;
      last_control = command.control;

      // pitch: 强滤波 + 死区（0.3°以内不动）
      constexpr double pitch_alpha = 0.12;   // 越小越平滑
      constexpr double pitch_deadband = 0.3 / 57.3;  // 0.3° 死区
      double pitch_delta = command.pitch - last_cmd.pitch;
      if (std::abs(pitch_delta) < pitch_deadband)
        command.pitch = last_cmd.pitch;
      else
        command.pitch = pitch_alpha * command.pitch + (1.0 - pitch_alpha) * last_cmd.pitch;

      // yaw: 弱滤波（不能影响正常跟踪响应）
      constexpr double yaw_alpha = 0.35;
      double yaw_delta = command.yaw - last_cmd.yaw;
      yaw_delta = tools::limit_rad(yaw_delta);
      command.yaw = last_cmd.yaw + yaw_alpha * yaw_delta;

      // 云台不受控时不更新滤波状态（避免积累偏差）
      if (command.control) last_cmd = command;
    }

    cboard.send(command);

    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);
  }

  return 0;
}