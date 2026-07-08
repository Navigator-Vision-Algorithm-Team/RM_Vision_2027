#include <fmt/core.h>

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/serial_board.hpp"
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
  tools::Recorder main_recorder(30, "main");

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  auto yaml = tools::load(config_path);

  io::ROS2 ros2;
  tools::logger()->info("[Main] Initializing SerialBoard...");
  io::SerialBoard serial_board(config_path);
  tools::logger()->info("[Main] SerialBoard ready.");

  tools::logger()->info("[Main] Opening camera...");
  io::Camera camera(config_path, "main");
  tools::logger()->info("[Main] Camera ready: {}", camera.device_name());

  // 读取全向感知相机配置
  int omni_count = 0;
  if (yaml["omni_camera_count"]) omni_count = yaml["omni_camera_count"].as<int>();

  // 给主相机的 daemon 线程留出时间完成 SDK 初始化和 StartGrabbing，
  // 避免与全向相机的 SDK 调用并发（Hik SDK 枚举/打开设备不是完全线程安全的）
  if (omni_count > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::vector<std::unique_ptr<io::Camera>> omni_cameras;
  std::vector<std::unique_ptr<tools::Recorder>> omni_recorders;
  std::vector<omniperception::OmniCameraConfig> omni_configs;

  for (int i = 1; i <= omni_count; i++) {
    auto sursign = "omni" + std::to_string(i);
    auto cam = std::make_unique<io::Camera>(config_path, sursign);
    auto recorder = std::make_unique<tools::Recorder>(30, sursign);

    // 每个全向相机初始化后等待 300ms，确保其 daemon 完成 SDK 操作，
    // 防止下一个相机的枚举/打开与当前相机的 StartGrabbing 竞争 USB 资源
    if (i < omni_count) {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    omniperception::OmniCameraConfig cfg;
    cfg.camera = cam.get();
    cfg.recorder = std::shared_ptr<tools::Recorder>(std::move(recorder));
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

  auto yolo = std::make_shared<auto_aim::YOLO>(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);
  omniperception::Perceptron perceptron(omni_configs, config_path, yolo);

  omniperception::DetectionResult switch_target;
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  bool main_loop_started = false;
  bool game_started_logged = false;
  int frame_count = 0;
  bool spin_mode_initialized = false;
  double spin_yaw_angle = 0.0;
  std::chrono::steady_clock::time_point last_spin_timestamp;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) {
      continue;
    }

    Eigen::Quaterniond q = serial_board.imu_at(timestamp - 1ms);

    if (!main_loop_started) {
      main_loop_started = true;
      tools::logger()->info("[Main] Main loop started — receiving frames");
    }

    frame_count++;
    // 比赛未开始时跳过自瞄逻辑（裁判系统控制）
    if (!serial_board.is_game_started()) {
      if (!game_started_logged) {
        game_started_logged = true;
        tools::logger()->info("[Main] Game not started yet — recording raw frames, waiting...");
      }
      main_recorder.record(img, q, timestamp);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if (!game_started_logged) {
      game_started_logged = true;
      tools::logger()->info("[Main] Game started! Entering detection & tracking loop");
    }

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // MCU 复位信号 → 重置跟踪器
    if (serial_board.reset_pending()) {
      tracker.reset();
    }

    // 先做一次检测（用于在自旋模式下判断是否应停止自旋）
    auto armors = yolo->detect(img);

    io::Command command{false, false, 0, 0};

    if (omni_count == 0 && !spin_mode_initialized) {
      spin_mode_initialized = true;
      last_spin_timestamp = timestamp;
      tools::logger()->info("[Main] Omni camera count is 0, enabling spin mode after game start");
    }
    // 仅当没有全向相机、已经初始化自旋、且当前没有检测到目标、且跟踪器处于丢失状态时才自旋
    if (omni_count == 0 && spin_mode_initialized && armors.empty() && tracker.state() == "lost") {
      const double spin_speed = 0.05;  // rad/s，适中且平稳

      command.control = true;
      command.shoot = false;
      command.yaw = tools::limit_rad(spin_speed + gimbal_pos[0]);
      command.pitch = tools::limit_rad(0.0);

      main_recorder.record(img, q, timestamp);
      serial_board.send(command);
    } else {
      static bool first_detection_logged = false;
      if (!first_detection_logged) {
        first_detection_logged = true;
        tools::logger()->info("[Main] First detection complete: {} armors found", armors.size());
      }

      decider.get_invincible_armor(ros2.subscribe_enemy_status());

      decider.armor_filter(armors);

      decider.set_priority(armors);

      auto detection_queue = perceptron.get_detection_queue();

      decider.sort(detection_queue);

      auto [switch_target, targets] = tracker.track(detection_queue, armors, timestamp);

      /// 全向感知逻辑
      if (tracker.state() == "switching") {
        command.control = switch_target.armors.empty() ? false : true;
        command.shoot = false;
        command.pitch = tools::limit_rad(switch_target.delta_pitch);
        command.yaw = tools::limit_rad(switch_target.delta_yaw + gimbal_pos[0]);
      }

      else if (tracker.state() == "lost") {
        command = decider.decide(detection_queue);
        command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0]);
      }

      else {
        command = aimer.aim(targets, timestamp, serial_board.bullet_speed);
      }

      /// 发射逻辑
      command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
      // command.shoot = false;

      // 每 150 帧 (~5 秒) 输出一次跟踪状态
      if (frame_count % 150 == 0) {
        tools::logger()->info(
          "[Main] frame={} state={} armors={} targets={}", frame_count, tracker.state(),
          armors.size(), targets.size());
      }

      // 在画面上绘制检测框和跟踪信息
      for (const auto & armor : armors) {
        cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);
        std::string label = fmt::format("{} {} {:.2f}",
          auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name], armor.confidence);
        cv::putText(img, label, cv::Point(armor.box.x, armor.box.y - 5),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        for (const auto & pt : armor.points) {
          cv::circle(img, pt, 2, cv::Scalar(0, 0, 255), -1);
        }
      }
      cv::putText(img,
        fmt::format("State: {} | Targets: {}", tracker.state(), targets.size()),
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

      main_recorder.record(img, q, timestamp);
      serial_board.send(command);

      /// ROS2通信
      Eigen::Vector4d target_info = decider.get_target_info(armors, targets);
      ros2.publish(target_info);
    }
  }

  return 0;
}