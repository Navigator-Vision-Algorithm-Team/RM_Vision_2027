#include <fmt/core.h>

#include <chrono>
#include <memory>
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
  // 串口初始化
  io::SerialBoard serial_board(config_path);
  tools::logger()->info("[Main] SerialBoard ready.");

  tools::logger()->info("[Main] Opening camera...");
  // 相机初始化
  io::Camera camera(config_path, "main");
  tools::logger()->info("[Main] Camera ready: {}", camera.device_name());

  int omni_count = 0;
  if (yaml["omni_camera_count"]) omni_count = yaml["omni_camera_count"].as<int>();

  // 缓冲时间， 确保全向相机初始化完成，避免出现空帧，但是我们的全向相机和主相机并不在同一个物理链路上，所以这个sleep并没有非常的必要
  // if (omni_count > 0) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // }

  // 初始化全向相机
  std::vector<std::unique_ptr<io::Camera>> omni_cameras;
  std::vector<omniperception::OmniCameraConfig> omni_configs;

  for (int i = 1; i <= omni_count; i++) {
    auto sursign = "omni" + std::to_string(i);
    auto cam = std::make_unique<io::Camera>(config_path, sursign);
    auto recorder = std::make_unique<tools::Recorder>(30, sursign);

    //缓冲时间
    if (i < omni_count) {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    //配置设置
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

  // 初始化其他组件
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

  bool game_started_logged = false;
  bool spin_mode_initialized = false;
  std::chrono::steady_clock::time_point last_spin_timestamp;

    // 全向感知缓存：全向发现目标后的 yaw/pitch，主相机锁定前反复播报
  bool has_omni_cache = false;
  double cached_omni_yaw = 0;
  double cached_omni_pitch = 0;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) {
      continue;
    }

    // 获取IMU数据，一个四元数，表示这个时候云台的朝向
    Eigen::Quaterniond q = serial_board.imu_at(timestamp - 1ms);

    // 比赛未开始时跳过自瞄逻辑（裁判系统控制）
    if (!serial_board.is_game_started()) {
      if (!game_started_logged) {
        game_started_logged = true;
        tools::logger()->info("[Main] Game not started yet, waiting...");
      }
      main_recorder.record(img, q, timestamp);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if (!game_started_logged) {
      game_started_logged = true;
      tools::logger()->info("[Main] Game started! Entering detection & tracking loop");
    }

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    if (serial_board.reset_pending()) {
      tracker.reset();
    }

    auto armors = yolo->detect(img);

    io::Command command{false, false, 0, 0};

    if (omni_count == 0 && !spin_mode_initialized) {
      spin_mode_initialized = true;
      last_spin_timestamp = timestamp;
      tools::logger()->info("[Main] Omni camera count is 0, enabling spin mode");
    }
    if (omni_count == 0 && spin_mode_initialized && armors.empty() && tracker.state() == "lost") {
      const double spin_speed = 0.05;

      command.control = true;
      command.shoot = false;
      command.yaw = tools::limit_rad(spin_speed + gimbal_pos[0]);
      command.pitch = tools::limit_rad(0.0);

      main_recorder.record(img, q, timestamp);
      serial_board.send(command);
    } else {
      decider.armor_filter(armors);

      decider.set_priority(armors);

      auto detection_queue = perceptron.get_detection_queue();

      decider.sort(detection_queue);

      auto [switch_target, targets] = tracker.track(detection_queue, armors, timestamp);

      if (tracker.state() == "switching") {
        command.control = switch_target.armors.empty() ? false : true;
        command.shoot = false;
        command.pitch = tools::limit_rad(switch_target.delta_pitch);
        command.yaw = tools::limit_rad(switch_target.delta_yaw + gimbal_pos[0]);
      }

      else if (tracker.state() == "lost") {
        command = decider.decide(detection_queue);
        if (command.control) {
          // 全向有新检测：更新缓存，发送新值
          command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0]);
          cached_omni_yaw = command.yaw;
          cached_omni_pitch = command.pitch;
          has_omni_cache = true;
        } else if (has_omni_cache) {
          // 全向暂时丢失：回放缓存值，保持云台继续转向目标
          command.control = true;
          command.shoot = false;
          command.yaw = cached_omni_yaw;
          command.pitch = cached_omni_pitch;
        }
      }

      else {
        command = aimer.aim(targets, timestamp, serial_board.bullet_speed);
        // aimer 返回云台坐标系的相对偏移，MCU 需要绝对位置
        // if (command.control) {
        //   command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0]);
        // }
        has_omni_cache = false;
      }

      command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

      // 绘制检测框和跟踪信息（用于赛后录像复盘）
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

      tools::logger()->info("yaw={:.1f}° gimbal={:.1f}° target={:.1f}°",
  command.yaw * 57.3, gimbal_pos[0] * 57.3, (command.yaw - gimbal_pos[0]) * 57.3);
      serial_board.send(command);

      io::NavCommand nav_command = ros2.subscribe_get_data();

      serial_board.send(nav_command);

      // //test//

      // sleep(5);
      // tools::logger()->info("sleeping```````");

      // //test//
    }
  }

  return 0;
}
