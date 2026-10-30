#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

#include "io/camera.hpp"
#include "io/can/cboard.hpp"
#include "io/serial/serial_board.hpp"
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

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto yaml = tools::load(config_path);

  // SerialBoard 构造函数强制读取 com_port，缺失时会直接 exit(1)，这里提前给出可操作提示
  if (!yaml["com_port"]) {
    tools::logger()->error(
      "[soldier] {} 缺少串口配置，SerialBoard 需要以下键：\n"
      "  com_port: \"/dev/gimbal\"\n"
      "  baud_rate: 921600\n"
      "  bullet_speed: 28.0\n"
      "  use_referee_system: false",
      config_path);
    return 1;
  }

  // Tracker / Aimer 构造时用 yaml[...].as<T>() 读取参数，缺键会抛 YAML::BadConversion
  // 直接终止进程，这里提前检查并列出缺失的键。
  const std::vector<std::string> tracking_keys = {
    "min_detect_count",
    "max_temp_lost_count",
    "outpost_max_temp_lost_count",
    "yaw_offset",
    "pitch_offset",
    "comming_angle",
    "leaving_angle",
    "decision_speed",
    "high_speed_delay_time",
    "low_speed_delay_time"};
  std::string missing_keys;
  for (const auto & key : tracking_keys) {
    if (!yaml[key]) missing_keys += "  " + key + "\n";
  }
  if (!missing_keys.empty()) {
    tools::logger()->error(
      "[soldier] {} 缺少跟踪/瞄准配置，Tracker 与 Aimer 需要：\n{}", config_path, missing_keys);
    return 1;
  }

  // YOLO 不做敌我过滤，颜色由配置决定
  auto enemy_color =
    (yaml["enemy_color"].as<std::string>("red") == "red") ? auto_aim::red : auto_aim::blue;

  io::Camera camera(config_path);  // 不带 sursign：读取 camera_name / exposure_ms / gain
  io::SerialBoard serial_board(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  // Tracker：目标选择 + 整车 EKF（11 维状态），负责装甲板匹配与短时丢框外推
  auto_aim::Tracker tracker(config_path, solver);
  // Aimer：时间补偿 + 选板 + 弹道解算，输出世界系绝对 yaw/pitch
  auto_aim::Aimer aimer(config_path);

  // ROS2 调试数据流（Foxglove）：只读业务数据，不参与控制；debug_ros2.enable=false 时整体关闭
  auto_aim::DebugStream debug(config_path, "soldier");

  // Exiter 必须在 DebugStream 之后构造：DebugStream 内部的 rclcpp::init 会用自己的
  // signal handler 接管 SIGINT/SIGTERM，后安装的 handler 才能让 Ctrl+C 重新置位退出标志。
  tools::Exiter exiter;

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto last_log_time = std::chrono::steady_clock::now();
  bool game_started_logged = false;

  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) continue;
    debug.frame(img);  // 抓取本帧原图（内部按 image_hz 节流）

    // IMU 四元数 -> 云台到世界系的旋转，PnP 结果据此转到世界系
    q = serial_board.imu_at(t - 1ms);
    solver.set_R_gimbal2world(q);
    const Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // 裁判系统判定未开赛时不下发指令（use_referee_system=false 时恒为 true）
    if (!serial_board.is_game_started()) {
      if (!game_started_logged) {
        game_started_logged = true;
        tools::logger()->info("[soldier] 等待比赛开始（裁判系统未就绪），暂停自瞄");
      }
      std::this_thread::sleep_for(10ms);
      continue;
    }

    // YOLO 检测
    debug.begin_yolo();
    auto armors = detector.detect(img);
    debug.end_yolo();
    armors.remove_if([&enemy_color](const auto_aim::Armor & armor) {
      return armor.color != enemy_color;
    });

    // 下位机复位信号：清空跟踪状态，避免用上一局的目标继续外推
    if (serial_board.reset_pending()) tracker.reset();

    // Tracker 接管目标选择与 PnP：只有选中的目标会做解算，输出 0 或 1 个整车目标。
    // 状态机 lost -> detecting -> tracking -> temp_lost 保证短时丢框仍能按预测位置瞄准。
    auto targets = tracker.track(armors, t);

    // 默认指令：保持当前朝向、不开火。
    // 0x0402 报文没有 control 位，只有 shoot 位受 control 影响，所以无目标时必须回填
    // 当前云台角，否则云台会奔向 (0, 0)。
    io::Command command{false, false, gimbal_pos[0], gimbal_pos[1]};

    // Aimer：预测到子弹出膛时刻 + 选板 + 弹道解算，输出世界系绝对 yaw/pitch。
    // control=false 表示当前不可瞄准（无目标 / 选板无效 / 弹道无解），此时保持位姿。
    const auto aim_command = aimer.aim(targets, t, serial_board.bullet_speed);
    if (aim_command.control) {
      command.control = true;
      command.yaw = aim_command.yaw;
      command.pitch = aim_command.pitch;
    }
    command.shoot = false;  // 本程序只做瞄准，不接管发射

    // 0x0402 下发到下位机，由下位机驱动云台；shoot 恒为 0
    serial_board.send(command);

    // 画框与四个关键点（不做 imshow/imwrite，无显示器环境下安全）
    for (const auto & armor : armors) {
      cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);
      for (const auto & point : armor.points) {
        cv::circle(img, point, 2, cv::Scalar(0, 0, 255), -1);
      }
    }

    // 调试数据流：跟踪状态 / 预测位置与速度 / 下发角度
    auto_aim::DebugStatus status;
    status.gimbal_ypr = gimbal_pos;
    status.target_ypr = Eigen::Vector3d(command.yaw, command.pitch, 0.0);
    status.control = command.control;
    status.shoot = command.shoot;
    status.state = tracker.state();
    if (!targets.empty()) {
      auto & target = targets.front();
      const Eigen::VectorXd x = target.ekf_x();  // x vx y vy z vz a w r l h
      if (x.size() >= 11) {
        status.has_track = true;
        status.track_xyz = Eigen::Vector3d(x[0], x[2], x[4]);
        status.track_vel = Eigen::Vector3d(x[1], x[3], x[5]);
        status.track_w = x[7];
        status.track_r = x[8];
        status.converged = target.convergened();
      }
    }
    debug.publish(armors, status);

    // 2Hz 打印跟踪与下发结果，便于无显示环境下调试
    const auto now = std::chrono::steady_clock::now();
    if (now - last_log_time > 500ms) {
      last_log_time = now;
      if (!targets.empty()) {
        const auto & target = targets.front();
        const Eigen::VectorXd x = target.ekf_x();
        tools::logger()->info(
          "[soldier] {} {} | state {} | d {:.2f}m | v({:+.2f},{:+.2f},{:+.2f}) w {:+.1f}deg/s "
          "r {:.3f}m | aim {} | cmd yaw={:.1f} pitch={:.1f}",
          auto_aim::ARMOR_NAMES[target.name], auto_aim::ARMOR_TYPES[target.armor_type],
          tracker.state(), std::hypot(x[0], x[2]), x[1], x[3], x[5], x[7] * 57.3, x[8],
          aimer.debug_aim_point.valid ? "ok" : "invalid", command.yaw * 57.3,
          command.pitch * 57.3);
      } else {
        tools::logger()->info(
          "[soldier] 无目标 | state {} | 保持 yaw={:.1f} pitch={:.1f} | gimbal yaw={:.1f}",
          tracker.state(), command.yaw * 57.3, command.pitch * 57.3, gimbal_pos[0] * 57.3);
      }
    }
  }

  return 0;
}
