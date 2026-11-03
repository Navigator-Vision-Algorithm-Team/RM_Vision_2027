#include <chrono>
#include <cmath>
#include <limits>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/can/cboard.hpp"
#include "io/serial/serial_board.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
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

  // YOLO 不做敌我过滤，颜色由配置决定
  auto enemy_color =
    (yaml["enemy_color"].as<std::string>("red") == "red") ? auto_aim::red : auto_aim::blue;

  tools::Exiter exiter;

  io::Camera camera(config_path);  // 不带 sursign：读取 camera_name / exposure_ms / gain
  io::SerialBoard serial_board(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto last_log_time = std::chrono::steady_clock::now();
  bool game_started_logged = false;

  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) continue;

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

    // YOLO 检测。颜色先按步兵配置过滤，Tracker 仍负责连续帧目标关联与状态管理。
    auto armors = detector.detect(img);
    armors.remove_if([&enemy_color](const auto_aim::Armor & armor) {
      return armor.color != enemy_color;
    });

    // Tracker 内部完成 PnP、目标关联、EKF 预测与观测更新。
    // 使用相机帧时间戳 t，让滤波时间间隔对应真实帧间隔。
    auto targets = tracker.track(armors, t);

    // 没有可用跟踪目标时保持当前云台角度；有效跟踪时由 Aimer 做未来位置和弹道预测。
    io::Command command{false, false, gimbal_pos[0], gimbal_pos[1]};
    if (!targets.empty()) {
      command = aimer.aim(targets, t, serial_board.bullet_speed);
      // Aimer 在无效瞄准点或弹道不可解时返回 control=false、角度为 0；
      // 串口仍携带角度字段，因此退回当前云台角，避免无效指令造成跳转。
      if (!command.control) {
        command.yaw = gimbal_pos[0];
        command.pitch = gimbal_pos[1];
      }
      command.shoot = false;  // 步兵程序不接管发射
    }

    // 0x0402 下发到下位机，由下位机驱动云台；shoot 恒为 0
    serial_board.send(command);

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
      if (!targets.empty()) {
        const auto & target = targets.front();
        const Eigen::VectorXd x = target.ekf_x();
        tools::logger()->info(
          "[soldier] state={} {} | EKF center({:.2f}, {:.2f}, {:.2f}) velocity({:.2f}, {:.2f}, {:.2f}) | cmd yaw={:.1f} pitch={:.1f}",
          tracker.state(), auto_aim::ARMOR_NAMES[target.name], x[0], x[2], x[4], x[1], x[3],
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
