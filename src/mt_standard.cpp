#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
// #include "io/dm_imu/dm_imu.hpp"  // [CAN] 已切换为串口通信
#include "io/serial_board.hpp"
#include "tasks/auto_aim/aimer.hpp"
// #include "tasks/auto_aim/multithread/commandgener.hpp"  // [CAN] CommandGener 依赖 CBoard
// #include "tasks/auto_aim/multithread/mt_detector.hpp"   // [CAN] 多线程检测器
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
// [CAN] 打符模块，串口协议暂不支持模式切换
// #include "tasks/auto_buff/buff_aimer.hpp"
// #include "tasks/auto_buff/buff_detector.hpp"
// #include "tasks/auto_buff/buff_solver.hpp"
// #include "tasks/auto_buff/buff_target.hpp"
// #include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Camera camera(config_path);

  // [CAN] 已切换为串口通信
  // io::CBoard cboard(config_path);

  io::SerialBoard serial_board(config_path);

  // [CAN] 多线程检测器，已改为单线程 YOLO
  // auto_aim::multithread::MultiThreadDetector detector(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  // [CAN] 打符模块，串口协议暂不支持模式切换
  // auto_buff::Buff_Detector buff_detector(config_path);
  // auto_buff::Solver buff_solver(config_path);
  // auto_buff::SmallTarget buff_small_target;
  // auto_buff::BigTarget buff_big_target;
  // auto_buff::Aimer buff_aimer(config_path);

  // [CAN] CommandGener 依赖 CBoard，串口版在内联完成
  // auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = serial_board.imu_at(t - 1ms);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

    auto command = aimer.aim(targets, t, serial_board.bullet_speed);

    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    serial_board.send(command);
  }

  return 0;
}