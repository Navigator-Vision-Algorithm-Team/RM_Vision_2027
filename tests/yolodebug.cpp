#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"
#include "tasks/auto_aim/debug_stream.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? |                      | 输出命令行参数说明}"
  "{@config-path   | configs/sentry_example.yaml | YAML 配置文件路径（支持 sentry_example.yaml / soldier.yaml）}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>(0);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }

  const auto yaml = tools::load(config_path);
  const bool sentry_config = static_cast<bool>(yaml["camera_name_main"]);
  const std::string camera_sursign = sentry_config ? "main" : "";

  auto_aim::DebugStream debug(config_path, "yolodebug", true);
  if (!debug.enabled()) {
    tools::logger()->error(
      "[yolodebug] ROS2 调试流未启动，请确认配置中 debug_ros2.enable: true 且已安装 ROS2");
    return 1;
  }

  tools::Exiter exiter;
  io::Camera camera(config_path, camera_sursign);
  auto_aim::YOLO detector(config_path, false);

  tools::logger()->info(
    "[yolodebug] 使用 {} 相机，仅运行 YOLO；ROS 话题前缀为 /sp_debug/yolodebug",
    sentry_config ? "sentry 主" : "soldier");

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) continue;

    debug.frame(img);
    debug.begin_yolo();
    const auto armors = detector.detect(img);
    debug.end_yolo();
    debug.publish(armors, auto_aim::DebugStatus{});
  }

  return 0;
}
