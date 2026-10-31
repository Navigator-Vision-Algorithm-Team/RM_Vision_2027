#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include "io/command.hpp"
#include "io/serial/serial_board.hpp"

namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}

int main(int argc, char * argv[])
{
  if (argc > 2 || (argc == 2 && (std::string(argv[1]) == "--help" ||
                                std::string(argv[1]) == "-h"))) {
    std::cout << "Usage: testgimbalspeed [config.yaml]\n"
                 "Enter absolute yaw and pitch in degrees, separated by a space.\n"
                 "Enter q to quit. Defaults to configs/soldier.yaml.\n";
    return argc > 2 ? 1 : 0;
  }

  const std::string config_path = argc == 2 ? argv[1] : "configs/soldier.yaml";
  io::SerialBoard serial_board(config_path);

  std::cout << "云台响应测试：输入绝对 yaw 和 pitch（度），空格分隔；输入 q 退出。\n";
  std::string line;
  while (std::cout << "yaw pitch (deg)> " && std::getline(std::cin, line)) {
    if (line == "q" || line == "Q") break;

    std::istringstream input(line);
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    std::string extra;
    if (!(input >> yaw_deg >> pitch_deg) || (input >> extra) || !std::isfinite(yaw_deg) ||
        !std::isfinite(pitch_deg)) {
      std::cerr << "输入无效，请输入两个有限数字（yaw pitch），或输入 q 退出。\n";
      continue;
    }

    const io::Command command{true, false, yaw_deg * kDegToRad, pitch_deg * kDegToRad};
    serial_board.send(command);
  }

  return 0;
}
