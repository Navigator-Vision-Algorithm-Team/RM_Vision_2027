#include <cmath>
#include <iostream>
#include <stdexcept>

#include "tools/trajectory.hpp"

int main()
{
  try {
    constexpr double speed = 28.0;
    constexpr double g = 9.7833;  // Same gravity used by the production trajectory.
    for (double forward : {1.0, 3.0, 10.0}) {
      // Camera-axis target after the single camera -> execution translation.
      const double horizontal = std::hypot(forward, 0.07764);
      const tools::Trajectory compensated(speed, horizontal, 0.005);
      const tools::Trajectory at_axis_height(speed, horizontal, 0.0);
      if (compensated.unsolvable || at_axis_height.unsolvable ||
          compensated.pitch <= at_axis_height.pitch)
        throw std::runtime_error("camera above execution axis must increase elevation");
      // Aimer changes elevation-positive to actuator pitch-positive-down.
      const double command_pitch = -compensated.pitch;
      if (command_pitch >= -at_axis_height.pitch)
        throw std::runtime_error("actuator pitch correction must be negative (up)");
      const double flight_time = horizontal / (speed * std::cos(-command_pitch));
      const double height = speed * std::sin(-command_pitch) * flight_time -
        0.5 * g * flight_time * flight_time;
      if (std::abs(height - 0.005) > 1e-9)
        throw std::runtime_error("production trajectory must reach the compensated height");
      const tools::Trajectory axis_target(speed, forward, 0.0);
      if (axis_target.unsolvable || axis_target.pitch <= 0)
        throw std::runtime_error("axis target still needs gravity compensation");
      std::cout << "PASS depth=" << forward << "m: camera-axis command pitch="
                << command_pitch << " rad, reached height=" << height << "m\n";
    }
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
