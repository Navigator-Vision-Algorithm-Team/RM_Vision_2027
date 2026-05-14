#ifndef RM_OMNIPERCEPTION__DETECTION_RESULT_HPP_
#define RM_OMNIPERCEPTION__DETECTION_RESULT_HPP_

#include <chrono>
#include <list>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_omniperception
{

struct DetectionResult
{
  std::list<rm_auto_aim::Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
  double delta_yaw;    // yaw offset from camera center to target (radians)
  double delta_pitch;  // pitch offset from camera center to target (radians)

  DetectionResult()
  : delta_yaw(0.0), delta_pitch(0.0) {}

  DetectionResult(
    std::list<rm_auto_aim::Armor> _armors,
    std::chrono::steady_clock::time_point _timestamp,
    double _delta_yaw, double _delta_pitch)
  : armors(std::move(_armors)),
    timestamp(_timestamp),
    delta_yaw(_delta_yaw),
    delta_pitch(_delta_pitch) {}
};

}  // namespace rm_omniperception

#endif  // RM_OMNIPERCEPTION__DETECTION_RESULT_HPP_
