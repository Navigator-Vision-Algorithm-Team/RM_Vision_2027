#include "rm_omniperception/decider.hpp"

#include <algorithm>
#include <cmath>

namespace rm_omniperception
{

Decider::Decider()
{
  initPriorityMaps();
}

void Decider::initPriorityMaps()
{
  // mode1: prioritize infantry 3/4, then hero(1), then 5/sentry, then engineer(2)
  mode1_["3"] = 1;
  mode1_["4"] = 1;
  mode1_["1"] = 2;
  mode1_["5"] = 3;
  mode1_["sentry"] = 3;
  mode1_["2"] = 4;
  mode1_["outpost"] = 5;
  mode1_["base"] = 5;
  mode1_["guard"] = 5;
  mode1_[""] = 5;
  mode1_["negative"] = 5;

  // mode2: prioritize engineer(2), then others
  mode2_["2"] = 1;
  mode2_["1"] = 2;
  mode2_["3"] = 2;
  mode2_["4"] = 2;
  mode2_["5"] = 2;
  mode2_["sentry"] = 3;
  mode2_["outpost"] = 3;
  mode2_["base"] = 3;
  mode2_["guard"] = 3;
  mode2_[""] = 3;
  mode2_["negative"] = 3;
}

void Decider::setImageSize(int width, int height)
{
  img_width_ = width;
  img_height_ = height;
}

void Decider::setFOV(double fov_h, double fov_v)
{
  fov_h_ = fov_h;
  fov_v_ = fov_v;
}

void Decider::setEnemyColor(int color)
{
  enemy_color_ = color;  // 0=RED, 1=BLUE
}

void Decider::setPriorityMode(PriorityMode mode)
{
  mode_ = mode;
}

Eigen::Vector2d Decider::delta_angle(
  std::list<rm_auto_aim::Armor> & armors, const std::string & camera_label)
{
  if (armors.empty()) return {0.0, 0.0};

  // Find closest armor to image center
  auto closest = std::min_element(
    armors.begin(), armors.end(),
    [](const rm_auto_aim::Armor & a, const rm_auto_aim::Armor & b) {
      cv::Point2f center(320, 240);  // default center
      return cv::norm(a.center - center) < cv::norm(b.center - center);
    });

  double pixel_per_rad_h = img_width_ / fov_h_;
  double pixel_per_rad_v = img_height_ / fov_v_;

  cv::Point2f img_center(img_width_ / 2.0, img_height_ / 2.0);
  double dx = closest->center.x - img_center.x;
  double dy = closest->center.y - img_center.y;

  double delta_yaw = dx / pixel_per_rad_h;
  double delta_pitch = dy / pixel_per_rad_v;

  // Camera-specific offset
  if (camera_label == "left") {
    delta_yaw -= 1.57;    // -90 degrees (left-facing camera)
  } else if (camera_label == "right") {
    delta_yaw += 1.57;    // +90 degrees (right-facing camera)
  }
  // "back" camera: no offset

  return {delta_yaw, delta_pitch};
}

bool Decider::armor_filter(std::list<rm_auto_aim::Armor> & armors)
{
  auto expected_color = enemy_color_ == 0
    ? std::string("red") : std::string("blue");

  armors.remove_if([&](const rm_auto_aim::Armor & a) {
    // Remove wrong type (invalid)
    if (a.type == rm_auto_aim::ArmorType::INVALID) return true;

    // Remove if confidence too low
    if (a.confidence < 0.5) return true;

    // Remove invincible enemies
    for (int id : invincible_ids_) {
      try {
        int armor_id = std::stoi(a.number);
        if (armor_id == id) return true;
      } catch (...) {
        // number is not an int (e.g., "outpost", "guard")
      }
    }

    return false;
  });

  return armors.empty();
}

void Decider::set_priority(std::list<rm_auto_aim::Armor> & armors)
{
  const auto & priority_map = (mode_ == PriorityMode::MODE_ONE) ? mode1_ : mode2_;

  for (auto & armor : armors) {
    auto it = priority_map.find(armor.number);
    armor.priority = (it != priority_map.end()) ? it->second : 5;
  }
}

void Decider::sort(std::vector<DetectionResult> & detection_queue)
{
  for (auto & result : detection_queue) {
    if (!armor_filter(result.armors)) {
      set_priority(result.armors);
    }
  }

  // Sort each result's armors by priority (lower number = higher priority)
  for (auto & result : detection_queue) {
    result.armors.sort([](const rm_auto_aim::Armor & a, const rm_auto_aim::Armor & b) {
      return a.priority < b.priority;
    });
  }

  // Sort queue by highest priority armor in each result
  std::sort(detection_queue.begin(), detection_queue.end(),
    [](const DetectionResult & a, const DetectionResult & b) {
      int pa = a.armors.empty() ? 99 : a.armors.front().priority;
      int pb = b.armors.empty() ? 99 : b.armors.front().priority;
      return pa < pb;
    });
}

Eigen::Vector2d Decider::decide(const std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) return {0.0, 0.0};

  // Round-robin through cameras
  const auto & result = detection_queue[count_ % detection_queue.size()];
  count_++;

  return {result.delta_yaw, result.delta_pitch};
}

void Decider::set_invincible_armors(const std::vector<int> & invincible_ids)
{
  invincible_ids_ = invincible_ids;
}

void Decider::filter_by_target(
  std::list<rm_auto_aim::Armor> & armors, const std::vector<int> & target_ids)
{
  if (target_ids.empty()) return;

  armors.remove_if([&](const rm_auto_aim::Armor & a) {
    try {
      int armor_id = std::stoi(a.number);
      return std::find(target_ids.begin(), target_ids.end(), armor_id) == target_ids.end();
    } catch (...) {
      return true;  // remove non-numeric names
    }
  });
}

Eigen::Vector4d Decider::get_target_info(
  const std::list<rm_auto_aim::Armor> & armors, const std::string & tracked_id) const
{
  for (const auto & armor : armors) {
    if (armor.number == tracked_id) {
      int id = 0;
      try { id = std::stoi(armor.number); } catch (...) { id = 0; }
      return {armor.center.x, armor.center.y, 1.0, static_cast<double>(id + 1)};
    }
  }
  return {0.0, 0.0, 0.0, 0.0};
}

}  // namespace rm_omniperception
