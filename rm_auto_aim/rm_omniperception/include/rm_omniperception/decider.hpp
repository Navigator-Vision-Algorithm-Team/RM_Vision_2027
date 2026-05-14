#ifndef RM_OMNIPERCEPTION__DECIDER_HPP_
#define RM_OMNIPERCEPTION__DECIDER_HPP_

#include <Eigen/Dense>
#include <list>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "armor_detector/armor.hpp"
#include "rm_omniperception/detection_result.hpp"

namespace rm_omniperception
{

using ArmorName = std::string;
using PriorityMap = std::unordered_map<ArmorName, int>;

enum class PriorityMode
{
  MODE_ONE = 1,
  MODE_TWO = 2
};

class Decider
{
public:
  Decider();

  void setImageSize(int width, int height);
  void setFOV(double fov_h, double fov_v);
  void setEnemyColor(int color);  // 0=RED, 1=BLUE
  void setPriorityMode(PriorityMode mode);

  // Compute pixel-to-angle offsets for the closest armor.
  // camera_label: "left", "right", or "back" for different offset constants.
  Eigen::Vector2d delta_angle(
    std::list<rm_auto_aim::Armor> & armors, const std::string & camera_label);

  // Filter armors: remove wrong color, invincible armors, and invalid types.
  // Returns true if filtered list is empty.
  bool armor_filter(std::list<rm_auto_aim::Armor> & armors);

  // Assign priority to each armor based on current priority mode.
  void set_priority(std::list<rm_auto_aim::Armor> & armors);

  // Full sort pipeline: filter + priority-sort each DetectionResult, then sort results.
  void sort(std::vector<DetectionResult> & detection_queue);

  // Fallback decision: cycle through omni-camera results.
  // Returns (yaw_offset, pitch_offset) for gimbal to turn toward target.
  Eigen::Vector2d decide(const std::vector<DetectionResult> & detection_queue);

  // Update invincible enemy list (from referee system).
  void set_invincible_armors(const std::vector<int> & invincible_ids);

  // Filter to keep only specified target armor names.
  void filter_by_target(std::list<rm_auto_aim::Armor> & armors,
                        const std::vector<int> & target_ids);

  // Get target info as [x, y, flag, id] for nav publishing.
  Eigen::Vector4d get_target_info(
    const std::list<rm_auto_aim::Armor> & armors,
    const std::string & tracked_id) const;

private:
  int img_width_ = 640;
  int img_height_ = 480;
  double fov_h_ = 1.047;  // 60 degrees
  double fov_v_ = 0.785;  // 45 degrees
  int enemy_color_ = 0;   // 0=RED, 1=BLUE
  PriorityMode mode_ = PriorityMode::MODE_ONE;
  int count_ = 0;         // round-robin counter

  std::vector<int> invincible_ids_;

  PriorityMap mode1_;
  PriorityMap mode2_;

  void initPriorityMaps();
};

}  // namespace rm_omniperception

#endif  // RM_OMNIPERCEPTION__DECIDER_HPP_
