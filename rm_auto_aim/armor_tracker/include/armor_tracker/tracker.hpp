#ifndef ARMOR_TRACKER__TRACKER_HPP_
#define ARMOR_TRACKER__TRACKER_HPP_

#include <Eigen/Eigen>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <memory>
#include <string>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{

enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class Tracker
{
public:
  Tracker(double max_match_distance, double max_match_yaw_diff);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armors::SharedPtr & armors_msg);
  void update(const Armors::SharedPtr & armors_msg, double dt = 0.01);

  ExtendedKalmanFilter ekf;

  int tracking_thres;
  int lost_thres;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  std::string tracked_id;
  Armor tracked_armor;
  ArmorsNum tracked_armors_num;

  double info_position_diff;
  double info_yaw_diff;

  Eigen::VectorXd measurement;
  Eigen::VectorXd target_state;

  double dz, another_r;

  // Convergence / divergence (sp_vision_25 style)
  bool is_converged() const;
  bool is_diverged() const;
  bool jumped = false;
  int update_count;

private:
  // -- EKF helpers (sp_vision_25 Target pattern) --
  void predict(double dt);
  void update_ypda(const Armor & armor, int id);

  // Compute single armor position in cartesian from state vector
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;

  // Full measurement Jacobian: d(ypd, angle)/d(state)
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

  // All armor (x,y,z,angle) from current EKF state
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  void initEKF(const Armor & a);
  void updateArmorsNum(const Armor & a);
  void handleArmorJump(const Armor & a);

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  double max_match_distance_;
  double max_match_yaw_diff_;

  int detect_count_;
  int lost_count_;

  double last_yaw_;

  // sp_vision_25 Target state tracking
  bool is_converged_ = false;
  int last_id_ = 0;
  bool is_switch_ = false;
  int switch_count_ = 0;

  // Outpost speed clamping
  static constexpr double outpost_max_v_yaw = 2.51;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_TRACKER__TRACKER_HPP_
