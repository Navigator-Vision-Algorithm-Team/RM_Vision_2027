#ifndef IO__TF__TF_TREE_HPP
#define IO__TF__TF_TREE_HPP

#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

namespace YAML
{
class Node;
}

namespace io
{
namespace tf
{

// 刚体变换：p_parent = R * p_child + t
struct Transform
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  Transform operator*(const Transform & o) const { return {R * o.R, R * o.t + t}; }
  Transform inverse() const { return {R.transpose(), -R.transpose() * t}; }
};

// -----------------------------------------------------------------------------
// 轻量 tf 树（nav2 tf2 的最小实现，够用即可）
//
// 结构与本项目的机械链一一对应：
//
//   world
//   ├── yaw_axis        (动态关节 lower_yaw：绕 z 纯 yaw，大yaw 没有 pitch/roll)
//   │   ├── omni_cam_1   (静态边：标定值 相机 -> 大yaw)
//   │   ├── omni_cam_2   ...
//   │   └── omni_cam_4
//   └── small_yaw       (动态关节 upper_pitch_roll：Rz(upper)·Ry(pitch)·Rx(roll))
//       └── main_cam     (静态边：标定值 相机 -> 小yaw)
//
// · 静态边 = 标定值，全部写在配置 tf.frames 里，代码里不出现任何硬编码几何量；
// · 动态边 = joint，每帧用 0x502 的角度刷新（set_imu）；
// · lookup(from, to) 给出 T_to_from，任意两帧之间自动沿链复合。
// -----------------------------------------------------------------------------
class Tree
{
public:
  // 关节名（也是配置文件 tf.frames.<child>.joint 的合法取值）
  static constexpr const char * LOWER_YAW = "lower_yaw";                  // Rz(lower_yaw)
  static constexpr const char * UPPER_PITCH_ROLL = "upper_pitch_roll";    // Rz(upper)·Ry(pitch)·Rx(roll)

  // 业务约定的帧名（配置里保持一致）
  static constexpr const char * WORLD = "world";
  static constexpr const char * YAW_AXIS = "yaw_axis";   // 大yaw 旋转轴（全向相机所在）
  static constexpr const char * SMALL_YAW = "small_yaw"; // 小yaw 板（主相机所在）
  static constexpr const char * MAIN_CAM = "main_cam";
  static constexpr const char * OMNI_CAM_PREFIX = "omni_cam_";

  // 静态边：child 相对 parent 的固定变换（即 p_parent = R * p_child + t）
  void add_static(
    const std::string & parent, const std::string & child, const Transform & T_parent_child);

  // 动态边：child 相对 parent 的变换由 joint 决定（纯旋转，无平移）
  void add_joint(const std::string & parent, const std::string & child, const std::string & joint);

  void set_joint(const std::string & joint, const Eigen::Matrix3d & R_parent_child);

  // 按一帧 IMU 数据刷新所有关节（弧度；upper/lower 绝对角，pitch 低头为正，roll 绕 x）
  void set_imu(double pitch, double roll, double upper_yaw, double lower_yaw);

  bool has(const std::string & frame) const;
  bool has_joint(const std::string & joint) const;
  bool empty() const { return frames_.empty(); }

  // 求 T_to_from：把 from 系下的点变到 to 系。找不到路径返回 false
  bool lookup(const std::string & from, const std::string & to, Transform & out) const;

  const std::vector<std::string> & frames() const { return frames_; }

private:
  struct Edge
  {
    std::string neighbor;
    Transform T;        // 静态边的 T_neighbor_this（动态边忽略）
    std::string joint;  // 非空 = 动态边
    bool inverse = false;  // 动态边：true 表示需要取逆（parent -> child 方向）
  };

  std::multimap<std::string, Edge> adj_;
  std::map<std::string, Eigen::Matrix3d> joints_;
  std::vector<std::string> frames_;
};

// 从 yaml 的 tf.frames 构建（写法见 configs/sentry_example.yaml）
io::tf::Tree load_tree(const YAML::Node & yaml);

// Z-Y-X（yaw-pitch-roll，度）转旋转矩阵：R = Rz(yaw)·Ry(pitch)·Rx(roll)
Eigen::Matrix3d rpy_deg_to_R(double roll_deg, double pitch_deg, double yaw_deg);

}  // namespace tf
}  // namespace io

#endif  // IO__TF__TF_TREE_HPP
