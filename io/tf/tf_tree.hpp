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
//   world                       原点 = 大yaw 轴原点（上电瞬间的车头方向为 +x）
//   ├── yaw_axis        (动态关节 lower_yaw：绕 z 纯 yaw，大yaw 没有 pitch/roll，
//   │                    原点与 world 重合，offset 为 0)
//   │   ├── omni_cam_1   (静态边：标定值 相机 -> 大yaw)
//   │   ├── omni_cam_2   ...
//   │   └── omni_cam_4
//   └── small_yaw       (动态关节 upper_pitch_roll：Rz(upper)·Ry(pitch)·Rx(roll)
//                        与 yaw_axis 同轴但不同高，靠 offset=[0,0,h] 表达；
//                        原点取枪管根部/俯仰轴)
//       └── main_cam     (静态边：标定值 相机 -> 小yaw；相机比枪管根部高一点)
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

  // 动态边：child 相对 parent 的变换 = 固定平移 offset + 关节旋转
  //   offset 是 child 原点在 parent 系下的固定位置（米）。同轴但不同高度的两条 yaw
  //   轴就靠它表达：小yaw 板位于大yaw 轴上方 h 处，h 由 offset 给出。
  void add_joint(
    const std::string & parent, const std::string & child, const std::string & joint,
    const Eigen::Vector3d & offset = Eigen::Vector3d::Zero());

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
    Transform T;           // 静态边的 T_neighbor_this（动态边忽略）
    std::string joint;     // 非空 = 动态边
    bool inverse = false;  // 动态边：true 表示需要取逆（parent -> child 方向）
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();  // 动态边：child 原点在 parent 系下的位置
  };

  std::multimap<std::string, Edge> adj_;
  std::map<std::string, Eigen::Matrix3d> joints_;
  std::vector<std::string> frames_;
};

// 从 yaml 的 tf.frames 构建（写法见 configs/sentry_example.yaml）
io::tf::Tree load_tree(const YAML::Node & yaml);

// Z-Y-X（roll-pitch-yaw，度）转旋转矩阵：R = Rz(yaw)·Ry(pitch)·Rx(roll)
// 语义：**子帧相对父帧的旋转**，直接作用在子帧坐标轴上（通用 tf 用法）
Eigen::Matrix3d rpy_deg_to_R(double roll_deg, double pitch_deg, double yaw_deg);

// 相机安装角（yaw 左正、pitch 下正、roll 绕光轴，度）转「相机 -> 父帧」旋转。
// 与 rpy_deg_to_R 的区别：相机帧是 OpenCV 约定（x 右 / y 下 / z 光轴向前），
// 所以要多乘一个基准旋转 R0，把"光轴指向"换算成"相机帧姿态"。
// 手写配置时用 mount_deg 更不容易搞错；手眼标定工具输出的 R 已经是这个结果。
Eigen::Matrix3d mount_deg_to_R(double yaw_deg, double pitch_deg, double roll_deg);

}  // namespace tf
}  // namespace io

#endif  // IO__TF__TF_TREE_HPP
