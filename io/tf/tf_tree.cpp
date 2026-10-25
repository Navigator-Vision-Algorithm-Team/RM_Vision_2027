#include "tf_tree.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <deque>

namespace io
{
namespace tf
{

namespace
{
constexpr double DEG = M_PI / 180.0;

Eigen::Matrix3d rot_z(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }
Eigen::Matrix3d rot_y(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix(); }
Eigen::Matrix3d rot_x(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()).toRotationMatrix(); }

// 只在动态边里用的纯旋转
Transform rotation_only(const Eigen::Matrix3d & R, bool inverse)
{
  return inverse ? Transform{R.transpose(), Eigen::Vector3d::Zero()}
                 : Transform{R, Eigen::Vector3d::Zero()};
}

void push_frame(std::vector<std::string> & frames, const std::string & frame)
{
  if (std::find(frames.begin(), frames.end(), frame) == frames.end()) frames.push_back(frame);
}
}  // namespace

Eigen::Matrix3d rpy_deg_to_R(double roll_deg, double pitch_deg, double yaw_deg)
{
  return rot_z(yaw_deg * DEG) * rot_y(pitch_deg * DEG) * rot_x(roll_deg * DEG);
}

void Tree::add_static(
  const std::string & parent, const std::string & child, const Transform & T_parent_child)
{
  push_frame(frames_, parent);
  push_frame(frames_, child);

  // 从 parent 出发走到 child：T_child_parent = T_parent_child^{-1}
  adj_.emplace(parent, Edge{child, T_parent_child.inverse(), "", false});
  adj_.emplace(child, Edge{parent, T_parent_child, "", false});
}

void Tree::add_joint(const std::string & parent, const std::string & child, const std::string & joint)
{
  push_frame(frames_, parent);
  push_frame(frames_, child);

  // parent -> child 方向要取逆；child -> parent 直接用
  adj_.emplace(parent, Edge{child, Transform{}, joint, true});
  adj_.emplace(child, Edge{parent, Transform{}, joint, false});
  joints_.emplace(joint, Eigen::Matrix3d::Identity());
}

void Tree::set_joint(const std::string & joint, const Eigen::Matrix3d & R_parent_child)
{
  auto it = joints_.find(joint);
  if (it != joints_.end()) it->second = R_parent_child;
}

void Tree::set_imu(double pitch, double roll, double upper_yaw, double lower_yaw)
{
  set_joint(LOWER_YAW, rot_z(lower_yaw));
  set_joint(UPPER_PITCH_ROLL, rot_z(upper_yaw) * rot_y(pitch) * rot_x(roll));
}

bool Tree::has(const std::string & frame) const
{
  return std::find(frames_.begin(), frames_.end(), frame) != frames_.end();
}

bool Tree::has_joint(const std::string & joint) const { return joints_.count(joint) > 0; }

bool Tree::lookup(const std::string & from, const std::string & to, Transform & out) const
{
  if (from == to) {
    out = Transform{};
    return true;
  }
  if (!has(from) || !has(to)) return false;

  // 树很小，广度优先即可：seen[node] = T_node_from
  std::map<std::string, Transform> seen;
  std::deque<std::string> queue;
  seen[from] = Transform{};
  queue.push_back(from);

  while (!queue.empty()) {
    const std::string cur = queue.front();
    queue.pop_front();
    const Transform T_cur_from = seen[cur];

    const auto range = adj_.equal_range(cur);
    for (auto it = range.first; it != range.second; ++it) {
      const Edge & edge = it->second;
      if (seen.count(edge.neighbor)) continue;

      Transform T_nb_cur = edge.T;  // T_neighbor_cur
      if (!edge.joint.empty()) {
        auto j = joints_.find(edge.joint);
        const Eigen::Matrix3d & R = (j == joints_.end()) ? Eigen::Matrix3d::Identity() : j->second;
        T_nb_cur = rotation_only(R, edge.inverse);
      }

      const Transform T_nb_from = T_nb_cur * T_cur_from;
      seen[edge.neighbor] = T_nb_from;

      if (edge.neighbor == to) {
        out = T_nb_from;
        return true;
      }
      queue.push_back(edge.neighbor);
    }
  }
  return false;
}

Tree load_tree(const YAML::Node & yaml)
{
  Tree tree;
  if (!yaml["tf"]) return tree;

  const YAML::Node frames = yaml["tf"]["frames"];
  if (!frames || !frames.IsMap()) return tree;

  for (const auto & kv : frames) {
    const std::string child = kv.first.as<std::string>();
    const YAML::Node f = kv.second;

    const std::string parent = f["parent"] ? f["parent"].as<std::string>() : std::string("world");

    // 动态关节
    if (f["joint"]) {
      tree.add_joint(parent, child, f["joint"].as<std::string>());
      continue;
    }

    // 没填数值的静态边视为「尚未标定」，直接跳过：
    // 配置里保留结构说明，运行时回退到各模块自己的近似/旧键，不会把标定值覆盖成单位阵。
    const bool has_rotation = f["R"] || f["rpy_deg"];
    const bool has_translation = f["t"] || f["xyz"];
    if (!has_rotation && !has_translation) continue;

    // 静态标定值：优先 R(9) + t(3)，其次 xyz + rpy_deg
    Transform T;
    if (f["R"] && f["R"].size() == 9) {
      for (int i = 0; i < 9; ++i) T.R(i / 3, i % 3) = f["R"][i].as<double>();
    } else if (f["rpy_deg"] && f["rpy_deg"].size() == 3) {
      T.R = rpy_deg_to_R(
        f["rpy_deg"][0].as<double>(), f["rpy_deg"][1].as<double>(), f["rpy_deg"][2].as<double>());
    }

    if (f["t"] && f["t"].size() == 3) {
      for (int i = 0; i < 3; ++i) T.t[i] = f["t"][i].as<double>();
    } else if (f["xyz"] && f["xyz"].size() == 3) {
      for (int i = 0; i < 3; ++i) T.t[i] = f["xyz"][i].as<double>();
    }

    tree.add_static(parent, child, T);
  }

  return tree;
}

}  // namespace tf
}  // namespace io
