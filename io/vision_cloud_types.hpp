#ifndef IO__VISION_CLOUD_TYPES_HPP
#define IO__VISION_CLOUD_TYPES_HPP

// 视觉 -> 导航的点云点类型（方案 §5.3 / §5.4）。
// 专门拆成不依赖 ROS 的头：视觉算法侧（tasks/visual_depth）在没有 ROS2 的环境里
// 也要能编译，只有真正发布时才需要 io/ros2/publish2nav_pointcloud.hpp。

#include <Eigen/Core>

namespace io
{

// C3：实时障碍点（x,y,z,intensity，16 B/点）
struct ObstacleCloudPoint
{
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float intensity = 1.f;
};

// C4：建图点（PointXYZINormal 布局，48 B/点）
//   质量字段的含义见方案 §5.4：离线工具按 normal_x / normal_z 阈值筛点。
struct MapCloudPoint
{
  Eigen::Vector3f p = Eigen::Vector3f::Zero();
  float quality = 0.f;       // normal_x：1 / 重投影误差均方根（越大越好）
  int observations = 0;      // normal_y：观测次数 N
  float parallax_deg = 0.f;  // normal_z：三角化视差角（度）
  float consistency = 1.f;   // curvature：「一致性得分」= 1/(1+多窗口位置离散度)
};

}  // namespace io

#endif  // IO__VISION_CLOUD_TYPES_HPP
