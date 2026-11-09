#ifndef IO__ROS2__PUBLISH2NAV_POINTCLOUD_HPP
#define IO__ROS2__PUBLISH2NAV_POINTCLOUD_HPP

// =============================================================================
//  publish2nav_pointcloud.hpp —— 视觉 -> 导航的点云发布（方案 §5.3 / §5.4 / §5.5，V4）
//
//  三条出口：
//    C3 `vision/obstacle_cloud` sensor_msgs/PointCloud2  10 Hz  BEST_EFFORT
//        frame_id=gimbal_yaw，512 点上限，x/y/z/intensity 四个 FLOAT32（16 B/点）
//        => 导航 terrain_analysis 并入 terrain_map，同时喂 costmap(出口 A) 与
//           pointcloud_to_laserscan(出口 B)
//    C4 `vision/map_points`    sensor_msgs/PointCloud2   1 Hz  RELIABLE
//        frame_id=camera_init，PointXYZINormal 布局（48 B/点），带质量字段供离线筛选
//        => 出口 C（离线 PCD），比赛运行时 publish_hz_map: 0 关掉
//    C5 `vision/status`        std_msgs/String(JSON)     1 Hz
//
//  三条红线在这里落地：
//    · 独立工作线程 + 「最新帧槽位」：主循环 submit() 只投递不阻塞，
//      已有 pending 就整批替换（旧帧没有使用价值），发布/序列化全在线程里做；
//    · 手工填 PointCloud2，不引 PCL：只需要 4 个 float 字段，为它拖进
//      pcl_common / pcl_conversions 不值得；
//    · 只发点云 + frame_id，绝不发 TF（gimbal_yaw / front_mid360 由
//      导航侧 robot_state_publisher 统一发布）。
//
//  节流在这里做（不是主循环）：主循环可能 300 Hz 投递，本模块按
//  publish_hz_obstacle / publish_hz_map 取最新一批发布。
//
//  配置（整段缺失 = 三条出口全关，行为与旧版本完全一致）：
//      obstacle_cloud:
//        enable: true
//        topic_obstacle: "/red_standard_robot1/vision/obstacle_cloud"
//        topic_map: "/red_standard_robot1/vision/map_points"
//        topic_status: "/red_standard_robot1/vision/status"
//        frame_id_body: "gimbal_yaw"
//        frame_id_world: "camera_init"
//        publish_hz_obstacle: 10
//        publish_hz_map: 0
//        max_points_obstacle: 512
//        max_points_map: 20000
//        max_range_obstacle: 5.0
//        max_range_map: 12.0
//        intensity: 1.0
// =============================================================================

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "io/vision_cloud_types.hpp"

namespace io
{

class Publish2NavPointCloud
{
public:
  // 配置里没有 obstacle_cloud 段、或 enable=false => 全部关闭
  explicit Publish2NavPointCloud(const std::string & config_path);
  ~Publish2NavPointCloud();

  Publish2NavPointCloud(const Publish2NavPointCloud &) = delete;
  Publish2NavPointCloud & operator=(const Publish2NavPointCloud &) = delete;

  // 任意一条出口开启即为 true
  bool enabled() const;
  bool obstacle_enabled() const;
  bool map_enabled() const;
  bool status_enabled() const;

  // 非阻塞投递：只保留最新一批（旧批直接丢弃）。空 vector 也会投递 =>
  // 无目标时话题上发 width=0 的空点云，绝不停发（导航侧靠这一点判断链路活着）。
  // 距离过滤 / 点数上限 / intensity 覆盖在本模块内完成。
  void submit_obstacles(std::vector<ObstacleCloudPoint> points, const rclcpp::Time & stamp);
  void submit_map(std::vector<MapCloudPoint> points, const rclcpp::Time & stamp);
  void submit_status(std::string json);

  // 实测发布频率（Hz，滑动统计），供 status 上报
  double obstacle_hz() const;
  size_t obstacle_sent() const;
  size_t map_sent() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace io

#endif  // IO__ROS2__PUBLISH2NAV_POINTCLOUD_HPP
