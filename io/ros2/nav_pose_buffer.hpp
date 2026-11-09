#ifndef IO__ROS2__NAV_POSE_BUFFER_HPP
#define IO__ROS2__NAV_POSE_BUFFER_HPP

// =============================================================================
//  nav_pose_buffer.hpp —— 导航 -> 视觉的位姿缓冲（方案 §5.2 / V3）
//
//  订阅 `/red_standard_robot1/aft_mapped_to_init`（nav_msgs/Odometry，
//  frame_id=camera_init，child_frame_id=body），拿到 T_camera_init<-body。
//  Point-LIO 在 publish_odometry_without_downsample=true 时是「逐点发布」，
//  频率远高于图像帧率，所以维护 500 ms 环形缓冲，按图像时间戳插值：
//      平移 lerp、旋转 slerp（Eigen::Quaterniond::slerp）
//
//  ⚠ Point-LIO 不填 pose.covariance（只写 pose），所以位姿先验只能用常数，
//    不能用协方差矩阵（§4.2）。
//
//  ⚠ 时间戳必须用图像采集时刻（io::rostime::to_ros），不能用「收到时刻」。
//
//  配置（整段缺失 = 不启用，视觉侧只发 PnP 点，degraded）：
//      nav_pose:
//        topic: "/red_standard_robot1/aft_mapped_to_init"
//        buffer_ms: 500
//        timeout_ms: 200
// =============================================================================

#include <Eigen/Geometry>
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace io
{

class NavPoseBuffer
{
public:
  // config_path 为空、或配置里没有 nav_pose 段 => enabled() == false
  explicit NavPoseBuffer(const std::string & config_path);
  ~NavPoseBuffer();

  NavPoseBuffer(const NavPoseBuffer &) = delete;
  NavPoseBuffer & operator=(const NavPoseBuffer &) = delete;

  bool enabled() const { return enabled_; }
  const std::string & topic() const { return topic_; }
  // 缓冲里最新的样本距现在是否还在 timeout 内（§5.8 的降级判据）
  bool healthy() const;

  // 按 ROS 时间戳插值出 T_camera_init<-body。返回 false 表示该时刻无可用位姿
  // （缓冲为空，或超出 timeout），调用方应计入 pose_miss 并跳过该帧。
  bool lookup(const rclcpp::Time & stamp, Eigen::Isometry3d & T_ci_body) const;

  size_t miss_count() const { return misses_.load(std::memory_order_relaxed); }
  size_t sample_count() const;

private:
  struct Sample
  {
    int64_t stamp_ns = 0;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  // T_camera_init<-body
  };

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void prune_locked();

  bool enabled_ = false;
  std::string topic_;
  double buffer_s_ = 0.5;
  double timeout_s_ = 0.2;

  mutable std::mutex mtx_;
  std::deque<Sample> buf_;

  mutable std::atomic<size_t> misses_{0};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  // 自带 executor：析构时 cancel() 就能让 spin 线程退出，
  // 不需要去动全局的 rclcpp::shutdown（那会影响同进程的其他节点）。
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spin_thread_;
  bool own_context_ = false;
};

}  // namespace io

#endif  // IO__ROS2__NAV_POSE_BUFFER_HPP
