#ifndef VISUAL_DEPTH__VISUAL_DEPTH_HPP
#define VISUAL_DEPTH__VISUAL_DEPTH_HPP

// =============================================================================
//  visual_depth.hpp —— 视觉补盲总装（方案 §6 V7：主循环接入点）
//
//  把「已算好的 PnP 点」和「左后相机的连续帧」汇总成导航要的三条出口：
//
//      PnP 3D 点（大yaw 系） ──┐
//                              ├─> C3 vision/obstacle_cloud  10 Hz（出口 A/B，比赛运行）
//      特征跟踪 + 三角化/BA ───┴─> C4 vision/map_points       1 Hz（出口 C，仅建图运行）
//                              └─> C5 vision/status           1 Hz
//
//  为什么要有这一层（而不是把逻辑塞进 main）：
//    · main 只需要「喂数据 + 收尾」两个调用，绑核推理线程完全不受影响；
//    · 算法（跟踪/三角化/地面投影/PCD）与 ROS（发布/订阅）解耦，
//      没有 ROS2 的时候整段退化为空实现，soldier 等目标照常编译。
//
//  三条红线在这里再次确认：
//    1) 动态点绝不进 IVox / 不参与位姿：本模块只发 terrain_map，不碰导航定位；
//    2) 不在 YOLO 推理线程里发布：本模块只在主循环（main）里被调用；
//    3) 不加第二套 TF 发布者：只发点云 + frame_id。
//
//  使用顺序（每个主循环帧）：
//      depth.add_obstacle_points(pts, stamp);       // 有几路就调几次，内部取最新 stamp
//      depth.add_frame(cam_id, img, stamp, R, t);   // 只有三角化那一路会真正处理
//      depth.finish(main_stamp, pose_lookup);       // 发布 + 三角化 + 统计
// =============================================================================

#include <Eigen/Geometry>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "tasks/auto_aim/multithread/yolo_pipeline.hpp"

namespace visual_depth
{

class VisualDepth
{
public:
  // 配置里没有视觉补盲相关段落 => enabled() == false，所有调用都是空操作
  explicit VisualDepth(const std::string & config_path);
  ~VisualDepth();

  VisualDepth(const VisualDepth &) = delete;
  VisualDepth & operator=(const VisualDepth &) = delete;

  bool enabled() const;
  // 三角化是否开启（比赛配置为 false：只发 PnP 点，把 CPU 留给自瞄）
  bool triangulation_enabled() const;
  // 需要保留原图的那一路相机（0 基 CameraResult.camera_id）；-1 = 都不保留
  int keep_image_camera_id() const;

  // 汇总本帧 PnP 点（大yaw 系，x 前 y 左 z 上）。stamp 为这批点所在图像的采集时刻。
  // 没有目标时也要调用（传空 vector），保证话题上持续有空点云。
  void add_obstacle_points(
    const std::vector<auto_aim::multithread::ObstaclePoint> & points,
    std::chrono::steady_clock::time_point stamp);

  // 三角化相机的一帧原图。R_cam2yaw / t_cam2yaw 是「相机 -> 大yaw」的静态外参
  // （来自 tf 树 tf.frames.omni_cam_N）。
  void add_frame(
    int camera_id, const cv::Mat & color, std::chrono::steady_clock::time_point stamp,
    const Eigen::Matrix3d & R_cam2yaw, const Eigen::Vector3d & t_cam2yaw);

  // 图像 steady 时间戳 -> T_camera_init<-body（导航位姿）。
  // 返回 false 表示该时刻没有可用位姿，三角化通道本帧跳过并计入 pose_miss。
  using PoseLookup = std::function<bool(std::chrono::steady_clock::time_point, Eigen::Isometry3d &)>;

  // 一帧收尾：发 C3/C4/C5，必要时做三角化。必须每帧调用一次（哪怕什么都没检测到）。
  void finish(std::chrono::steady_clock::time_point main_stamp, const PoseLookup & pose_lookup);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace visual_depth

#endif  // VISUAL_DEPTH__VISUAL_DEPTH_HPP
