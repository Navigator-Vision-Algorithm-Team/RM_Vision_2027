#ifndef OMNIPERCEPTION__DETECTION_HPP
#define OMNIPERCEPTION__DETECTION_HPP

#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "tasks/auto_aim/armor.hpp"

namespace tools
{
class Recorder;
}

namespace omniperception
{

struct OmniCameraConfig
{
  io::Camera * camera = nullptr;
  std::shared_ptr<tools::Recorder> recorder;
  double mount_yaw = 0.0;    // degrees
  double mount_pitch = 0.0;  // degrees
  double fov_h = 54.2;       // degrees
  double fov_v = 44.5;       // degrees

  // ===========================================================================
  //  【改动】把主相机 PnP 用到全向相机所需的标定参数（对应 yaml 的 omni_*_N 键）
  //
  //  渐进式设计，缺哪一项就用近似值，保证「不标定也能跑」：
  //    · camera_matrix 空  -> 用 approximate_camera_matrix() 从 fov + 图像尺寸估算
  //    · distort_coeffs 空 -> 当作无畸变
  //    · has_extrinsic false -> 用 mount_to_R_camera2gimbal() 从安装角估算 R，
  //                             并令 t = 0（相机位置相对机器人中心的偏移）
  //    · 连 fov 都没有 -> 才回退到旧的「像素线性插值」方法（decider.cpp 兜底分支）
  // ===========================================================================
  // 注意：全向相机挂在大yaw（只有 yaw）的支架上，所以这里标的是
  //       「相机 -> 大yaw 旋转轴」，且必须带平移（光心不在转轴上）。
  //       标定值统一来自配置的 tf.frames.omni_cam_N（见 io/tf/tf_tree）。
  cv::Mat camera_matrix;   // 3x3 行优先内参（CV_64F）；空 => 用 FOV 近似
  cv::Mat distort_coeffs;  // 1x5 畸变 (k1 k2 p1 p2 k3)；空 => 无畸变
  Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();  // 相机 -> 大yaw 旋转
  Eigen::Vector3d t_camera2gimbal = Eigen::Vector3d::Zero();      // 相机 -> 大yaw 平移 (m)
  bool has_extrinsic = false;  // R_camera2gimbal / t_camera2gimbal 是否来自标定值
};

//一个识别结果可能包含多个armor,需要排序和过滤。armors, timestamp, delta_yaw, delta_pitch
struct DetectionResult
{
  std::list<auto_aim::Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
  double delta_yaw;    //rad
  double delta_pitch;  //rad

  // Assignment operator
  DetectionResult & operator=(const DetectionResult & other)
  {
    if (this != &other) {
      armors = other.armors;
      timestamp = other.timestamp;
      delta_yaw = other.delta_yaw;
      delta_pitch = other.delta_pitch;
    }
    return *this;
  }
};
}  // namespace omniperception

#endif