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
  //  图像方向修正（与 io::Camera::rotation() 一致）
  //    90 = 顺时针、-90 = 逆时针、0 = 未旋转
  //
  //  io::Camera 已经按安装方位把全向图像转正了，而 yaml 里的
  //  omni_camera_matrix_N / omni_distort_coeffs_N 是在「未旋转」的原始图像上
  //  标定的。decider 会按这个值把主点与焦距变换到旋转后的坐标系，否则 PnP
  //  解出的 yaw/pitch 会整体偏 90°，且 fov_h/fov_v 也要交换后再用。
  // ===========================================================================
  int rotate = 0;


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
  std::chrono::steady_clock::time_point timestamp{};
  double delta_yaw = 0.0;    //rad
  double delta_pitch = 0.0;  //rad

  // 不要手写拷贝赋值：只要显式声明了拷贝赋值，编译器就不会再隐式生成移动构造/移动赋值，
  // 于是 std::vector<DetectionResult>::push_back(std::move(x)) 会退化成深拷贝 list<Armor>。
  // 成员全部可拷贝可移动，用默认生成版本即可。
  DetectionResult() = default;
  DetectionResult(
    std::list<auto_aim::Armor> armors_, std::chrono::steady_clock::time_point timestamp_,
    double delta_yaw_, double delta_pitch_)
  : armors(std::move(armors_)), timestamp(timestamp_), delta_yaw(delta_yaw_),
    delta_pitch(delta_pitch_)
  {
  }
};
}  // namespace omniperception

#endif