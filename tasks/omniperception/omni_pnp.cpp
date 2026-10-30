#include "omni_pnp.hpp"

#include <cmath>
#include <string>
#include <vector>

// AngleAxisd 定义在 Geometry 里；omni_pnp.hpp 只引了 <Eigen/Core>（Matrix/Vector），
// 所以这里显式补上，避免依赖 tasks/auto_aim/pnp.hpp 的传导包含。
#include <Eigen/Geometry>

#include "tasks/auto_aim/pnp.hpp"
#include "tools/logger.hpp"

namespace omniperception
{
// -----------------------------------------------------------------------------
//  数据来源（实车配置 configs/sentry_example.yaml）：
//    · 内参 K / 畸变 D <- yaml 的 omni_camera_matrix_N / omni_distort_coeffs_N
//    · 外参 R / t      <- 入口查 tf 树 tf.frames.omni_cam_N（本文件只负责接收参数）
//    · 本文件两个近似函数（approximate_camera_matrix / mount_to_R_camera2gimbal）
//      只在标定值缺失时作为兜底被 decider.cpp 调用，正常不会执行。
// -----------------------------------------------------------------------------

void load_omni_pnp_config(OmniCameraConfig & cfg, const YAML::Node & yaml, int index)
{
  const std::string suffix = std::to_string(index);
  // 小工具：判断某个键是否存在（YAML::Node 的 operator bool 是 explicit，用 static_cast）
  auto exists = [&](const std::string & name) {
    return static_cast<bool>(yaml[name + suffix]);
  };

  // ---- 内参 ----
  if (exists("omni_camera_matrix_")) {
    // 注意：这里没有 try/catch，值不是数字时会抛 YAML::TypedBadConversion（健壮性缺口）
    auto d = yaml["omni_camera_matrix_" + suffix].as<std::vector<double>>();
    if (d.size() == 9) {
      // yaml 里按行优先填写，转成 Eigen 行优先矩阵后再转 cv::Mat
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K(d.data());
      cv::eigen2cv(K, cfg.camera_matrix);
    } else {
      tools::logger()->warn(
        "[omni{}] omni_camera_matrix_{} 需要 9 个数，实际 {}，已忽略", index, index, d.size());
    }
  } else {
    tools::logger()->warn(
      "[omni{}] 未配置 omni_camera_matrix_{} -> 将用 FOV 估算内参（占位方案，建议尽快标定）",
      index, index);
  }

  // ---- 畸变 ----
  if (exists("omni_distort_coeffs_")) {
    auto d = yaml["omni_distort_coeffs_" + suffix].as<std::vector<double>>();
    if (!d.empty()) {
      // d 是局部变量，必须 clone，否则 cfg.distort_coeffs 会悬空引用
      cv::Mat D(1, static_cast<int>(d.size()), CV_64F, d.data());
      cfg.distort_coeffs = D.clone();
    }
  }
  // 未配置 => 保持空 Mat => solvePnP 视为无畸变

  // ---- 外参（R 与 t 必须成对；实车由 tf 树覆盖，这里只作兜底）----
  //
  //  日志策略（重要）：
  //    · 键【不存在】时这里不打 warn。本函数看不到 tf 树，不知道入口会不会用
  //      tf.frames.omni_cam_N 兜底（实车就是这种情况）；此时若报警
  //      "将用安装角近似"，会与实际行为矛盾，属于误导。
  //    · 键存在但【个数不对】仍然 warn：那是配置写错了，无论 tf 有没有兜底都该修。
  //    · "是否真的落到安装角近似"由调用方（sentry_multithread.cpp）在查完 tf
  //      之后统一判断并打日志，见那里的三态说明。
  //
  //  出口状态：函数返回时 cfg.has_extrinsic 恰好表示「yaml 是否提供了有效外参」，
  //            调用方直接读它即可，不需要额外的返回值。
  const bool has_R = exists("omni_R_camera2gimbal_");
  const bool has_t = exists("omni_t_camera2gimbal_");
  if (has_R && has_t) {
    auto r = yaml["omni_R_camera2gimbal_" + suffix].as<std::vector<double>>();
    auto t = yaml["omni_t_camera2gimbal_" + suffix].as<std::vector<double>>();
    if (r.size() == 9 && t.size() == 3) {
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R(r.data());
      cfg.R_camera2gimbal = R;
      cfg.t_camera2gimbal = Eigen::Vector3d(t.data());
      cfg.has_extrinsic = true;
    } else {
      tools::logger()->warn(
        "[omni{}] omni_R_camera2gimbal_{} 需要 9 个数、omni_t_camera2gimbal_{} 需要 3 个数，已忽略",
        index, index, index);
    }
  }
  // 未配置 -> cfg.has_extrinsic 保持 false，外参来源交给调用方决定
}

namespace
{
// ---- 可调参数（预留空位：调试时按实际检测效果调整）----
// 目标必须在相机前方，且距离落在这个区间内，否则认为 PnP 解不可信。
// 全向相机主要用于「发现大致方向」，宁可回退也不要发一个错误的大角度。
constexpr double MIN_TARGET_DISTANCE = 0.5;   // m
constexpr double MAX_TARGET_DISTANCE = 30.0;  // m
constexpr double MIN_CAMERA_Z = 1e-3;         // m，相机系 z 太小说明点在相机后方/镜面上
}  // namespace

//内参K未配置时用FOV估计：
cv::Mat approximate_camera_matrix(
  const cv::Size & image_size, double fov_h_deg, double fov_v_deg)
{
  // 参数不合法时返回空 Mat，调用方（decider）会据此回退到旧的像素线性近似。
  if (image_size.width <= 0 || image_size.height <= 0 || fov_h_deg <= 0 || fov_v_deg <= 0) {
    return {};
  }

  const double half_h = fov_h_deg * CV_PI / 360.0;  // 转成弧度并取一半
  const double half_v = fov_v_deg * CV_PI / 360.0;

  const double fx = (image_size.width / 2.0) / std::tan(half_h);
  const double fy = (image_size.height / 2.0) / std::tan(half_v);
  const double cx = image_size.width / 2.0;
  const double cy = image_size.height / 2.0;

  cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  return K;
}

Eigen::Matrix3d mount_to_R_camera2gimbal(double mount_yaw_deg, double mount_pitch_deg)
{
  const double yaw = mount_yaw_deg * CV_PI / 180.0;
  const double pitch = mount_pitch_deg * CV_PI / 180.0;

  // 基准姿态：光轴指向云台 +x，相机 x(右) 对齐云台 -y，相机 y(下) 对齐云台 -z
  Eigen::Matrix3d R0;
  R0 << 0, 0, 1,   //
    -1, 0, 0,      //
    0, -1, 0;

  // 先按安装 pitch 低头/抬头（绕云台 y 轴，正号 = 朝下），再按安装 yaw 转（绕云台 z 轴）
  const Eigen::Matrix3d R_z = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d R_y =
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();

  return R_z * R_y * R0;
}

OmniPnPResult solve_omni_armor(
  const auto_aim::Armor & armor, const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs,
  const Eigen::Matrix3d & R_camera2gimbal, const Eigen::Vector3d & t_camera2gimbal)
{
  OmniPnPResult result;

  // 1) 调用与主相机完全相同的 PnP 内核
  const auto pnp = auto_aim::solve_pnp_armor(armor, camera_matrix, distort_coeffs);
  if (!pnp.success) return result;

  // 2) 几何合理性检查：目标必须在相机前方（OpenCV 相机系光轴为 +z）
  if (pnp.xyz_in_camera.z() <= MIN_CAMERA_Z) return result;

  // 3) 相机系 -> 云台系
  const Eigen::Vector3d xyz_in_gimbal =
    R_camera2gimbal * pnp.xyz_in_camera + t_camera2gimbal;

  // 4) 距离检查。IPPE 对平面四点有镜像二义性，偶尔会解出一个「贴脸/无穷远」的
  //    荒唐解，这里用物理距离把它挡掉，交给调用方回退。
  const double distance = xyz_in_gimbal.norm();
  if (distance < MIN_TARGET_DISTANCE || distance > MAX_TARGET_DISTANCE) return result;

  // 5) 云台系角度：与 tools::xyz2ypd 完全相同的定义
  //      yaw   = atan2(y, x)          （向左为正）
  //      pitch = atan2(z, sqrt(x²+y²))（向上为正）
  const double yaw = std::atan2(xyz_in_gimbal.y(), xyz_in_gimbal.x());
  const double pitch = std::atan2(xyz_in_gimbal.z(), std::hypot(xyz_in_gimbal.x(), xyz_in_gimbal.y()));

  result.success = true;
  result.xyz_in_gimbal = xyz_in_gimbal;
  result.yaw_pitch_deg = Eigen::Vector2d{yaw * 180.0 / CV_PI, pitch * 180.0 / CV_PI};
  result.distance = distance;
  result.reprojection_error = pnp.reprojection_error;
  return result;
}

}  // namespace omniperception
