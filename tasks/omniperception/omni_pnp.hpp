#ifndef OMNIPERCEPTION__OMNI_PNP_HPP
#define OMNIPERCEPTION__OMNI_PNP_HPP

// =============================================================================
//  omni_pnp.hpp —— 把主相机的 PnP 解算搬到全向相机上（全向侧胶水代码）
//
//  分工：
//    · tasks/auto_aim/pnp.*      ：与相机无关的公共内核（模型点 + solvePnP）
//    · tasks/omniperception/omni_pnp.*：本文件，全向相机特有的三件事
//        1) 解出「装甲板中心在云台系（大yaw 系）下的 yaw / pitch / 距离」
//        2) 兜底：没有标定内参时，用 FOV + 图像尺寸估一个近似内参
//        3) 兜底：没有实测外参时，用安装角 mount_yaw/mount_pitch 构造 R_camera2gimbal（平移置 0）
//
//  【当前标定状态】实车配置 configs/sentry_example.yaml 里：
//    · 4 台全向相机的内参 K 与畸变 D 已标定（omni_camera_matrix_N /
//      omni_distort_coeffs_N，由 calibration/calibrate_intrinsics 产出）；
//    · 4 台全向相机的外参 R/t 由 tf 树的 tf.frames.omni_cam_N 实测给出，
//      入口 sentry_multithread.cpp 查表后覆盖 cfg.R_camera2gimbal / t_camera2gimbal。
//    因此正常运行（实车配置）时 2)、3) 两条兜底分支都不会触发，保留它们只为：
//      · 配置漏填 / 键名写错（内参缺键会打 warn；外参缺键由入口按三态打日志）
//      · tf 里查不到 omni_cam_N（帧名或条目缺失）
//      · 仿真配置（sentry_virtual.yaml 的 omni_* 键全部注释）
//    调试时看日志里的 "(calibrated)" / "(approx-FOV)" 即可确认当前走的哪条路。
//
//  与旧做法的区别：
//    旧代码 Decider::delta_angle 用「像素到角度」的线性插值：
//        yaw = mount_yaw + fov_h/2 - center_norm.x * fov_h
//    这在视场边缘误差很大（真实关系是 arctan，不是线性），
//    现在改为：solvePnP 得到相机系 3D 点 -> 外参转到云台系 -> atan2 求角度。
//
//  【术语】下文所说的「云台系 / gimbal 系」= 本工程的 IMU 参考系 = **大yaw 系**：
//         大yaw 是下面那根轴，只有 yaw；四个全向相机挂在它的支架上，
//         主相机与 IMU 在它上面的小yaw 板上（小yaw 带 yaw + pitch + roll）。
//         详见下面 mount_to_R_camera2gimbal 处的「大yaw + 小yaw」说明。
// =============================================================================

// 本头只用到 Eigen 的 Core（Matrix3d / Vector2d / Vector3d）；
// AngleAxisd 在 Geometry 里，由 omni_pnp.cpp 自己包含，不要指望这里。
#include <Eigen/Core>  // 必须在 opencv2/core/eigen.hpp 上面
#include <opencv2/core/eigen.hpp>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include "detection.hpp"
#include "tasks/auto_aim/armor.hpp"

namespace omniperception
{

// -----------------------------------------------------------------------------
//  从 yaml 读取第 index 个全向相机的 PnP 标定参数（键名形如 omni_camera_matrix_<index>）
//
//  可识别的键（全部可选，缺省时保持 cfg 现有值，由 decider 用近似值兜底）：
//    omni_camera_matrix_N    : 9 个数，3x3 行优先内参        ← 实车配置已填标定值
//    omni_distort_coeffs_N   : 5 个数，k1 k2 p1 p2 k3        ← 实车配置已填标定值
//    omni_R_camera2gimbal_N  : 9 个数，3x3 行优先，相机 -> 云台 旋转  ← 实车配置里已注释
//    omni_t_camera2gimbal_N  : 3 个数，相机 -> 云台 平移（m）          ← 实车配置里已注释
//
//  外参优先级：入口调用完本函数后还会查一次 tf 树（omni_cam_N -> yaw_axis），
//  查到就覆盖 cfg.R_camera2gimbal / t_camera2gimbal 并置 has_extrinsic。
//  所以实车的外参来源是 tf 树，上面两个 omni_R/t_camera2gimbal_N 键只是
//  「tf 里没有这台相机」时的兜底。
//
//  R 与 t 必须成对出现才会把 cfg.has_extrinsic 置 true。
//  参数不合法（个数不对）时会打 warn 并保持原值，不会抛异常。
//
//  【日志约定】键缺失时本函数故意【不】打 warn——因为此刻还不知道调用方会不会
//  用 tf 树兜底（实车就是这种情况），贸然报警会与实际行为矛盾。"是否真的落到
//  安装角近似"由入口（sentry_multithread.cpp）查完 tf 之后按三态统一输出。
//  函数返回时 cfg.has_extrinsic 等价于「yaml 是否提供了有效外参」。
// -----------------------------------------------------------------------------
void load_omni_pnp_config(OmniCameraConfig & cfg, const YAML::Node & yaml, int index);

// -----------------------------------------------------------------------------
//  兜底方案 1：用 FOV 估计相机内参
//
//  ⚠ 仅当 cfg.camera_matrix 为空时才会被 decider.cpp 调用（实车配置已有标定值，
//    正常不会走到这里）。用途：全向相机还没做棋盘格标定、或仿真配置没填内参时，
//    用配置里的 fov_h/fov_v 和当前帧尺寸反推出一个「理想针孔相机」内参：
//      fx = (w/2) / tan(fov_h/2),  fy = (h/2) / tan(fov_v/2)
//      cx = w/2,                   cy = h/2
//  它忽略了镜头畸变，对广角镜头只是近似；但方向是 arctan 正确的，
//  比旧的线性插值好得多。拿到标定结果后请在 yaml 里填 omni_camera_matrix_N，
//  代码会自动优先使用标定值（见 decider.cpp）。
//
//  注意：fov 必须是「全画幅」的水平/垂直视场角，单位度。
// -----------------------------------------------------------------------------
cv::Mat approximate_camera_matrix(
  const cv::Size & image_size, double fov_h_deg, double fov_v_deg);

// -----------------------------------------------------------------------------
//  兜底方案 2：用安装角构造 R_camera2gimbal
//
//  ⚠ 仅当 !cfg.has_extrinsic（yaml 没填 omni_R/t_camera2gimbal_N，且 tf 树里
//    查不到 omni_cam_N）时才会被 decider.cpp 调用，此时平移 t 被置 0。
//    实车走的是 tf 树里实测的 R/t，本函数只是「一台都没配」时的近似。
//
//  与 io/tf/tf_tree.cpp 的 mount_deg 约定一致（同一套 R0 + Rz·Ry·Rx 基变换），
//  改动任一处请同步另一处。
//
//  坐标约定（与主相机 R_camera2gimbal 的约定保持一致，整份代码统一）：
//    · 云台系：x 向前，y 向左，z 向上（与 tools::xyz2ypd / 世界系一致）
//    · 相机系：OpenCV 约定，x 向右，y 向下，z 为光轴向前
//
//  基准姿态（mount_yaw = mount_pitch = 0）：光轴指向云台正前方，于是
//      R0 = [ 0, 0, 1;
//            -1, 0, 0;
//             0,-1, 0 ]
//  即：相机 z(前) -> 云台 x(前)，相机 x(右) -> 云台 -y(右)，相机 y(下) -> 云台 -z(下)
//
//  再叠加安装角：
//      R = Rz(mount_yaw) * Ry(+mount_pitch) * R0
//  其中 mount_yaw 为正表示相机被装成「朝向云台左侧」；
//  mount_pitch 为正表示相机「朝下低头」。后者是为了与配置里原有的语义一致：
//  旧 delta_angle 把 mount_pitch 直接加在「向下为正」的 pitch 命令上
//  （见 main 里的 Aimer：command.pitch = -(世界系抬头为正的 pitch)）。
//  如果你在标定时习惯用「抬头为正」，请在这里取负号。
//
//  ⚠ 关于哨兵的「大yaw + 小yaw」结构（重要，别搞反）：
//    · 大yaw 是下面那根轴，**只有 yaw**；四个全向相机挂在它的支架上，
//      主相机与 IMU 在它上面的小yaw 板上（小yaw 带 yaw + pitch + roll）。
//    · 因此全向相机的外参应标成「相机 -> 大yaw」，并包含平移（光心不在转轴上）。
//      这些标定值统一放在配置的 tf.frames.omni_cam_N，由 io/tf/tf_tree 查表得到。
//    · 于是 xyz_world = Rz(lower_yaw) * xyz_bigyaw 严格成立，全向链路与 pitch/roll
//      无关（大yaw 不会因为小yaw 俯仰而俯仰）。
//    · 必须遵守的耦合：本模块返回的角度是「大yaw 系」，入口叠加的必须是
//      大yaw 的世界系 yaw（= 0x502 的 lower_yaw），**不能用** Solver 的
//      R_gimbal2world（那是小yaw 板的姿态）。
//    · 主相机在小yaw 板上，直接用 Rz(upper)·Ry(pitch)·Rx(roll) 作为基座姿态，
//      外参标成「相机 -> 小yaw」，不需要任何附加修正角。
//
//  标定结果写到 tf.frames.omni_cam_N 的 xyz + mount_deg（实车已填，见
//  configs/sentry_example.yaml）；老的 omni_R_camera2gimbal_N / omni_t_camera2gimbal_N
//  键仍保留，但只在 tf 里查不到这台相机时才作为兜底生效。
// -----------------------------------------------------------------------------
Eigen::Matrix3d mount_to_R_camera2gimbal(double mount_yaw_deg, double mount_pitch_deg);

// 全向相机一次 PnP 解算的结果
struct OmniPnPResult
{
  bool success = false;

  // 装甲板中心在云台系下的坐标（m）。x 前、y 左、z 上。
  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();  //必要零初始化

  // 相对云台正前方的角度（度），采用「数学/世界系」约定：
  //   yaw   向左为正（与 tools::xyz2ypd 一致，也与 command.yaw 一致）
  //   pitch 向上为正（与 tools::xyz2ypd 一致）
  // ⚠ 注意：下位机的 command.pitch 是「向下为正」（见 aimer.cpp 里的取负），
  //   所以 Decider::delta_angle 会在返回前把 pitch 取负，转换成命令约定。
  Eigen::Vector2d yaw_pitch_deg = Eigen::Vector2d::Zero();

  double distance = 0.0;                 // 到装甲板的直线距离（m）
  double reprojection_error = 0.0;       // 重投影误差（px），可用于过滤误检
};

// -----------------------------------------------------------------------------
//  全向相机 PnP 主入口：相机系 3D 点 -> 云台系角度
//
//  参数：
//    armor            检测到的装甲板（需要 armor.type 与 armor.points）
//    camera_matrix    该全向相机的内参（实车为标定值；只有兜底时才用
//                     approximate_camera_matrix 的近似值）
//    distort_coeffs   该全向相机的畸变（可为空 Mat => 无畸变）
//    R_camera2gimbal  相机 -> 大yaw 旋转（实车由入口查 tf 树 tf.frames.omni_cam_N 得到）
//    t_camera2gimbal  相机 -> 大yaw 平移（m，同上；走兜底时为 0）
//
//  内部会做几何合理性检查（目标必须在相机前方、距离在合理范围内），
//  不通过就返回 success = false，由调用方决定是否回退到旧的像素线性近似。
// -----------------------------------------------------------------------------
OmniPnPResult solve_omni_armor(
  const auto_aim::Armor & armor, const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs,
  const Eigen::Matrix3d & R_camera2gimbal, const Eigen::Vector3d & t_camera2gimbal);

}  // namespace omniperception

#endif  // OMNIPERCEPTION__OMNI_PNP_HPP
