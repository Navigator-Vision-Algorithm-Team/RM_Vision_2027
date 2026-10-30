// =============================================================================
//  calibrate_extrinsic.cpp —— 相机外参标定（相机 -> tf 树父帧）
//
//  ---------------------------------------------------------------------------
//  一、这个程序输出什么
//  ---------------------------------------------------------------------------
//  输出的是 tf 树里的 **静态边**，也就是 5 个相机各自的
//
//      父帧：全向相机 = yaw_axis（大yaw 旋转轴）
//            主相机   = small_yaw（小yaw 板，主相机与枪管所在）
//
//      内容：R（3x3，相机系 -> 父帧）+ t（3，相机光心在父帧下的坐标，米）
//
//  本工具不会碰动态关节（Rz(lower)、Rz(upper)Ry(pitch)Rx(roll)），那些由 0x502 给。
//
//  ---------------------------------------------------------------------------
//  二、两种标法，先用哪种？
//  ---------------------------------------------------------------------------
//  ① 物理测量法（--mode manual）：拿卷尺/角尺量"相机光心在哪、朝哪"，直接填数。
//       优点：不需要转云台、不需要棋盘、5 分钟一台。
//       缺点：精度取决于手艺，角度误差 1° 就会让 5 m 外的目标偏 ~9 cm。
//       适用：先跑通、做初值、或者结构方正容易量的相机（全向相机装在标准支架上时）。
//
//  ② 算法标定法（--mode handeye）：转云台 + 固定棋盘，用手眼标定解出来。
//       优点：平移（臂长）精度高，角度不受手工测量影响。
//       缺点：要采集、要转，主相机还要求 pitch/roll 归零。
//       适用：正式标定，尤其是"光心不在转轴上"带来的视差必须算准的全向相机。
//
//  推荐流程：先 ① 出一个能跑的结果 → 再用 ② 标精 → 用 ②的结果覆盖 ①。
//  两者输出格式完全一样，都是能直接粘进配置的 tf 片段。
//
//  ---------------------------------------------------------------------------
//  三、先做内参！顺序不能反
//  ---------------------------------------------------------------------------
//  外参解算依赖 PnP，PnP 依赖内参。请先跑 calibration/calibrate_intrinsics.cpp，
//  把 camera_matrix / omni_camera_matrix_N 填进配置，再来标外参。
//  本工具会从配置里读内参，读不到会直接报错。
//
//  ---------------------------------------------------------------------------
//  四、物理测量法要怎么量（--mode manual）
//  ---------------------------------------------------------------------------
//  1) 先确定"父帧原点"：
//       全向相机（yaw_axis）：大yaw 旋转轴与小yaw 旋转轴的交点。
//       主相机（small_yaw） ：小yaw 板的俯仰/偏航轴交点（同轴时就是上电时主轴上的点）。
//       实操：在机械上贴一个标记点，之后所有测量都从这个点量。
//
//  2) 坐标系方向（务必对齐）：
//        x = 车头正前方
//        y = 向左为正
//        z = 向上为正（右手系）
//
//  3) 量三个距离（米）：相机光心（镜头成像中心，不是外壳）在父帧下的 x / y / z。
//       技巧：用卷尺量"标记点到相机前表面的距离"再退回镜片厚度；全向相机通常左右对称，
//       y 可以取 ±(左右相机间距/2)。
//
//  4) 量三个角度（度）：
//        yaw   = 光轴水平偏角，**向左数为正**（光轴指向车头左侧 10° 就填 10）
//        pitch = 光轴俯仰，**向下数为正**（低头 15° 就填 15）
//        roll  = 绕光轴的滚转，一般填 0（只有相机歪装才需要，小角度近似）
//      这些角度描述的是"相机光轴相对父帧 +x 轴"的朝向，与
//      tasks/omniperception/omni_pnp.cpp 里 mount_to_R_camera2gimbal() 完全同一套约定。
//
//  5) 精度预期：x/y/z 量到 5 mm、角度量到 1° 算合格。
//
//  ---------------------------------------------------------------------------
//  五、算法标定法怎么做（--mode handeye）
//  ---------------------------------------------------------------------------
//  原理一句话：靶固定不动，云台转到多个角度，同一个靶在相机里的位姿变化
//  只可能由"相机相对转轴的固定位姿 X"解释，于是可以解 AX = XB。
//  （推导见 calibration/README.md §4）
//
//  采集要求：
//   · 棋盘固定在地上/桌上，采集期间不许动；棋盘尺寸与内置角点见 --cols/--rows。
//   · 云台转角**上限由相机 FOV 决定**：相机跟着云台一起转，云台转 θ，靶在画面里
//     也扫过 θ。所以  转角上限 ≈ 水平 FOV − 棋盘角尺寸 − 余量。
//         全向相机（54°）   实际可用 ≈ 35~40°，取 10~12 个姿态（相邻 3~5°）
//         主相机（33°）     实际可用 ≈ 20~25°，取 8~10 个姿态（相邻 2~3°）
//     "相邻 ≥15°、总覆盖 ≥120°"是本结构做不到的，不要照抄。要的是**把可用范围用满**。
//   · 距离越近视差越大（|t|/D）：在不出画的前提下尽量把靶放近（1.5~2.5 m）。
//   · **单向逼近**（每次从同一方向转到目标角再停，躲齿轮间隙），停稳 200 ms 再拍。
//   · 全向相机转 **大yaw**（--joint lower，默认）；主相机转 **小yaw**（--joint upper）。
//   · 主相机标定时大yaw 不要动，**pitch/roll 必须归零**（否则父帧不是纯 yaw）。
//   · 串口配置必须是真机（serial_mode: real），否则角度是假的，标出来全是错的。
//   · 标定期间**底盘不能动**（不能推、不能转、导航/底盘跟随要停掉）：A_i 的平移项
//     写死为 0，靠的就是"转轴在世界里不动"这个前提。
//
//  ⚠ 单轴标定有两个**数学上不可观**的自由度（不是 bug，是几何决定的）：
//
//     ① t 的 z 分量（沿转轴方向的高度）
//        —— X 和 Y 同时沿 z 平移任意距离，观测完全不变。
//        对策：用卷尺量光心高度，--tz-mm 填进去。
//
//     ② 相机绕转轴的**方位角**（往左还是往右看）
//        —— X 和 Y 同时绕 z 转任意角度，观测也完全不变。
//        对策：用 --yaw-hint 给出安装方位角（度，向左为正）。
//              全向相机默认读配置里的 omni_mount_yaw_N，主相机默认 0。
//
//     重点：② 的误差**不会被一致性指标发现**（实测：先验错 5°，散布依然接近 0）。
//     也就是说方位角是"你给它多少它就信多少"，必须靠机械测量或设计图纸给准。
//     一般 ±2° 的安装角估读，对 5 m 目标约 3~5 cm 的横向误差。
//
//  ---------------------------------------------------------------------------
//  六、命令示例
//  ---------------------------------------------------------------------------
//  (a) 物理测量法（会逐项提示你输入，回车用默认值）：
//
//        ./build/calibrate_extrinsic --mode manual --camera omni1
//
//      也可以直接给参数（免交互）：
//
//        ./build/calibrate_extrinsic --mode manual --camera omni1 \
//              --xyz 0.12,-0.05,0.03 --ypr-deg 90,0,0
//
//  (b) 算法标定（全向相机，转大yaw）：
//
//        ./build/calibrate_extrinsic configs/sentry.yaml --mode handeye \
//              --camera omni1 --cols 8 --rows 12 --square-mm 30 \
//              --yaw-hint 90 --tz-mm 55
//        （--yaw-hint = 相机安装方位角，左正；--tz-mm = 量出来的光心高度）
//
//  (c) 算法标定（主相机，转小yaw，先让 pitch/roll 归零）：
//
//        ./build/calibrate_extrinsic configs/sentry.yaml --mode handeye \
//              --camera main --joint upper --cols 8 --rows 12 --square-mm 30 \
//              --yaw-hint 0 --tz-mm 120
//
//  (d) 采完想换算法/剔除坏点重算，不用再转云台：
//
//        ./build/calibrate_extrinsic --mode handeye --camera omni1 \
//              --from-samples calibration/out/omni1/samples.csv
//
//  窗口按键（handeye 模式）：
//        空格 = 拍一张（只有检测到棋盘才存）
//        c    = 解算并输出
//        d    = 删除上一张
//        q    = 退出
//
//  ---------------------------------------------------------------------------
//  七、结果怎么看
//  ---------------------------------------------------------------------------
//  程序会打印：
//    · R / t 数值；
//    · 可直接粘贴的 tf 片段；
//    · **一致性指标**：把每个姿态反推回"靶在父帧里的位姿 Y_i"，理论上所有 Y_i 应该
//      完全一样。程序会打印它们的角度散布（度）和平移散布（mm）：
//         角度散布 < 0.5°、平移散布 < 10 mm  → 数据可信
//         明显更大 → 有姿态角度记错、棋盘没固定、或云台有回差，剔掉坏点重来
//    · 结果写入 calibration/out/<相机>/tf_extrinsic.yaml，采集数据写入 samples.csv。
//
//  拿到结果后：粘进 configs/xxx.yaml 的 tf.frames 里对应相机（把注释掉的
//  xyz/rpy_deg 换成 R/t），重启程序，日志里会出现 "[Main] main_cam extrinsic <- tf tree"
//  与 "tf frames: ..."。
//
//  ---------------------------------------------------------------------------
//  八、常见问题
//  ---------------------------------------------------------------------------
//  · "读不到内参"：先跑 calibrate_intrinsics，把 K/D 填进配置。
//  · "角度散布很大"：
//      ① 转角太小（<10° 的相对运动解不稳定）；
//      ② 棋盘动了；
//      ③ 主相机标定时 pitch/roll 没归零；
//      ④ 用了 serial_mode: fake（角度是假的）。
//  · "t 的 z 分量看起来不对"：单轴旋转的固有奇异性——沿轴方向（高度）不可观，
//      用 --tz-mm 填机械测量值。详见 calibration/README.md §7。
//  · "结果的角度看起来偏了 90°"：多半是 --yaw-hint 给错。数据本身分辨不出方位角，
//      它完全由这个先验决定（侧视相机必须给 ±90 之类，不能默认 0）。
//  · OpenCV 的 park/tsai 方法在本项目数据上会出 NaN：它们假设有多个旋转轴，
//      单轴时秩缺。默认的 --method axis 是本项目专用的解析解，不要换。
//  · 全向相机画面里棋盘太小 → PnP 精度差 → 把板放近一点或换大板。
// =============================================================================

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "io/serial/serial_board.hpp"
#include "tools/yaml.hpp"

namespace
{

using namespace std::chrono_literals;

constexpr double DEG = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

// -----------------------------------------------------------------------------
//  基础工具
// -----------------------------------------------------------------------------

Eigen::Matrix3d rot_z(double a)
{
  return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
Eigen::Matrix3d rot_y(double a)
{
  return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix();
}
Eigen::Matrix3d rot_x(double a)
{
  return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

// 四元数/矩阵 -> Eigen 4x4
Eigen::Matrix4d homogeneous(const Eigen::Matrix3d & R, const Eigen::Vector3d & t)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

cv::Mat to_cv_mat(const Eigen::Matrix3d & R)
{
  cv::Mat m(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) m.at<double>(r, c) = R(r, c);
  return m;
}

std::string join_list(const cv::Mat & m, const char * fmt = "%.10g")
{
  std::string out = "[";
  const int total = static_cast<int>(m.total());
  const double * data = reinterpret_cast<const double *>(m.data);
  for (int i = 0; i < total; ++i) {
    if (i) out += ", ";
    out += cv::format(fmt, data[i]);
  }
  return out + "]";
}

// 与 tasks/omniperception/omni_pnp.cpp::mount_to_R_camera2gimbal() 同一约定：
//   基准姿态 = 光轴指向父帧 +x，相机 x(右) -> 父帧 -y，相机 y(下) -> 父帧 -z
//   然后绕父帧 z 转 yaw（向左为正）、绕父帧 y 转 pitch（向下为正）
Eigen::Matrix3d mount_to_R(double yaw_deg, double pitch_deg, double roll_deg)
{
  Eigen::Matrix3d R0;
  R0 << 0, 0, 1,  //
    -1, 0, 0,     //
    0, -1, 0;
  return rot_z(yaw_deg * DEG) * rot_y(pitch_deg * DEG) * rot_x(roll_deg * DEG) * R0;
}

// --camera 的后缀 -> tf 帧名（运行时是按这个名字 lookup 的）
//   main -> main_cam ；omni3 -> omni_cam_3
std::string frame_name_of(const std::string & camera)
{
  if (camera.rfind("omni", 0) == 0) return "omni_cam_" + camera.substr(4);
  if (camera == "main") return "main_cam";
  return camera;
}

std::string parent_of(const std::string & camera)
{
  return camera.rfind("omni", 0) == 0 ? "yaw_axis" : "small_yaw";
}

std::string key_of(const std::string & camera, const std::string & name)
{
  // main -> camera_matrix ；omni3 -> omni_camera_matrix_3 / omni_distort_coeffs_3
  if (camera.rfind("omni", 0) == 0) return "omni_" + name + "_" + camera.substr(4);
  return name;
}

// 打印并写盘：tf 片段
void emit_tf(
  const std::string & camera, const std::string & parent, const Eigen::Matrix3d & R,
  const Eigen::Vector3d & t, const std::string & out_dir)
{
  const std::string R_list = join_list(to_cv_mat(R));
  const std::string t_list =
    cv::format("[%.6f, %.6f, %.6f]", t.x(), t.y(), t.z());

  std::cout << "\n--------------------- 粘贴到 configs 的 tf.frames ---------------------\n";
  std::cout << "    " << camera << ":\n"
            << "      parent: " << parent << "\n"
            << "      R: " << R_list << "\n"
            << "      t: " << t_list << "\n";
  std::cout << "----------------------------------------------------------------------\n";
  std::cout << "（同目录的 tf_extrinsic.yaml 里也有同样内容，可整段并入配置）\n";

  std::filesystem::create_directories(out_dir);
  std::ofstream ofs(std::filesystem::path(out_dir) / "tf_extrinsic.yaml");
  ofs << "tf:\n  frames:\n"
      << "    " << camera << ":\n"
      << "      parent: " << parent << "\n"
      << "      R: " << R_list << "\n"
      << "      t: " << t_list << "\n";
  ofs.close();
  std::cout << "已写入: " << (std::filesystem::path(out_dir) / "tf_extrinsic.yaml").string()
            << "\n";
}

// -----------------------------------------------------------------------------
//  物理测量法
// -----------------------------------------------------------------------------

bool run_manual(const std::string & camera, std::string xyz, std::string ypr, std::string out_dir)
{
  const std::string parent = parent_of(camera);

  auto split3 = [](std::string s, double v[3]) -> bool {
    std::replace(s.begin(), s.end(), '/', ',');
    std::replace(s.begin(), s.end(), ' ', ',');
    std::stringstream ss(s);
    std::string item;
    int n = 0;
    while (std::getline(ss, item, ',')) {
      if (item.empty()) continue;
      if (n >= 3) return false;
      v[n++] = std::stod(item);
    }
    return n == 3;
  };

  double p[3] = {0, 0, 0}, a[3] = {0, 0, 0};

  if (xyz.empty()) {
    std::cout << "\n=== 物理测量法：相机 " << camera << "（父帧 " << parent << "）===\n"
              << "坐标约定：x 向前、y 向左、z 向上（米），原点 = 旋转轴交点。\n\n";
  }
  if (!xyz.empty() && !split3(xyz, p)) {
    std::cout << "[error] --xyz 需要 3 个数，例如 --xyz 0.12,-0.05,0.03\n";
    return false;
  }
  if (xyz.empty()) {
    std::cout << "光心在父帧下的 x（米，向前为正）: ";
    std::cin >> p[0];
    std::cout << "光心在父帧下的 y（米，向左为正）: ";
    std::cin >> p[1];
    std::cout << "光心在父帧下的 z（米，向上为正）: ";
    std::cin >> p[2];
  }

  if (!ypr.empty() && !split3(ypr, a)) {
    std::cout << "[error] --ypr-deg 需要 3 个数，例如 --ypr-deg 90,0,0\n";
    return false;
  }
  if (ypr.empty()) {
    std::cout << "\n光轴水平偏角 yaw（度，向左为正）: ";
    std::cin >> a[0];
    std::cout << "光轴俯仰 pitch（度，向下为正）: ";
    std::cin >> a[1];
    std::cout << "绕光轴 roll（度，一般 0）: ";
    std::cin >> a[2];
  }

  const Eigen::Matrix3d R = mount_to_R(a[0], a[1], a[2]);
  const Eigen::Vector3d t(p[0], p[1], p[2]);

  std::cout << "\n[manual] yaw=" << a[0] << "° pitch=" << a[1] << "° roll=" << a[2]
            << "°  xyz=(" << t.x() << ", " << t.y() << ", " << t.z() << ") m\n";

  // 直接输出"人输入的形态"（xyz + mount_deg），并附等价的 R/t 便于核对
  const std::string R_list = join_list(to_cv_mat(R));
  const std::string t_list = cv::format("[%.6f, %.6f, %.6f]", t.x(), t.y(), t.z());

  std::cout << "\n--------------------- 粘贴到 configs 的 tf.frames ---------------------\n";
  std::cout << "    " << frame_name_of(camera) << ":\n"
            << "      parent: " << parent << "\n"
            << cv::format("      xyz: [%.6f, %.6f, %.6f]\n", t.x(), t.y(), t.z())
            << cv::format("      mount_deg: [%.4f, %.4f, %.4f]   # [yaw, pitch, roll]\n", a[0], a[1], a[2])
            << "      # 等价的矩阵写法（二选一，R/t 优先级更高）\n"
            << "      # R: " << R_list << "\n"
            << "      # t: " << t_list << "\n";
  std::cout << "----------------------------------------------------------------------\n";

  const std::string cam_out = std::filesystem::path(out_dir).append(camera).string();
  std::filesystem::create_directories(cam_out);
  std::ofstream ofs(std::filesystem::path(cam_out) / "tf_extrinsic.yaml");
  ofs << "tf:\n  frames:\n"
      << "    " << frame_name_of(camera) << ":\n"
      << "      parent: " << parent << "\n"
      << cv::format("      xyz: [%.6f, %.6f, %.6f]\n", t.x(), t.y(), t.z())
      << cv::format("      mount_deg: [%.4f, %.4f, %.4f]\n", a[0], a[1], a[2]);
  ofs.close();
  std::cout << "已写入: " << (std::filesystem::path(cam_out) / "tf_extrinsic.yaml").string()
            << "\n";
  std::cout << "提示：物理量精度有限，标完建议用 §6 的 L2/L3 验证。\n";
  return true;
}

// -----------------------------------------------------------------------------
//  算法标定法（手眼 AX = XB）
// -----------------------------------------------------------------------------

struct Sample
{
  double angle = 0.0;  // 关节角（rad）
  Eigen::Matrix3d R_target2cam = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_target2cam = Eigen::Vector3d::Zero();
};

std::vector<cv::Point3f> board_object_points(int cols, int rows, double square)
{
  std::vector<cv::Point3f> obj;
  for (int y = 0; y < rows; ++y)
    for (int x = 0; x < cols; ++x)
      obj.emplace_back(static_cast<float>(x * square), static_cast<float>(y * square), 0.0F);
  return obj;
}

bool find_board(const cv::Mat & gray, const cv::Size & pattern, std::vector<cv::Point2f> & corners)
{
  corners.clear();
  const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  if (!cv::findChessboardCorners(gray, pattern, corners, flags)) return false;
  cv::cornerSubPix(
    gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));
  return true;
}

// 读配置里的内参（必须先跑内参标定）
bool load_intrinsics(const std::string & config_path, const std::string & camera, cv::Mat & K, cv::Mat & D)
{
  auto yaml = tools::load(config_path);
  const std::string kK = key_of(camera, "camera_matrix");
  const std::string kD = key_of(camera, "distort_coeffs");
  if (!yaml[kK]) {
    std::cout << "[error] 配置里没有 " << kK
              << "，请先用 calibrate_intrinsics 标内参并填入配置。\n";
    return false;
  }
  const auto kv = yaml[kK].as<std::vector<double>>();
  const auto dv = yaml[kD] ? yaml[kD].as<std::vector<double>>() : std::vector<double>{};
  if (kv.size() != 9) {
    std::cout << "[error] " << kK << " 需要 9 个数\n";
    return false;
  }
  K = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) K.at<double>(i / 3, i % 3) = kv[i];
  D = cv::Mat(1, static_cast<int>(dv.size()), CV_64F);
  for (size_t i = 0; i < dv.size(); ++i) D.at<double>(0, static_cast<int>(i)) = dv[i];
  return true;
}

// -----------------------------------------------------------------------------
//  单轴（纯 yaw）手眼解析解
//
//  我们的云台只有一个转轴，A_i = Rz(θ_i)、平移为 0。此时 AX=XB 退化：
//    · OpenCV 的 calibrateHandEye 在这种数据上会给出 NaN（内部假设有多个旋转轴）；
//    · 沿转轴方向（z）的平移本来就不可观。
//  这里按单轴结构直接解：
//    旋转： Rz(Δ_i)·R_x = R_x·C_i ，C_i = (R_0^B)^T·R_i^B
//           ⇒ R_x 必须把 C_i 的转轴映到 z；剩下的绕 z 自由度用一维搜索定。
//    平移： (Rz(θ_i) − Rz(θ_0))·t_x = R_y·(t_i^B − t_0^B)
//           左矩阵秩为 2（零空间 = z），所以 t_z 由外部给定（机械测量/0），
//           其余两个分量用最小二乘解。
// -----------------------------------------------------------------------------
bool solve_axis_handeye(
  const std::vector<Sample> & samples, double tz_m, double yaw_hint_rad, Eigen::Matrix3d & R_x,
  Eigen::Vector3d & t_x, std::string & why)
{
  if (samples.size() < 3) {
    why = "样本太少（至少 3 组，推荐 8 组以上）";
    return false;
  }

  const Eigen::Matrix3d B0 = samples[0].R_target2cam;
  const Eigen::Vector3d b0 = samples[0].t_target2cam;
  const double th0 = samples[0].angle;

  // 相对关系：Rz(Δ_i)·R_x = R_x·C_i ，其中 Δ_i = θ_i − θ_0，C_i = B_0·B_iᵀ
  auto C_of = [&](size_t i) { return B0 * samples[i].R_target2cam.transpose(); };
  auto dtheta_of = [&](size_t i) { return samples[i].angle - th0; };

  // 1) 基准旋转：把 C_i 的转轴映到 z（取转角最大的样本，数值最稳）
  int best = -1;
  double best_angle = 0.0;
  for (size_t i = 1; i < samples.size(); ++i) {
    const Eigen::AngleAxisd aa(C_of(i));
    if (std::abs(aa.angle()) > best_angle) {
      best_angle = std::abs(aa.angle());
      best = static_cast<int>(i);
    }
  }
  if (best < 0 || best_angle < 5.0 * DEG) {
    why = "相机看到的旋转太小（相邻角度差 < 5°），多转几个角度";
    return false;
  }

  const Eigen::AngleAxisd aa_best(C_of(static_cast<size_t>(best)));
  const Eigen::Vector3d n_best = aa_best.axis().normalized();
  const Eigen::Matrix3d R_map =
    Eigen::Quaterniond::FromTwoVectors(n_best, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // 2) 剩下的绕 z 自由度 = 相机绕转轴的方位角，数据上不可观（与靶的朝向耦合），
  //    用外部先验（安装角测量值）固定：光轴方位角 = yaw_hint。
  const Eigen::Vector3d axis_map = R_map * Eigen::Vector3d::UnitZ();
  const double az_map = std::atan2(axis_map.y(), axis_map.x());
  const double gamma = std::remainder(yaw_hint_rad - az_map, 2.0 * M_PI);
  R_x = rot_z(gamma) * R_map;

  const Eigen::Matrix3d R_y = rot_z(th0) * R_x * B0;  // Y = A_0 · X · B_0

  // 3) 平移：Rz(θ_i)·t_x − t_y = −Rz(θ_i)·R_x·t_i
  //    未知量 [t_x.x, t_x.y, t_y.x, t_y.y, t_y.z]，t_x.z 由机械测量给定（不可观）
  const size_t n = samples.size();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * n, 5);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(3 * n);
  for (size_t i = 0; i < n; ++i) {
    const Eigen::Matrix3d Rz_i = rot_z(samples[i].angle);
    const Eigen::Vector3d d = -Rz_i * R_x * samples[i].t_target2cam;

    A.block<3, 2>(3 * i, 0) = Rz_i.block<3, 2>(0, 0);      // t_x.x, t_x.y
    A.block<3, 3>(3 * i, 2) = -Eigen::Matrix3d::Identity();  // t_y
    rhs.segment<3>(3 * i) = d - Rz_i.col(2) * tz_m;          // 移项：t_x.z 已知
  }

  const Eigen::VectorXd sol = A.colPivHouseholderQr().solve(rhs);
  if (!sol.allFinite()) {
    why = "平移最小二乘失败（数据可能自相矛盾）";
    return false;
  }
  t_x = Eigen::Vector3d(sol(0), sol(1), tz_m);
  (void)b0;
  return true;
}

// 解算 + 一致性检查 + 输出
bool solve_and_report(
  std::vector<Sample> & samples, const std::string & camera, const std::string & joint,
  const std::string & method_name, const std::string & out_dir, double tz_measured_mm,
  bool tz_given, double yaw_hint_deg)
{
  if (samples.size() < 5) {
    std::cout << "[error] 至少需要 5 组数据（推荐 8~12 组），当前 " << samples.size() << "\n";
    return false;
  }

  std::vector<cv::Mat> R_g2b, t_g2b, R_t2c, t_t2c;
  for (const auto & s : samples) {
    // 父帧相对世界：只有 yaw（世界原点定在转轴线上，所以平移为 0）
    R_g2b.push_back(to_cv_mat(rot_z(s.angle)));
    t_g2b.push_back(cv::Mat::zeros(3, 1, CV_64F));
    R_t2c.push_back(to_cv_mat(s.R_target2cam));
    t_t2c.push_back((cv::Mat_<double>(3, 1) << s.t_target2cam.x(), s.t_target2cam.y(),
                     s.t_target2cam.z()));
  }

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  if (method_name == "axis") {
    // 默认：按本项目"纯 yaw 关节"的结构直接解析求解（单轴，OpenCV 会退化）
    std::string why;
    if (!solve_axis_handeye(
          samples, tz_given ? tz_measured_mm / 1000.0 : 0.0, yaw_hint_deg * DEG, R, t, why)) {
      std::cout << "[error] 单轴解算失败：" << why << "\n";
      return false;
    }
  } else {
    cv::HandEyeCalibrationMethod method = cv::CALIB_HAND_EYE_PARK;
    if (method_name == "tsai")
      method = cv::CALIB_HAND_EYE_TSAI;
    else if (method_name == "horaud")
      method = cv::CALIB_HAND_EYE_HORAUD;
    else if (method_name == "andreff")
      method = cv::CALIB_HAND_EYE_ANDREFF;
    else if (method_name == "daniilidis")
      method = cv::CALIB_HAND_EYE_DANIILIDIS;

    cv::Matx33d R_x;
    cv::Vec3d t_x;
    try {
      cv::calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, R_x, t_x, method);
    } catch (const cv::Exception & e) {
      std::cout << "[error] calibrateHandEye 失败：" << e.what() << "\n";
      return false;
    }
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) R(r, c) = R_x(r, c);
    t = Eigen::Vector3d(t_x[0], t_x[1], t_x[2]);
    if (!R.allFinite() || !t.allFinite()) {
      std::cout << "[error] " << method_name << " 在单轴数据上退化（结果为 NaN）。\n"
                << "        请改用默认的 --method axis（本项目专用解析解）。\n";
      return false;
    }
  }

  // ---- 一致性检查：Y_i = A_i * X * B_i 应当恒定 -------------------------
  std::vector<Eigen::Matrix4d> Y;
  Y.reserve(samples.size());
  for (const auto & s : samples) {
    const Eigen::Matrix4d A = homogeneous(rot_z(s.angle), Eigen::Vector3d::Zero());
    const Eigen::Matrix4d B = homogeneous(s.R_target2cam, s.t_target2cam);
    Y.push_back(A * homogeneous(R, t) * B);
  }
  double max_rot = 0.0, max_trans = 0.0;
  for (size_t i = 1; i < Y.size(); ++i) {
    const Eigen::Matrix3d dR = Y[0].block<3, 3>(0, 0).transpose() * Y[i].block<3, 3>(0, 0);
    const double ang = std::acos(std::clamp((dR.trace() - 1.0) * 0.5, -1.0, 1.0)) * RAD2DEG;
    const double dt = (Y[i].block<3, 1>(0, 3) - Y[0].block<3, 1>(0, 3)).norm();
    max_rot = std::max(max_rot, ang);
    max_trans = std::max(max_trans, dt);
  }

  std::cout << "\n==================== 外参结果 (" << camera << ") ====================\n";
  std::cout << "关节        : " << joint << "（父帧 = " << parent_of(camera) << "）\n";
  std::cout << "样本数      : " << samples.size() << "，求解方法 " << method_name << "\n";
  std::cout << "R =\n" << R << "\n";
  std::cout << "t = " << t.transpose() << " m   |t_xy| = " << std::hypot(t.x(), t.y())
            << " m\n";
  std::cout << "一致性      : 角度散布 " << max_rot << "°，平移散布 " << max_trans * 1000.0
            << " mm" << (max_rot < 0.5 && max_trans < 0.01 ? "   (可信)" : "   (偏大，检查数据)") << "\n";

  std::cout << "\n⚠ 单轴旋转的固有奇异性：沿转轴方向（z，高度）的平移不可观，"
               "上式的 t.z 只反映初值与噪声。\n";
  if (tz_given) {
    std::cout << "  已按机械测量覆盖: t.z = " << tz_measured_mm / 1000.0 << " m\n";
    t.z() = tz_measured_mm / 1000.0;
  } else {
    std::cout << "  建议用卷尺量出光心高度，然后用 --tz-mm <毫米> 重跑一次覆盖它。\n";
  }

  emit_tf(
    frame_name_of(camera), parent_of(camera), R, t,
    std::filesystem::path(out_dir).append(camera).string());
  return true;
}

bool read_samples_csv(const std::string & path, std::vector<Sample> & samples)
{
  std::ifstream ifs(path);
  if (!ifs) {
    std::cout << "[error] 打不开 " << path << "\n";
    return false;
  }
  std::string line;
  std::getline(ifs, line);  // 表头
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string item;
    std::vector<double> v;
    while (std::getline(ss, item, ',')) v.push_back(std::stod(item));
    if (v.size() < 7) continue;  // angle, rvec(3), tvec(3)
    Sample s;
    s.angle = v[0];
    // 存的是 Rodrigues 向量，转回矩阵
    const cv::Vec3d rvec(v[1], v[2], v[3]);
    cv::Matx33d Rm;
    cv::Rodrigues(rvec, Rm);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) s.R_target2cam(r, c) = Rm(r, c);
    s.t_target2cam = Eigen::Vector3d(v[4], v[5], v[6]);
    samples.push_back(s);
  }
  std::cout << "[from-samples] 读入 " << samples.size() << " 组数据\n";
  return !samples.empty();
}

bool write_samples_csv(const std::string & path, const std::vector<Sample> & samples)
{
  std::ofstream ofs(path);
  if (!ofs) return false;
  ofs << "angle_rad,rvx,rvy,rvz,tx,ty,tz\n";
  for (const auto & s : samples) {
    const cv::Mat R = to_cv_mat(s.R_target2cam);
    const Eigen::Vector3d t = s.t_target2cam;
    // 矩阵 -> Rodrigues
    cv::Matx33d Rm;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) Rm(r, c) = R.at<double>(r, c);
    cv::Vec3d rvec;
    cv::Rodrigues(Rm, rvec);
    ofs << cv::format(
      "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", s.angle, rvec[0], rvec[1], rvec[2], t.x() + 0.0,
      t.y() + 0.0, t.z() + 0.0);
  }
  return true;
}

bool run_handeye(
  const std::string & config_path, const std::string & camera, const std::string & joint,
  int cols, int rows, double square_mm, const std::string & method_name,
  const std::string & out_dir, const std::string & from_samples, double tz_mm, bool tz_given,
  double yaw_hint_deg)
{
  const std::string cam_out = std::filesystem::path(out_dir).append(camera).string();
  std::vector<Sample> samples;

  if (!from_samples.empty()) {
    if (!read_samples_csv(from_samples, samples)) return false;
    return solve_and_report(
      samples, camera, joint, method_name, out_dir, tz_mm, tz_given, yaw_hint_deg);
  }

  cv::Mat K, D;
  if (!load_intrinsics(config_path, camera, K, D)) return false;

  io::Camera cam(config_path, camera);
  io::SerialBoard board(config_path);  // 提供关节角（必须 real，否则角度是假的）

  // 防呆：假下位机的角度是正弦生成的，标出来必然是错的
  {
    auto yaml = tools::load(config_path);
    const std::string mode =
      yaml["serial_mode"] ? yaml["serial_mode"].as<std::string>() : std::string("unset");
    if (mode != "real") {
      std::cout << "\n[warn] serial_mode = " << mode
                << "，关节角不是真机数据，标定结果无效！\n"
                   "       请把配置里的 serial_mode 改成 real 再标外参。\n\n";
    }
  }

  const cv::Size pattern(cols, rows);
  const auto obj = board_object_points(cols, rows, square_mm);
  const std::string title = "calibrate_extrinsic - " + camera;
  std::cout << "\n空格=拍照  c=解算  d=删上一张  q=退出   棋盘 " << cols << "x" << rows << "，"
            << square_mm << " mm\n";

  cv::Mat img;
  std::chrono::steady_clock::time_point ts;
  for (int i = 0; i < 10; ++i) cam.read(img, ts);

  while (true) {
    cam.read(img, ts);
    if (img.empty()) continue;

    // 与图像同一时刻的关节角（不要用"现在最新"的值）
    const io::ImuState imu = board.imu_state_at(ts - 1ms);
    const double angle = (joint == "upper") ? imu.upper_yaw : imu.lower_yaw;

    cv::Mat gray, vis;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> corners;
    const bool found = find_board(gray, pattern, corners);

    cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);
    if (found) cv::drawChessboardCorners(vis, pattern, corners, found);

    const double angle_deg = angle * RAD2DEG;
    cv::putText(
      vis, cv::format("%s | %s | %s=%.2f deg | %zu samples", camera.c_str(),
                      found ? "BOARD OK" : "no board", joint.c_str(), angle_deg,
                      samples.size()),
      cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.65,
      found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
    cv::putText(
      vis, "SPACE=capture  c=solve  d=undo  q=quit", cv::Point(12, 56),
      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);

    cv::Mat show = vis;
    if (vis.cols > 1400) {
      const double s = 1400.0 / vis.cols;
      cv::resize(vis, show, cv::Size(), s, s);
    }
    cv::imshow(title, show);

    const int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;

    if (key == ' ') {
      if (!found) {
        std::cout << "[info] 没检测到棋盘，未保存\n";
        continue;
      }
      cv::Vec3d rvec, tvec;
      cv::solvePnP(obj, corners, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

      Sample s;
      s.angle = angle;
      cv::Matx33d Rm;
      cv::Rodrigues(rvec, Rm);
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) s.R_target2cam(r, c) = Rm(r, c);
      s.t_target2cam = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);
      samples.push_back(s);
      std::cout << "[capture] " << samples.size() << " 组，关节角 " << angle_deg << "°\n";
    }

    if (key == 'd') {
      if (!samples.empty()) {
        samples.pop_back();
        std::cout << "[undo] 已删除，当前 " << samples.size() << " 组\n";
      }
    }

    if (key == 'c') {
      if (solve_and_report(
            samples, camera, joint, method_name, out_dir, tz_mm, tz_given, yaw_hint_deg)) {
        std::filesystem::create_directories(cam_out);
        const auto csv = std::filesystem::path(cam_out) / "samples.csv";
        if (write_samples_csv(csv.string(), samples)) {
          std::cout << "采集数据已保存: " << csv.string()
                    << "（可用 --from-samples 离线重算）\n";
        }
        std::cout << "按任意键退出...\n";
        cv::waitKey(0);
        break;
      }
    }
  }

  cv::destroyWindow(title);
  return true;
}

void print_help()
{
  std::cout
    << "用法：\n"
    << "  calibrate_extrinsic --mode manual  --camera <main|omni1..4> [--xyz x,y,z] [--ypr-deg y,p,r]\n"
    << "  calibrate_extrinsic <配置> --mode handeye --camera <main|omni1..4> [选项]\n"
    << "  calibrate_extrinsic --mode handeye --camera <...> --from-samples <csv> [选项]\n\n"
    << "选项（--key value 与 --key=value 都支持）：\n"
    << "  --mode <manual|handeye>  标定方式（必填）\n"
    << "  --camera <name>          相机：main / omni1 / omni2 / omni3 / omni4\n"
    << "  --xyz <x,y,z>            物理法：光心在父帧下的坐标（米）\n"
    << "  --ypr-deg <y,p,r>        物理法：光轴 yaw(左正) / pitch(下正) / roll（度）\n"
    << "  --cols/--rows <n>        内角点行列（默认 8 / 12；9×13 的板子是 8×12）\n"
    << "  --square-mm <x>          格长（毫米，默认 30）\n"
    << "  --joint <lower|upper>    转哪个轴（默认：全向=lower，主相机=upper）\n"
    << "  --method <axis|park|tsai|horaud|andreff|daniilidis>\n"
    << "                           手眼解算方法：默认 axis = 本项目单轴 yaw 的解析解\n"
    << "                           （OpenCV 的 park/tsai 在单轴数据上会退化出 NaN）\n"
    << "  --tz-mm <毫米>           用机械测量的光心高度覆盖 t.z（单轴不可观）\n"
    << "  --yaw-hint <度>          相机安装方位角（向左为正）；单轴数据无法分辨方位角，\n"
    << "                           全向相机默认取配置里的 omni_mount_yaw_N\n"
    << "  --from-samples <csv>     离线重算（读 samples.csv，不连相机）\n"
    << "  --out <dir>              输出目录（默认 calibration/out）\n"
    << "  -h, --help               打印本帮助\n";
}

struct Args
{
  std::string config;
  std::string mode;
  std::string camera = "main";
  std::string xyz;
  std::string ypr;
  std::string joint;  // 空 = 按相机自动选
  std::string method = "axis";
  std::string out = "calibration/out";
  std::string from_samples;
  int cols = 8;
  int rows = 12;
  double square_mm = 30.0;
  double tz_mm = 0.0;
  bool tz_given = false;
  double yaw_hint_deg = 0.0;
  bool yaw_hint_given = false;
  bool help = false;
};

Args parse_args(int argc, char ** argv)
{
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string token = argv[i];
    if (token == "-h" || token == "--help") {
      a.help = true;
      continue;
    }
    const auto eq = token.find('=');
    const std::string key = token.substr(0, eq);
    const auto value = [&]() -> std::string {
      if (eq != std::string::npos) return token.substr(eq + 1);
      if (i + 1 >= argc) throw std::runtime_error("参数 " + key + " 需要一个值");
      return argv[++i];
    };

    if (key == "--config" || key == "-c")
      a.config = value();
    else if (key == "--mode")
      a.mode = value();
    else if (key == "--camera")
      a.camera = value();
    else if (key == "--xyz")
      a.xyz = value();
    else if (key == "--ypr-deg")
      a.ypr = value();
    else if (key == "--joint")
      a.joint = value();
    else if (key == "--method")
      a.method = value();
    else if (key == "--out")
      a.out = value();
    else if (key == "--from-samples")
      a.from_samples = value();
    else if (key == "--cols")
      a.cols = std::stoi(value());
    else if (key == "--rows")
      a.rows = std::stoi(value());
    else if (key == "--square-mm")
      a.square_mm = std::stod(value());
    else if (key == "--tz-mm") {
      a.tz_mm = std::stod(value());
      a.tz_given = true;
    } else if (key == "--yaw-hint") {
      a.yaw_hint_deg = std::stod(value());
      a.yaw_hint_given = true;
    } else if (!token.empty() && token[0] != '-') {
      if (a.config.empty()) a.config = token;
    } else {
      throw std::runtime_error("未知参数: " + token + "（用 --help 看用法）");
    }
  }
  return a;
}

}  // namespace

int main(int argc, char * argv[])
{
  Args args;
  try {
    args = parse_args(argc, argv);
  } catch (const std::exception & e) {
    std::cout << "[error] " << e.what() << "\n\n";
    print_help();
    return 1;
  }
  if (args.help || argc <= 1) {
    print_help();
    return 0;
  }

  if (args.mode != "manual" && args.mode != "handeye") {
    std::cout << "[error] 必须指定 --mode manual 或 --mode handeye\n\n";
    print_help();
    return 1;
  }

  const std::string joint =
    args.joint.empty() ? (args.camera.rfind("omni", 0) == 0 ? "lower" : "upper") : args.joint;

  if (args.mode == "manual") {
    return run_manual(args.camera, args.xyz, args.ypr, args.out) ? 0 : 1;
  }

  if (args.from_samples.empty() && args.config.empty()) {
    std::cout << "[error] handeye 模式需要配置文件（用它的内参和串口），或者 --from-samples\n";
    return 1;
  }
  // 方位角先验：命令行优先，其次配置里的 omni_mount_yaw_N，最后默认 0
  double yaw_hint = args.yaw_hint_deg;
  if (!args.yaw_hint_given && args.method == "axis") {
    bool from_config = false;
    if (args.camera.rfind("omni", 0) == 0 && !args.config.empty()) {
      auto yaml = tools::load(args.config);
      const std::string key = "omni_mount_yaw_" + args.camera.substr(4);
      if (yaml[key]) {
        yaw_hint = yaml[key].as<double>();
        from_config = true;
      }
    }
    if (from_config) {
      std::cout << "[hint] 方位角先验取自配置: " << yaw_hint << " 度\n";
    } else {
      std::cout << "[warn] 未提供 --yaw-hint，方位角先验按 0 度处理；\n"
                   "       若该相机不是朝正前方，请用 --yaw-hint 给出安装角（如侧视 90）。\n";
    }
  }

  return run_handeye(
           args.config, args.camera, joint, args.cols, args.rows, args.square_mm, args.method,
           args.out, args.from_samples, args.tz_mm, args.tz_given, yaw_hint)
           ? 0
           : 1;
}
