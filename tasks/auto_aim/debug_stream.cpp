#include "debug_stream.hpp"

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

// PnP 一级校验：与 Solver 共用同一份解算内核（tasks/auto_aim/pnp.cpp），
// 调试线程里重算一次位姿再投影回图像，不改变主链路任何行为。
#include "pnp.hpp"

// SP_WITH_ROS2 由 io 目标在找到 ROS2 时以 PUBLIC 方式定义（见 io/CMakeLists.txt）。
// 没有该宏时本文件整体退化为空实现，业务代码无需任何 #ifdef。
#ifdef SP_WITH_ROS2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#endif

namespace auto_aim
{
namespace
{
using Clock = std::chrono::steady_clock;
constexpr double kRad2Deg = 57.29577951308232;

// ---------------- 配置 ----------------

struct Options
{
  bool enable = false;       // 只有写了 debug_ros2 段才考虑启用
  std::string ns = "/sp_debug";
  double image_hz = 15.0;    // 图像推送频率上限，<= 0 表示每帧都推
  int jpeg_quality = 75;
  bool publish_raw = false;
  bool publish_yolo_input = true;
  bool pnp_check = true;     // 在推流图像上叠加 PnP 一级校验（重投影点 + 板面三轴）
};

Options load_options(const YAML::Node & yaml)
{
  Options o;
  const auto y = yaml["debug_ros2"];
  if (!y || !y.IsMap()) return o;  // 没有这一段 = 不启用，其他入口的 yaml 不受影响

  if (y["enable"]) o.enable = y["enable"].as<bool>();
  if (y["namespace"]) o.ns = y["namespace"].as<std::string>();
  if (y["image_hz"]) o.image_hz = y["image_hz"].as<double>();
  if (y["jpeg_quality"]) o.jpeg_quality = std::clamp(y["jpeg_quality"].as<int>(), 1, 100);
  if (y["publish_raw"]) o.publish_raw = y["publish_raw"].as<bool>();
  if (y["publish_yolo_input"]) o.publish_yolo_input = y["publish_yolo_input"].as<bool>();
  if (y["pnp_check"]) o.pnp_check = y["pnp_check"].as<bool>();
  return o;
}

// YOLO 输入图边长：与 yolos/*.cpp 的 letterbox 保持一致
int load_input_size(const YAML::Node & yaml)
{
  const auto name = yaml["yolo_name"] ? yaml["yolo_name"].as<std::string>() : std::string("yolov5");
  return name == "yolov8" ? 416 : 640;
}

struct RoiCfg
{
  bool use = false;
  cv::Rect roi;
};

RoiCfg load_roi(const YAML::Node & yaml)
{
  RoiCfg r;
  if (yaml["use_roi"]) r.use = yaml["use_roi"].as<bool>();
  if (yaml["roi"] && yaml["roi"].IsMap()) {
    const auto n = yaml["roi"];
    try {
      r.roi = cv::Rect(n["x"].as<int>(), n["y"].as<int>(), n["width"].as<int>(), n["height"].as<int>());
    } catch (const std::exception &) {
      r.roi = cv::Rect();
    }
  }
  return r;
}

// ---------------- PnP 一级校验的内参 ----------------

// 与 Solver 完全同源：读同一个 yaml 的 camera_matrix / distort_coeffs。
// 一级校验只依赖内参，不碰 R_camera2gimbal / IMU，所以看到的重投影残差
// 可以直接归因到「检测点 + PnP」本身。
struct PnpCfg
{
  cv::Mat camera_matrix;   // 3x3 CV_64F，空 = 未配置
  cv::Mat distort_coeffs;  // 1xN CV_64F，可空（= 无畸变）

  bool ready() const { return camera_matrix.rows == 3 && camera_matrix.cols == 3; }
};

PnpCfg load_pnp_cfg(const YAML::Node & yaml)
{
  PnpCfg c;
  try {
    if (yaml["camera_matrix"]) {
      auto d = yaml["camera_matrix"].as<std::vector<double>>();
      if (d.size() == 9) c.camera_matrix = cv::Mat(3, 3, CV_64F, d.data()).clone();
    }
    if (yaml["distort_coeffs"]) {
      auto d = yaml["distort_coeffs"].as<std::vector<double>>();
      if (!d.empty()) c.distort_coeffs = cv::Mat(1, static_cast<int>(d.size()), CV_64F, d.data()).clone();
    }
  } catch (const std::exception &) {
    c = PnpCfg{};
  }
  return c;
}

// ---------------- 图像处理（全部在工作线程执行） ----------------

cv::Scalar armor_color(Color c)
{
  switch (c) {
    case red:
      return {0, 0, 255};
    case blue:
      return {255, 128, 0};
    case extinguish:
      return {160, 160, 160};
    default:
      return {255, 0, 255};
  }
}

// 复刻 yolos/*.cpp::detect 的预处理（ROI 裁剪 + 等比 letterbox），得到网络真正的输入图
cv::Mat make_yolo_input(const cv::Mat & raw, const RoiCfg & cfg, int size)
{
  cv::Mat bgr = raw;
  if (cfg.use && cfg.roi.area() > 0) {
    cv::Rect roi = cfg.roi;
    if (roi.width < 0) roi.width = raw.cols - roi.x;    // -1 表示该维度不裁切
    if (roi.height < 0) roi.height = raw.rows - roi.y;
    roi &= cv::Rect(0, 0, raw.cols, raw.rows);
    if (roi.area() > 0) bgr = raw(roi);
  }

  const double scale = std::min(static_cast<double>(size) / bgr.rows, static_cast<double>(size) / bgr.cols);
  const int h = std::clamp(static_cast<int>(bgr.rows * scale), 1, size);
  const int w = std::clamp(static_cast<int>(bgr.cols * scale), 1, size);

  cv::Mat input(size, size, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::resize(bgr, input(cv::Rect(0, 0, w, h)), {w, h});
  return input;
}

// 左上角半透明信息面板
void draw_panel(cv::Mat & img, const std::vector<std::string> & lines, double s)
{
  if (lines.empty()) return;

  const int line_h = static_cast<int>(30 * s);
  int max_w = 0;
  for (const auto & line : lines) {
    int base = 0;
    max_w = std::max(max_w, cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, 0.6 * s, 2, &base).width);
  }

  cv::Rect box(8, 8, std::min(max_w + 20, img.cols - 16), std::min(line_h * static_cast<int>(lines.size()) + 12, img.rows - 16));
  if (box.width <= 0 || box.height <= 0) return;

  cv::Mat roi = img(box);
  cv::Mat black(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
  cv::addWeighted(roi, 0.25, black, 0.75, 0, roi);

  for (size_t i = 0; i < lines.size(); ++i) {
    const int y = 8 + line_h * static_cast<int>(i) + line_h - 10;
    if (y >= img.rows) break;
    cv::putText(
      img, lines[i], cv::Point(8 + 10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6 * s, cv::Scalar(0, 255, 255), 2,
      cv::LINE_AA);
  }
}

// ---------------------------------------------------------------------------
//  PnP 一级校验（只看内参，不含外参 / IMU）
//
//  对每块装甲板重跑一次 solve_pnp_armor()（与 Solver::solve 同一份内核），
//  然后把解出来的位姿重新投影回图像：
//      · 绿色十字   = 4 个模型点按解出位姿重投影的位置
//      · 连线       = 检测点 -> 重投影点，<1px 绿 / <3px 黄 / 更大 红
//      · 三色细线    = R_armor2camera 的三列（X 红 / Y 绿 / Z 蓝），即装甲板姿态
//      · 框角 px 数字 = 该板的 reprojection_error（4 点平均）
//  位姿正确时十字应精确压住检测点，连线长度就是残差。
//  注意：这里在调试工作线程重新解算，主线程零额外开销。
// ---------------------------------------------------------------------------
struct PnpStats
{
  int solved = 0;       // 成功解算的装甲板数
  double mean_px = 0;   // 各板 reprojection_error 的均值
  double max_px = 0;    // 单板最差
};

PnpStats draw_pnp_check(
  cv::Mat & img, const std::list<Armor> & armors, const PnpCfg & cfg, double s)
{
  PnpStats st;
  if (!cfg.ready() || armors.empty()) return st;

  const int thick = std::max(1, static_cast<int>(std::round(1.2 * s)));
  const int cross = std::max(4, static_cast<int>(std::round(7 * s)));
  const cv::Vec3d zero(0, 0, 0);

  for (const auto & armor : armors) {
    const auto pnp = solve_pnp_armor(armor, cfg.camera_matrix, cfg.distort_coeffs);
    if (!pnp.success) continue;

    // 位姿 -> rvec/tvec：直接复用内核给出的 R / t，不在调试侧重新求解
    cv::Mat rmat;
    cv::eigen2cv(pnp.R_armor2camera, rmat);
    cv::Vec3d rvec;
    cv::Rodrigues(rmat, rvec);
    const cv::Vec3d tvec(
      pnp.xyz_in_camera.x(), pnp.xyz_in_camera.y(), pnp.xyz_in_camera.z());

    // --- 1) 4 个模型点重投影：与检测点逐点比对 ---
    const auto & object_points = armor_object_points(armor.type);
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(
      object_points, rvec, tvec, cfg.camera_matrix, cfg.distort_coeffs, reprojected);
    if (reprojected.size() != armor.points.size()) continue;

    for (std::size_t i = 0; i < reprojected.size(); ++i) {
      const double e = cv::norm(armor.points[i] - reprojected[i]);
      const cv::Scalar c = (e < 1.0)   ? cv::Scalar(0, 255, 0)
                           : (e < 3.0) ? cv::Scalar(0, 255, 255)
                                       : cv::Scalar(0, 0, 255);
      cv::line(img, armor.points[i], reprojected[i], c, thick, cv::LINE_AA);
      cv::drawMarker(img, reprojected[i], c, cv::MARKER_CROSS, cross, thick, cv::LINE_AA);
    }

    // --- 2) 板面三轴：R 的三列就是三根正交单位向量，投到图上即姿态 ---
    //     点在相机系下给出，所以 rvec = tvec = 0；z<=0 说明解跑到相机后方，跳过
    const double axis_len = (armor.type == ArmorType::big) ? 0.14 : 0.09;
    const Eigen::Vector3d & c0 = pnp.xyz_in_camera;
    std::vector<cv::Point3f> cam_points;
    cam_points.emplace_back(
      static_cast<float>(c0.x()), static_cast<float>(c0.y()), static_cast<float>(c0.z()));
    for (int i = 0; i < 3; ++i) {
      const Eigen::Vector3d p = c0 + pnp.R_armor2camera.col(i) * axis_len;
      cam_points.emplace_back(
        static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    }

    if (cam_points[0].z > 1e-3) {
      std::vector<cv::Point2f> axis_px;
      cv::projectPoints(cam_points, zero, zero, cfg.camera_matrix, cfg.distort_coeffs, axis_px);
      // BGR：X 红 / Y 绿 / Z 蓝
      const cv::Scalar axis_color[3] = {
        cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0)};
      const char * axis_name[3] = {"X", "Y", "Z"};
      for (int i = 0; i < 3; ++i) {
        cv::line(img, axis_px[0], axis_px[i + 1], axis_color[i], thick, cv::LINE_AA);
        cv::putText(
          img, axis_name[i],
          cv::Point(
            static_cast<int>(axis_px[i + 1].x + 3 * s), static_cast<int>(axis_px[i + 1].y - 3 * s)),
          cv::FONT_HERSHEY_SIMPLEX, 0.5 * s, axis_color[i], thick, cv::LINE_AA);
      }
    }

    // --- 3) 该板的「重投影误差 + 距离」，贴在框下方（避开左上角标签与中心数字）---
    //     距离可以直接拿卷尺量着核对：单目尺度全由大/小装甲板宽度决定，
    //     type 判错时距离会成比例偏掉。
    const std::string text =
      fmt::format("{:.1f}px {:.2f}m", pnp.reprojection_error, pnp.xyz_in_camera.norm());
    const cv::Scalar tc = (pnp.reprojection_error < 1.0)   ? cv::Scalar(0, 255, 0)
                          : (pnp.reprojection_error < 3.0) ? cv::Scalar(0, 255, 255)
                                                           : cv::Scalar(0, 0, 255);
    int base = 0;
    const auto tsize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5 * s, thick, &base);
    const cv::Point org(
      std::max(0, std::min(armor.box.x, img.cols - tsize.width - 1)),
      std::max(
        tsize.height + 2,
        std::min(armor.box.y + armor.box.height + tsize.height + 2, img.rows - 2)));
    cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.5 * s, tc, thick, cv::LINE_AA);

    st.solved += 1;
    st.mean_px += pnp.reprojection_error;
    st.max_px = std::max(st.max_px, pnp.reprojection_error);
  }

  if (st.solved > 0) st.mean_px /= st.solved;
  return st;
}

// 在原始图像上叠加：YOLO 检测框 + 敌人数字标记 + PnP 校验 + FPS/位姿文字
void annotate(
  cv::Mat & img, const std::list<Armor> & armors, const DebugStatus & status, double loop_fps,
  double yolo_fps, double yolo_ms, const std::string & tag, const PnpCfg & pnp_cfg,
  bool pnp_check)
{
  const double s = std::max(0.5, img.cols / 1280.0);
  const int thick = std::max(1, static_cast<int>(std::round(2 * s)));

  for (const auto & armor : armors) {
    const int id = static_cast<int>(armor.name) + 1;  // 与裁判系统一致：one=1 ... base=8
    const cv::Scalar color = armor_color(armor.color);

    cv::rectangle(img, armor.box, color, thick);

    // 左上角标签：编号 + 颜色 + 置信度
    const std::string label =
      fmt::format("#{} {} {:.2f}", id, COLORS[armor.color], armor.confidence);
    int base = 0;
    const auto size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6 * s, thick, &base);
    const cv::Point org(
      armor.box.x, std::max(armor.box.y - 6, size.height + 4));
    cv::rectangle(
      img, cv::Point(org.x, org.y - size.height - 4),
      cv::Point(std::min(org.x + size.width + 6, img.cols - 1), org.y + base), color, -1);
    cv::putText(
      img, label, org, cv::FONT_HERSHEY_SIMPLEX, 0.6 * s, cv::Scalar(0, 0, 0), thick, cv::LINE_AA);

    // 框中心的大号数字：远处也能一眼看清是几号敌人
    const std::string number = std::to_string(id);
    int nbase = 0;
    const auto nsize = cv::getTextSize(number, cv::FONT_HERSHEY_SIMPLEX, 1.4 * s, thick + 1, &nbase);
    cv::putText(
      img, number, cv::Point(static_cast<int>(armor.center.x - nsize.width / 2.0),
                             static_cast<int>(armor.center.y + nsize.height / 2.0)),
      cv::FONT_HERSHEY_SIMPLEX, 1.4 * s, color, thick + 1, cv::LINE_AA);

    for (const auto & p : armor.points) cv::circle(img, p, std::max(2, thick + 1), color, -1);
  }

  // PnP 一级校验：检测点 vs 重投影点 + 板面三轴（画在框和关键点之上）
  PnpStats pnp_stats;
  if (pnp_check) pnp_stats = draw_pnp_check(img, armors, pnp_cfg, s);

  std::vector<std::string> lines;
  lines.push_back(fmt::format(
    "[{}] YOLO {:.1f} FPS ({:.1f} ms)  loop {:.1f} Hz  boxes {}", tag, yolo_fps, yolo_ms, loop_fps,
    armors.size()));

  if (!status.state.empty()) lines.back() += "  state: " + status.state;

  lines.push_back(fmt::format(
    "gimbal  yaw {:+.1f}  pitch {:+.1f}  roll {:+.1f}  (deg)", status.gimbal_ypr[0] * kRad2Deg,
    status.gimbal_ypr[1] * kRad2Deg, status.gimbal_ypr[2] * kRad2Deg));

  lines.push_back(fmt::format(
    "target  yaw {:+.1f}  pitch {:+.1f}  (deg){}", status.target_ypr[0] * kRad2Deg,
    status.target_ypr[1] * kRad2Deg, status.control ? "" : "  [hold]"));

  lines.push_back(fmt::format(
    "delta   yaw {:+.1f}  pitch {:+.1f}  (deg)   {}",
    (status.target_ypr[0] - status.gimbal_ypr[0]) * kRad2Deg,
    (status.target_ypr[1] - status.gimbal_ypr[1]) * kRad2Deg, status.shoot ? "FIRE" : ""));

  // PnP 校验结果：绿色十字 = 重投影，连线 = 残差，三色细线 = 板面 X/Y/Z 轴
  if (pnp_check) {
    if (!pnp_cfg.ready()) {
      lines.push_back("pnp    [!] 缺少 camera_matrix，一级校验未绘制");
    } else if (pnp_stats.solved == 0) {
      lines.push_back("pnp    no solution");
    } else {
      lines.push_back(fmt::format(
        "pnp    reproj mean {:.2f} px  max {:.2f} px  ({}/{})", pnp_stats.mean_px, pnp_stats.max_px,
        pnp_stats.solved, armors.size()));
    }
  }

  if (!status.note.empty()) lines.push_back(status.note);

  draw_panel(img, lines, s);
}

#ifdef SP_WITH_ROS2
void publish_jpeg(
  const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & pub, cv::Mat & img,
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id, int quality)
{
  if (!pub || img.empty()) return;

  std::vector<uchar> buffer;
  const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
  if (!cv::imencode(".jpg", img, buffer, params) || buffer.empty()) return;

  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.format = "jpeg";
  msg.data = std::move(buffer);
  pub->publish(msg);
}

// 角度 -> 单位方向向量 + 姿态。工程约定 pitch 向上为负，这里换算成标准抬升角。
geometry_msgs::msg::PoseStamped make_pose(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id,
  const Eigen::Vector3d & ypr)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  const double yaw = ypr[0];
  const double pitch_up = -ypr[1];
  const double cp = std::cos(pitch_up);
  msg.pose.position.x = cp * std::cos(yaw);
  msg.pose.position.y = cp * std::sin(yaw);
  msg.pose.position.z = std::sin(pitch_up);

  const Eigen::Quaterniond q(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch_up, Eigen::Vector3d::UnitY()));
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  return msg;
}
#endif

}  // namespace

// ---------------- 实现 ----------------

struct DebugStream::Impl
{
  Options opt;
  RoiCfg roi;
  PnpCfg pnp_cfg;
  int input_size = 640;
  bool on = false;
  std::string tag;

  // 帧率统计
  Clock::time_point t_frame{};
  bool armed = false;
  Clock::time_point t_yolo_begin{};
  Clock::time_point t_yolo_last{};
  bool have_yolo_last = false;
  double yolo_ms = 0.0;
  double yolo_fps = 0.0;
  double loop_fps = 0.0;
  Clock::time_point t_loop_last{};
  bool have_loop_last = false;

  // 主线程 -> 工作线程：只保留最新一帧（旧帧直接丢弃，优先保证实时性）
  struct Job
  {
    cv::Mat raw;
    std::list<Armor> armors;
    DebugStatus status;
    double loop_fps = 0.0;
    double yolo_fps = 0.0;
    double yolo_ms = 0.0;
  };

  std::mutex mutex;
  std::condition_variable cv;
  std::optional<Job> pending;
  Job staging;
  bool stop = false;
  std::thread worker;

#ifdef SP_WITH_ROS2
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_image;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_raw;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_input;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gimbal;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target;
  bool own_context = false;
#endif

  Impl(const std::string & config_path, std::string tag_);
  ~Impl();

  void run();
  void emit(Job & job);
};

DebugStream::Impl::Impl(const std::string & config_path, std::string tag_) : tag(std::move(tag_))
{
  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(config_path);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[DebugStream] 读取配置失败（%s），调试数据流关闭\n", e.what());
    return;
  }

  opt = load_options(yaml);
  roi = load_roi(yaml);
  input_size = load_input_size(yaml);
  pnp_cfg = load_pnp_cfg(yaml);

  if (!opt.enable) return;
  while (!opt.ns.empty() && opt.ns.back() == '/') opt.ns.pop_back();

#ifdef SP_WITH_ROS2
  try {
    // 已有节点（sentry 的 io::ROS2）时直接复用同一个 context，不能重复 init
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      own_context = true;
    }

    node = std::make_shared<rclcpp::Node>("sp_debug_" + tag);
    const std::string prefix = opt.ns + "/" + tag;

    // 图像只保留最新一帧：网络跟不上时丢旧帧，不堆积延迟
    pub_image = node->create_publisher<sensor_msgs::msg::CompressedImage>(prefix + "/image", rclcpp::QoS(1));
    if (opt.publish_raw)
      pub_raw = node->create_publisher<sensor_msgs::msg::CompressedImage>(prefix + "/image_raw", rclcpp::QoS(1));
    if (opt.publish_yolo_input)
      pub_input =
        node->create_publisher<sensor_msgs::msg::CompressedImage>(prefix + "/yolo_input", rclcpp::QoS(1));
    pub_status = node->create_publisher<std_msgs::msg::String>(prefix + "/status", rclcpp::QoS(10));
    pub_gimbal =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(prefix + "/gimbal_pose", rclcpp::QoS(10));
    pub_target =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(prefix + "/target_pose", rclcpp::QoS(10));

    on = true;
    worker = std::thread([this] { run(); });

    RCLCPP_INFO(
      node->get_logger(), "调试数据流已启动：%s/{image,image_raw,yolo_input,status,gimbal_pose,target_pose}",
      prefix.c_str());

    // PnP 一级校验的状态提示：缺内参时不会画，先说清楚，免得以为是没实现
    if (!opt.pnp_check) {
      RCLCPP_INFO(node->get_logger(), "PnP 一级校验：关闭（debug_ros2.pnp_check=false）");
    } else if (!pnp_cfg.ready()) {
      RCLCPP_WARN(
        node->get_logger(), "PnP 一级校验：已开启但未读到 camera_matrix（需 9 个数），叠加层不绘制");
    } else {
      RCLCPP_INFO(
        node->get_logger(), "PnP 一级校验：已叠加到 image 话题（检测点 vs 重投影点 + 板面三轴）");
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "[DebugStream] ROS2 初始化失败（%s），调试数据流关闭\n", e.what());
    on = false;
  }
#else
  std::fprintf(stderr, "[DebugStream] 编译时未找到 ROS2，调试数据流关闭\n");
  on = false;
#endif
}

DebugStream::Impl::~Impl()
{
  {
    std::lock_guard<std::mutex> lock(mutex);
    stop = true;
  }
  cv.notify_all();
  if (worker.joinable()) worker.join();

#ifdef SP_WITH_ROS2
  pub_image.reset();
  pub_raw.reset();
  pub_input.reset();
  pub_status.reset();
  pub_gimbal.reset();
  pub_target.reset();
  node.reset();
  if (own_context && rclcpp::ok()) rclcpp::shutdown();
#endif
}

void DebugStream::Impl::run()
{
  while (true) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait(lock, [this] { return stop || pending.has_value(); });
      if (stop) break;
      job = std::move(*pending);
      pending.reset();
    }

    try {
      emit(job);
    } catch (const std::exception & e) {
      std::fprintf(stderr, "[DebugStream] 发布失败：%s\n", e.what());
    }
  }
}

void DebugStream::Impl::emit(Job & job)
{
  if (job.raw.empty()) return;

#ifdef SP_WITH_ROS2
  if (!node || !rclcpp::ok()) return;

  const std::string frame_id = "world";
  const auto stamp = node->now();

  // 1) 相机原图（可选）
  publish_jpeg(pub_raw, job.raw, stamp, frame_id, opt.jpeg_quality);

  // 2) YOLO 预处理后的输入图（可选）：ROI 裁剪 + letterbox，与网络真正吃进去的图一致
  if (pub_input) {
    cv::Mat input = make_yolo_input(job.raw, roi, input_size);
    publish_jpeg(pub_input, input, stamp, frame_id, opt.jpeg_quality);
  }

  // 3) 原图 + 检测框 + 数字编号 + FPS/位姿叠加
  annotate(
    job.raw, job.armors, job.status, job.loop_fps, job.yolo_fps, job.yolo_ms, tag, pnp_cfg,
    opt.pnp_check);
  publish_jpeg(pub_image, job.raw, stamp, frame_id, opt.jpeg_quality);

  // 4) 状态 JSON
  if (pub_status) {
    std_msgs::msg::String msg;
    msg.data = fmt::format(
      "{{\"tag\":\"{}\",\"loop_fps\":{:.1f},\"yolo_fps\":{:.1f},\"infer_ms\":{:.1f},\"boxes\":{},"
      "\"state\":\"{}\",\"control\":{},\"shoot\":{},"
      "\"gimbal\":{{\"yaw_deg\":{:.2f},\"pitch_deg\":{:.2f},\"roll_deg\":{:.2f}}},"
      "\"target\":{{\"yaw_deg\":{:.2f},\"pitch_deg\":{:.2f}}},"
      "\"error\":{{\"yaw_deg\":{:.2f},\"pitch_deg\":{:.2f}}}}}",
      tag, job.loop_fps, job.yolo_fps, job.yolo_ms, job.armors.size(), job.status.state,
      job.status.control, job.status.shoot, job.status.gimbal_ypr[0] * kRad2Deg,
      job.status.gimbal_ypr[1] * kRad2Deg, job.status.gimbal_ypr[2] * kRad2Deg,
      job.status.target_ypr[0] * kRad2Deg, job.status.target_ypr[1] * kRad2Deg,
      (job.status.target_ypr[0] - job.status.gimbal_ypr[0]) * kRad2Deg,
      (job.status.target_ypr[1] - job.status.gimbal_ypr[1]) * kRad2Deg);
    pub_status->publish(msg);
  }

  // 5) 云台位姿 / 目标位姿（Foxglove 3D 面板可直接看朝向；位置为单位方向向量）
  if (pub_gimbal) pub_gimbal->publish(make_pose(stamp, frame_id, job.status.gimbal_ypr));
  if (pub_target) pub_target->publish(make_pose(stamp, frame_id, job.status.target_ypr));
#else
  (void)job;
#endif
}

// ---------------- 对外接口 ----------------

DebugStream::DebugStream(const std::string & config_path, const std::string & tag)
: impl_(std::make_unique<Impl>(config_path, tag))
{
}

DebugStream::~DebugStream() = default;

bool DebugStream::enabled() const { return impl_ && impl_->on; }

void DebugStream::frame(const cv::Mat & raw_img)
{
  auto & d = *impl_;
  if (!d.on || raw_img.empty()) {
    d.armed = false;
    return;
  }

  const auto now = Clock::now();
  if (d.have_loop_last) {
    const double dt = std::chrono::duration<double>(now - d.t_loop_last).count();
    if (dt > 1e-6) d.loop_fps = d.loop_fps <= 0 ? 1.0 / dt : 0.9 * d.loop_fps + 0.1 / dt;
  }
  d.t_loop_last = now;
  d.have_loop_last = true;

  // 节流：超频时不拷贝图像，主链路几乎零代价
  if (d.opt.image_hz > 0) {
    const auto period = std::chrono::duration<double>(1.0 / d.opt.image_hz);
    if (now - d.t_frame < std::chrono::duration_cast<Clock::duration>(period)) {
      d.armed = false;
      return;
    }
  }
  d.t_frame = now;

  {
    std::lock_guard<std::mutex> lock(d.mutex);
    raw_img.copyTo(d.staging.raw);
  }
  d.armed = true;
}

void DebugStream::publish(const std::list<Armor> & armors, const DebugStatus & status)
{
  auto & d = *impl_;
  if (!d.on || !d.armed) return;
  d.armed = false;

  {
    std::lock_guard<std::mutex> lock(d.mutex);
    d.staging.armors = armors;
    d.staging.status = status;
    d.staging.loop_fps = d.loop_fps;
    d.staging.yolo_fps = d.yolo_fps;
    d.staging.yolo_ms = d.yolo_ms;
    d.pending = std::move(d.staging);  // 覆盖尚未处理完的旧帧
  }
  d.cv.notify_one();
}

void DebugStream::begin_yolo()
{
  if (!impl_->on) return;
  impl_->t_yolo_begin = Clock::now();
}

void DebugStream::end_yolo()
{
  auto & d = *impl_;
  if (!d.on) return;

  const double ms =
    std::chrono::duration<double, std::milli>(Clock::now() - d.t_yolo_begin).count();
  yolo_sample(ms);
}

void DebugStream::yolo_sample(double infer_ms)
{
  auto & d = *impl_;
  if (!d.on) return;

  const auto now = Clock::now();
  d.yolo_ms = d.yolo_ms <= 0 ? infer_ms : 0.9 * d.yolo_ms + 0.1 * infer_ms;

  if (d.have_yolo_last) {
    const double dt = std::chrono::duration<double>(now - d.t_yolo_last).count();
    if (dt > 1e-6) {
      const double fps = 1.0 / dt;
      d.yolo_fps = d.yolo_fps <= 0 ? fps : 0.9 * d.yolo_fps + 0.1 * fps;
    }
  }
  d.t_yolo_last = now;
  d.have_yolo_last = true;
}

}  // namespace auto_aim
