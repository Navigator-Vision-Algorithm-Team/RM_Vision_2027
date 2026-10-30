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

// 在原始图像上叠加：YOLO 检测框 + 敌人数字标记 + FPS/位姿文字
void annotate(
  cv::Mat & img, const std::list<Armor> & armors, const DebugStatus & status, double loop_fps,
  double yolo_fps, double yolo_ms, const std::string & tag)
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

  if (status.has_track) {
    const double distance = std::hypot(status.track_xyz[0], status.track_xyz[1]);
    lines.push_back(fmt::format(
      "track   d {:.2f}m  v({:+.2f},{:+.2f},{:+.2f})  w {:+.1f} deg/s  r {:.3f}m  {}", distance,
      status.track_vel[0], status.track_vel[1], status.track_vel[2], status.track_w * kRad2Deg,
      status.track_r, status.converged ? "converged" : "converging"));
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
  annotate(job.raw, job.armors, job.status, job.loop_fps, job.yolo_fps, job.yolo_ms, tag);
  publish_jpeg(pub_image, job.raw, stamp, frame_id, opt.jpeg_quality);

  // 4) 状态 JSON
  if (pub_status) {
    // 跟踪预测信息：未接入 Tracker 的入口（如哨兵）恒输出 ok:false，其余字段保持不变
    const std::string track_json =
      job.status.has_track
        ? fmt::format(
            "\"track\":{{\"ok\":true,\"x\":{:.3f},\"y\":{:.3f},\"z\":{:.3f},"
            "\"vx\":{:.2f},\"vy\":{:.2f},\"vz\":{:.2f},"
            "\"w\":{:.2f},\"r\":{:.3f},\"dist\":{:.2f},\"converged\":{}}}",
            job.status.track_xyz[0], job.status.track_xyz[1], job.status.track_xyz[2],
            job.status.track_vel[0], job.status.track_vel[1], job.status.track_vel[2],
            job.status.track_w, job.status.track_r,
            std::hypot(job.status.track_xyz[0], job.status.track_xyz[1]), job.status.converged)
        : std::string{"\"track\":{\"ok\":false}"};

    std_msgs::msg::String msg;
    msg.data = fmt::format(
      "{{\"tag\":\"{}\",\"loop_fps\":{:.1f},\"yolo_fps\":{:.1f},\"infer_ms\":{:.1f},\"boxes\":{},"
      "\"state\":\"{}\",\"control\":{},\"shoot\":{},"
      "\"gimbal\":{{\"yaw_deg\":{:.2f},\"pitch_deg\":{:.2f},\"roll_deg\":{:.2f}}},"
      "\"target\":{{\"yaw_deg\":{:.2f},\"pitch_deg\":{:.2f}}},"
      "\"error\":{{\"yaw_deg\":{:.2f},\"pitch_deg\":{:.2f}}},{}}}",
      tag, job.loop_fps, job.yolo_fps, job.yolo_ms, job.armors.size(), job.status.state,
      job.status.control, job.status.shoot, job.status.gimbal_ypr[0] * kRad2Deg,
      job.status.gimbal_ypr[1] * kRad2Deg, job.status.gimbal_ypr[2] * kRad2Deg,
      job.status.target_ypr[0] * kRad2Deg, job.status.target_ypr[1] * kRad2Deg,
      (job.status.target_ypr[0] - job.status.gimbal_ypr[0]) * kRad2Deg,
      (job.status.target_ypr[1] - job.status.gimbal_ypr[1]) * kRad2Deg, track_json);
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
