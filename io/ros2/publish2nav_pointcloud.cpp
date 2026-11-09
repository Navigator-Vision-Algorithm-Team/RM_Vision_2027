#include "publish2nav_pointcloud.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <utility>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros_time.hpp"
#include "tools/logger.hpp"

namespace io
{
namespace
{

// C3 点结构必须与 PointCloud2 的 point_step 严格一致（4 × FLOAT32）
static_assert(sizeof(ObstacleCloudPoint) == 16, "ObstacleCloudPoint 必须是紧凑的 16 字节");

struct Options
{
  bool enable = false;
  std::string topic_obstacle = "/red_standard_robot1/vision/obstacle_cloud";
  std::string topic_map = "/red_standard_robot1/vision/map_points";
  std::string topic_status = "/red_standard_robot1/vision/status";
  std::string frame_body = "gimbal_yaw";
  std::string frame_world = "camera_init";
  double hz_obstacle = 10.0;
  double hz_map = 0.0;  // 比赛配置写 0 => 不发 C4
  size_t max_points_obstacle = 512;
  size_t max_points_map = 20000;
  double max_range_obstacle = 5.0;
  double max_range_map = 12.0;
  float intensity = 1.0f;
};

Options load_options(const YAML::Node & yaml)
{
  Options opt;
  const auto node = yaml["obstacle_cloud"];
  if (!node) return opt;  // 整段缺失 => 全关

  opt.enable = node["enable"] ? node["enable"].as<bool>() : false;
  if (node["topic_obstacle"]) opt.topic_obstacle = node["topic_obstacle"].as<std::string>();
  if (node["topic_map"]) opt.topic_map = node["topic_map"].as<std::string>();
  if (node["topic_status"]) opt.topic_status = node["topic_status"].as<std::string>();
  if (node["frame_id_body"]) opt.frame_body = node["frame_id_body"].as<std::string>();
  if (node["frame_id_world"]) opt.frame_world = node["frame_id_world"].as<std::string>();
  if (node["publish_hz_obstacle"]) opt.hz_obstacle = node["publish_hz_obstacle"].as<double>();
  if (node["publish_hz_map"]) opt.hz_map = node["publish_hz_map"].as<double>();
  if (node["max_points_obstacle"]) {
    const auto v = node["max_points_obstacle"].as<int>();
    if (v > 0) opt.max_points_obstacle = static_cast<size_t>(v);
  }
  if (node["max_points_map"]) {
    const auto v = node["max_points_map"].as<int>();
    if (v > 0) opt.max_points_map = static_cast<size_t>(v);
  }
  if (node["max_range_obstacle"]) opt.max_range_obstacle = node["max_range_obstacle"].as<double>();
  if (node["max_range_map"]) opt.max_range_map = node["max_range_map"].as<double>();
  if (node["intensity"]) opt.intensity = node["intensity"].as<float>();
  return opt;
}

void add_field(
  sensor_msgs::msg::PointCloud2 & msg, const char * name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  msg.fields.push_back(field);
}

// C3：x y z intensity，4 × FLOAT32 = 16 B/点
void fill_obstacle_cloud(
  sensor_msgs::msg::PointCloud2 & msg, const std::string & frame, const rclcpp::Time & stamp,
  const std::vector<ObstacleCloudPoint> & points)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = true;

  msg.fields.clear();
  msg.fields.reserve(4);
  add_field(msg, "x", 0, sensor_msgs::msg::PointField::FLOAT32);
  add_field(msg, "y", 4, sensor_msgs::msg::PointField::FLOAT32);
  add_field(msg, "z", 8, sensor_msgs::msg::PointField::FLOAT32);
  add_field(msg, "intensity", 12, sensor_msgs::msg::PointField::FLOAT32);

  msg.data.assign(msg.row_step, 0);
  if (!points.empty()) std::memcpy(msg.data.data(), points.data(), msg.row_step);
}

// C4：pcl::PointXYZINormal 布局，48 B/点。
// x/y/z 之后是 4 字节对齐 padding（PCL 的 EIGEN_ALIGN16），所以 intensity 在
// 偏移 16 而不是 12；离线工具用 PCL 读回来时字段名与偏移必须完全对上。
void fill_map_cloud(
  sensor_msgs::msg::PointCloud2 & msg, const std::string & frame, const rclcpp::Time & stamp,
  const std::vector<MapCloudPoint> & points)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.point_step = 48;
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = true;

  msg.fields.clear();
  msg.fields.reserve(8);
  add_field(msg, "x", 0, sensor_msgs::msg::PointField::FLOAT32);
  add_field(msg, "y", 4, sensor_msgs::msg::PointField::FLOAT32);
  add_field(msg, "z", 8, sensor_msgs::msg::PointField::FLOAT32);
  add_field(msg, "intensity", 16, sensor_msgs::msg::PointField::FLOAT32);
  add_field(msg, "normal_x", 20, sensor_msgs::msg::PointField::FLOAT32);
  add_field(msg, "normal_y", 24, sensor_msgs::msg::PointField::FLOAT32);
  add_field(msg, "normal_z", 28, sensor_msgs::msg::PointField::FLOAT32);
  add_field(msg, "curvature", 32, sensor_msgs::msg::PointField::FLOAT32);

  msg.data.assign(msg.row_step, 0);
  auto * d = reinterpret_cast<float *>(msg.data.data());
  for (size_t i = 0; i < points.size(); i++) {
    float * base = d + i * 12;  // 48 B = 12 个 float
    base[0] = points[i].p.x();
    base[1] = points[i].p.y();
    base[2] = points[i].p.z();
    base[4] = 1.0f;  // intensity 占位，兼容下游
    base[5] = points[i].quality;
    base[6] = static_cast<float>(points[i].observations);
    base[7] = points[i].parallax_deg;
    base[8] = points[i].consistency;
  }
}

}  // namespace

struct Publish2NavPointCloud::Impl
{
  Options opt;
  bool on = false;

  std::mutex mutex;
  std::condition_variable cv;
  bool stop = false;

  bool has_obs = false;
  bool has_map = false;
  bool has_status = false;
  std::vector<ObstacleCloudPoint> obs;
  rclcpp::Time obs_stamp_pending;
  std::vector<MapCloudPoint> map;
  rclcpp::Time map_stamp_pending;
  std::string status;

  std::thread worker;

  std::atomic<double> obs_hz{0.0};
  std::atomic<size_t> obs_sent{0};
  std::atomic<size_t> map_sent{0};

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obs;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status;
  bool own_context = false;

  explicit Impl(const std::string & config_path);
  ~Impl();

  void run();
};

Publish2NavPointCloud::Impl::Impl(const std::string & config_path)
{
  if (config_path.empty()) return;

  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(config_path);
  } catch (const std::exception & e) {
    tools::logger()->warn("[VisionCloud] 读取配置失败（{}），视觉点云发布关闭", e.what());
    return;
  }

  opt = load_options(yaml);
  if (!opt.enable) return;
  if (opt.hz_obstacle <= 0.0 && opt.hz_map <= 0.0) {
    tools::logger()->info("[VisionCloud] enable=true 但两条点云频率都是 0，只有 status 在发");
  }

  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      own_context = true;
    }

    node = std::make_shared<rclcpp::Node>("vision_cloud_publisher");

    // 时间戳基准：启动时标定一次 steady -> ROS 偏移（§5.6），之后只做加法
    rostime::calibrate(node->now());

    if (opt.hz_obstacle > 0.0) {
      // C3：BEST_EFFORT + KEEP_LAST 1，网络跟不上时宁可丢旧帧也不堆积延迟
      pub_obs = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        opt.topic_obstacle, rclcpp::SensorDataQoS());
    }
    if (opt.hz_map > 0.0) {
      // C4：RELIABLE + KEEP_LAST 5，建图点云允许延迟但不能丢
      pub_map = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        opt.topic_map, rclcpp::QoS(rclcpp::KeepLast(5)));
    }
    // C5：诊断话题，始终创建（哪怕两条点云都关了，也要能看出是「没开」还是「断了」）
    pub_status = node->create_publisher<std_msgs::msg::String>(
      opt.topic_status, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    on = true;
    worker = std::thread([this] { run(); });

    tools::logger()->info(
      "[VisionCloud] 已启动：{} {:.0f}Hz / {} {:.0f}Hz / {} 1Hz（{}，intensity={:.2f}）",
      opt.topic_obstacle, opt.hz_obstacle, opt.topic_map, opt.hz_map, opt.topic_status,
      opt.frame_body, opt.intensity);
  } catch (const std::exception & e) {
    tools::logger()->warn("[VisionCloud] ROS2 初始化失败（{}），视觉点云发布关闭", e.what());
    on = false;
  }
}

Publish2NavPointCloud::Impl::~Impl()
{
  {
    std::lock_guard<std::mutex> lock(mutex);
    stop = true;
  }
  cv.notify_all();
  if (worker.joinable()) worker.join();

  pub_obs.reset();
  pub_map.reset();
  pub_status.reset();
  node.reset();
  // rclcpp::shutdown 由 io::ROS2 统一负责，这里不动全局状态
}

void Publish2NavPointCloud::Impl::run()
{
  using Clock = std::chrono::steady_clock;

  const auto to_clock = [](double seconds) {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
  };
  const auto obs_period = to_clock(opt.hz_obstacle > 0.0 ? 1.0 / opt.hz_obstacle : 0.0);
  const auto map_period = to_clock(opt.hz_map > 0.0 ? 1.0 / opt.hz_map : 0.0);
  const auto status_period = to_clock(1.0);

  auto next_obs = Clock::now();
  auto next_map = Clock::now();
  auto next_status = Clock::now();
  auto hz_window = Clock::now();
  size_t hz_count = 0;

  while (true) {
    std::vector<ObstacleCloudPoint> obs_out;
    std::vector<MapCloudPoint> map_out;
    std::string status_out;
    rclcpp::Time obs_stamp;
    rclcpp::Time map_stamp;
    bool do_obs = false;
    bool do_map = false;
    bool do_status = false;

    {
      std::unique_lock<std::mutex> lock(mutex);
      if (stop) break;

      const auto now = Clock::now();

      if (has_obs && opt.hz_obstacle > 0.0 && now >= next_obs) {
        obs_out.swap(obs);
        obs.clear();
        obs_stamp = obs_stamp_pending;
        has_obs = false;
        next_obs = now + obs_period;
        do_obs = true;
      }
      if (has_map && opt.hz_map > 0.0 && now >= next_map) {
        map_out.swap(map);
        map.clear();
        map_stamp = map_stamp_pending;
        has_map = false;
        next_map = now + map_period;
        do_map = true;
      }
      if (has_status && now >= next_status) {
        status_out.swap(status);
        status.clear();
        has_status = false;
        next_status = now + status_period;
        do_status = true;
      }

      // 没有到点的数据时精确睡到「最早到期的那一路」：
      // 不能用「谓词 + 短超时」的轮询（谓词一旦为真就立即返回，会变成忙等）。
      if (!do_obs && !do_map && !do_status) {
        auto deadline = Clock::time_point::max();
        if (has_obs) deadline = std::min(deadline, next_obs);
        if (has_map) deadline = std::min(deadline, next_map);
        if (has_status) deadline = std::min(deadline, next_status);

        // 手上没有任何待发数据时给一个 1 s 的兜底上限（新投递会 notify 提前唤醒，
        // 随后「最新数据覆盖旧数据」）；不用「谓词 + wait」，否则谓词为真会立即返回。
        if (deadline == Clock::time_point::max()) {
          deadline = now + std::chrono::seconds(1);
        }
        cv.wait_until(lock, deadline);
        continue;
      }
    }

    if (do_obs && pub_obs) {
      // 距离过滤 + 点数上限 + intensity 覆盖都在后台线程做，不占主循环
      std::vector<ObstacleCloudPoint> filtered;
      filtered.reserve(std::min(obs_out.size(), opt.max_points_obstacle));
      for (auto & p : obs_out) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) > opt.max_range_obstacle) continue;
        p.intensity = opt.intensity;  // 必须覆盖，保证不被 costmap 的强度过滤挡掉
        filtered.push_back(p);
      }
      if (filtered.size() > opt.max_points_obstacle) {
        // 只保留最近的：深度越近越可信（§1.2 出口 A）
        std::partial_sort(
          filtered.begin(), filtered.begin() + opt.max_points_obstacle, filtered.end(),
          [](const ObstacleCloudPoint & a, const ObstacleCloudPoint & b) {
            return a.x * a.x + a.y * a.y + a.z * a.z < b.x * b.x + b.y * b.y + b.z * b.z;
          });
        filtered.resize(opt.max_points_obstacle);
      }

      sensor_msgs::msg::PointCloud2 msg;
      fill_obstacle_cloud(msg, opt.frame_body, obs_stamp, filtered);
      pub_obs->publish(msg);
      obs_sent.fetch_add(1, std::memory_order_relaxed);

      hz_count++;
      const auto now = Clock::now();
      const double span = std::chrono::duration<double>(now - hz_window).count();
      if (span >= 2.0) {
        obs_hz.store(static_cast<double>(hz_count) / span, std::memory_order_relaxed);
        hz_count = 0;
        hz_window = now;
      }
    }

    if (do_map && pub_map) {
      std::vector<MapCloudPoint> filtered;
      filtered.reserve(std::min(map_out.size(), opt.max_points_map));
      // 注意：这里不做量程过滤。C4 的点在 camera_init 系，原点是「上电时 IMU 的位置」，
      // 车跑起来后 p.norm() 已经不是「离我多远」；量程由视觉侧按「到当前相机的距离」
      // 在三角化出口处完成过滤（visual_triangulation.max_range 口径）。
      for (auto & p : map_out) {
        if (!p.p.allFinite()) continue;
        filtered.push_back(p);
      }
      if (filtered.size() > opt.max_points_map) {
        // 超上限时优先保留质量高的点（离线按 normal_x 筛点的同一口径）
        std::partial_sort(
          filtered.begin(), filtered.begin() + opt.max_points_map, filtered.end(),
          [](const MapCloudPoint & a, const MapCloudPoint & b) { return a.quality > b.quality; });
        filtered.resize(opt.max_points_map);
      }

      sensor_msgs::msg::PointCloud2 msg;
      fill_map_cloud(msg, opt.frame_world, map_stamp, filtered);
      pub_map->publish(msg);
      map_sent.fetch_add(1, std::memory_order_relaxed);
    }

    if (do_status && pub_status) {
      std_msgs::msg::String msg;
      msg.data = status_out;
      pub_status->publish(msg);
    }
  }
}

Publish2NavPointCloud::Publish2NavPointCloud(const std::string & config_path)
: impl_(std::make_unique<Impl>(config_path))
{
}

Publish2NavPointCloud::~Publish2NavPointCloud() = default;

bool Publish2NavPointCloud::enabled() const { return impl_ && impl_->on; }

bool Publish2NavPointCloud::obstacle_enabled() const
{
  return enabled() && impl_->opt.hz_obstacle > 0.0;
}

bool Publish2NavPointCloud::map_enabled() const { return enabled() && impl_->opt.hz_map > 0.0; }

bool Publish2NavPointCloud::status_enabled() const { return enabled(); }

void Publish2NavPointCloud::submit_obstacles(
  std::vector<ObstacleCloudPoint> points, const rclcpp::Time & stamp)
{
  if (!obstacle_enabled()) return;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->obs.swap(points);
  impl_->obs_stamp_pending = stamp;
  impl_->has_obs = true;
  impl_->cv.notify_all();
}

void Publish2NavPointCloud::submit_map(std::vector<MapCloudPoint> points, const rclcpp::Time & stamp)
{
  if (!map_enabled()) return;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->map.swap(points);
  impl_->map_stamp_pending = stamp;
  impl_->has_map = true;
  impl_->cv.notify_all();
}

void Publish2NavPointCloud::submit_status(std::string json)
{
  if (!status_enabled()) return;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->status = std::move(json);
  impl_->has_status = true;
  impl_->cv.notify_all();
}

double Publish2NavPointCloud::obstacle_hz() const
{
  return impl_ ? impl_->obs_hz.load(std::memory_order_relaxed) : 0.0;
}

size_t Publish2NavPointCloud::obstacle_sent() const
{
  return impl_ ? impl_->obs_sent.load(std::memory_order_relaxed) : 0;
}

size_t Publish2NavPointCloud::map_sent() const
{
  return impl_ ? impl_->map_sent.load(std::memory_order_relaxed) : 0;
}

}  // namespace io
