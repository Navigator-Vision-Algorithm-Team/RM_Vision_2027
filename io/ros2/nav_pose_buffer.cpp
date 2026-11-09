#include "nav_pose_buffer.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#include "tools/logger.hpp"

namespace io
{

NavPoseBuffer::NavPoseBuffer(const std::string & config_path)
{
  if (config_path.empty()) return;

  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(config_path);
  } catch (const std::exception & e) {
    tools::logger()->warn("[NavPose] 读取配置失败（{}），导航位姿缓冲不启用", e.what());
    return;
  }

  const auto node_yaml = yaml["nav_pose"];
  if (!node_yaml) return;  // 整段缺失 = 不启用，静默（与 obstacle_cloud 一致）

  topic_ = node_yaml["topic"] ? node_yaml["topic"].as<std::string>()
                              : std::string("/red_standard_robot1/aft_mapped_to_init");
  buffer_s_ = (node_yaml["buffer_ms"] ? node_yaml["buffer_ms"].as<double>() : 500.0) / 1000.0;
  timeout_s_ = (node_yaml["timeout_ms"] ? node_yaml["timeout_ms"].as<double>() : 200.0) / 1000.0;
  if (buffer_s_ <= 0.0) buffer_s_ = 0.5;
  if (timeout_s_ <= 0.0) timeout_s_ = 0.2;

  try {
    // 已有节点（io::ROS2）时直接复用同一个 context，不能重复 init
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      own_context_ = true;
    }

    node_ = std::make_shared<rclcpp::Node>("nav_pose_listener");

    // C1 契约：BEST_EFFORT + KEEP_LAST 10。逐点发布频率极高，KEEP_LAST 10
    // 即可覆盖插值窗口；丢弃旧样本比撑爆 DDS 队列更安全。
    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(std::move(msg)); });

    enabled_ = true;
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    spin_thread_ = std::thread([this] { exec_->spin(); });

    tools::logger()->info(
      "[NavPose] 订阅 {}（buffer {:.0f} ms，timeout {:.0f} ms）", topic_, buffer_s_ * 1000.0,
      timeout_s_ * 1000.0);
  } catch (const std::exception & e) {
    tools::logger()->warn("[NavPose] ROS2 初始化失败（{}），导航位姿缓冲不启用", e.what());
    enabled_ = false;
  }
}

NavPoseBuffer::~NavPoseBuffer()
{
  sub_.reset();
  // 只 cancel 自己的 executor：全局的 rclcpp::shutdown 由 io::ROS2 统一负责，
  // 不能在这里调（同进程还有 Publish2Nav / 点云发布等节点在跑）。
  if (exec_) exec_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  exec_.reset();
  node_.reset();
  if (own_context_ && rclcpp::ok()) rclcpp::shutdown();
}

void NavPoseBuffer::on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;

  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
  if (!std::isfinite(quat.w()) || !std::isfinite(quat.x()) || !std::isfinite(quat.y()) ||
      !std::isfinite(quat.z()))
    return;
  if (quat.norm() < 1e-6) return;
  quat.normalize();

  Sample sample;
  sample.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  sample.T = Eigen::Isometry3d::Identity();
  sample.T.linear() = quat.toRotationMatrix();
  sample.T.translation() = Eigen::Vector3d(p.x, p.y, p.z);

  std::lock_guard<std::mutex> lock(mtx_);

  // 逐点发布时样本基本是单调递增的；偶发乱序就用插入排序的位置放进去，
  // 保证 buf_ 始终按时间升序（插值依赖这一点）。
  if (buf_.empty() || sample.stamp_ns >= buf_.back().stamp_ns) {
    buf_.push_back(sample);
  } else {
    auto it = std::lower_bound(
      buf_.begin(), buf_.end(), sample.stamp_ns,
      [](const Sample & s, int64_t v) { return s.stamp_ns < v; });
    if (it != buf_.end() && it->stamp_ns == sample.stamp_ns) {
      *it = sample;
    } else {
      buf_.insert(it, sample);
    }
  }

  prune_locked();
}

void NavPoseBuffer::prune_locked()
{
  if (buf_.empty()) return;
  const int64_t oldest =
    buf_.back().stamp_ns - static_cast<int64_t>(buffer_s_ * 1e9);
  while (buf_.size() > 2 && buf_.front().stamp_ns < oldest) buf_.pop_front();
}

bool NavPoseBuffer::lookup(const rclcpp::Time & stamp, Eigen::Isometry3d & T_ci_body) const
{
  const int64_t t = stamp.nanoseconds();
  const int64_t timeout_ns = static_cast<int64_t>(timeout_s_ * 1e9);

  std::lock_guard<std::mutex> lock(mtx_);
  if (buf_.empty()) {
    misses_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  // 比最新样本还新：容忍 timeout 内的外推（取最近端点），否则判为缺失
  if (t >= buf_.back().stamp_ns) {
    if (t - buf_.back().stamp_ns > timeout_ns) {
      misses_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    T_ci_body = buf_.back().T;
    return true;
  }

  // 比最旧样本还旧：同样只容忍 timeout 内的取端点
  if (t <= buf_.front().stamp_ns) {
    if (buf_.front().stamp_ns - t > timeout_ns) {
      misses_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    T_ci_body = buf_.front().T;
    return true;
  }

  // 找到第一个 stamp > t 的样本，与前一个样本插值
  auto hi = std::lower_bound(
    buf_.begin(), buf_.end(), t, [](const Sample & s, int64_t v) { return s.stamp_ns < v; });
  if (hi->stamp_ns == t) {
    T_ci_body = hi->T;
    return true;
  }
  auto lo = hi - 1;

  const double span = static_cast<double>(hi->stamp_ns - lo->stamp_ns);
  const double alpha = span > 0.0 ? static_cast<double>(t - lo->stamp_ns) / span : 0.0;

  const Eigen::Quaterniond q0(lo->T.linear());
  const Eigen::Quaterniond q1(hi->T.linear());
  Eigen::Quaterniond q = q0.slerp(alpha, q1);

  T_ci_body = Eigen::Isometry3d::Identity();
  T_ci_body.linear() = q.toRotationMatrix();
  T_ci_body.translation() = (1.0 - alpha) * lo->T.translation() + alpha * hi->T.translation();
  return true;
}

bool NavPoseBuffer::healthy() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (buf_.empty()) return false;
  const int64_t now_ns = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
  return static_cast<double>(now_ns - buf_.back().stamp_ns) / 1e9 <= timeout_s_;
}

size_t NavPoseBuffer::sample_count() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return buf_.size();
}

}  // namespace io
