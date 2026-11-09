#include "visual_depth.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>

#include "feature_tracker.hpp"
#include "ground_projection.hpp"
#include "io/vision_cloud_types.hpp"
#include "pcd_writer.hpp"
#include "tools/logger.hpp"
#include "triangulator.hpp"

#ifdef SP_WITH_ROS2
#include "io/ros2/publish2nav_pointcloud.hpp"
#include "io/ros2/ros_time.hpp"
#endif

namespace visual_depth
{
namespace
{

double to_sec(std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration<double>(t.time_since_epoch()).count();
}

// R = Rz(yaw)·Ry(pitch)·Rx(roll)，输入为度
Eigen::Matrix3d rpy_deg_to_R(double roll_deg, double pitch_deg, double yaw_deg)
{
  const double k = M_PI / 180.0;
  return (Eigen::AngleAxisd(yaw_deg * k, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch_deg * k, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll_deg * k, Eigen::Vector3d::UnitX()))
    .toRotationMatrix();
}

// 读 [x,y,z] 与 [roll,pitch,yaw]（度），语义：p_parent = R * p_child + t。
// 缺键时保持传入的默认值（默认单位阵）。
Eigen::Isometry3d read_extrinsic(const YAML::Node & node, const char * t_key, const char * rpy_key)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (!node) return T;

  if (node[t_key]) {
    const auto v = node[t_key].as<std::vector<double>>();
    if (v.size() == 3) T.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
  }
  if (node[rpy_key]) {
    const auto v = node[rpy_key].as<std::vector<double>>();
    if (v.size() == 3) T.linear() = rpy_deg_to_R(v[0], v[1], v[2]);
  }
  return T;
}

bool read_intrinsics(
  const YAML::Node & yaml, int camera_1based, CameraIntrinsics & K, cv::Mat & camera_matrix,
  cv::Mat & distort_coeffs)
{
  const std::string suffix = std::to_string(camera_1based);
  const auto km = yaml["omni_camera_matrix_" + suffix];
  if (!km) return false;

  const auto v = km.as<std::vector<double>>();
  if (v.size() != 9) return false;

  cv::Mat k(3, 3, CV_64F);
  for (int i = 0; i < 9; i++) k.at<double>(i / 3, i % 3) = v[static_cast<size_t>(i)];
  camera_matrix = k;

  K.fx = v[0];
  K.cx = v[2];
  K.fy = v[4];
  K.cy = v[5];
  if (!(K.fx > 1.0) || !(K.fy > 1.0)) return false;

  const auto dm = yaml["omni_distort_coeffs_" + suffix];
  if (dm) {
    const auto d = dm.as<std::vector<double>>();
    if (d.size() == 5) {
      cv::Mat m(1, 5, CV_64F);
      for (int i = 0; i < 5; i++) m.at<double>(0, i) = d[static_cast<size_t>(i)];
      distort_coeffs = m;
    }
  }
  return true;
}

}  // namespace

struct VisualDepth::Impl
{
  // ---------------- 配置 ----------------
  bool on = false;
  bool tri_on = false;
  bool ba_on = true;
  int camera_1based = 3;   // 与 omni_cam_N / tf frame 编号一致（1 基）
  int camera_0based = 2;   // CameraResult.camera_id（0 基）
  int omni_count = 0;
  cv::Size resolution{640, 480};
  double max_range_map = 12.0;
  int min_triangulated_points = 20;
  double map_hz = 1.0;

  // 位姿链上的两个恒定外参：T_ci<-yaw = T_ci<-body · T_body<-lidar · T_lidar<-yaw
  Eigen::Isometry3d T_body_lidar = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_lidar_yaw = Eigen::Isometry3d::Identity();

  CameraIntrinsics K;
  cv::Mat camera_matrix;
  cv::Mat distort_coeffs;
  bool have_intrinsics = false;

  FeatureTrackerConfig tracker_cfg;
  TriConfig tri_cfg;
  BaConfig ba_cfg;
  GroundProjectionConfig ground_cfg;
  DynamicFilterConfig df_cfg;

  // ---------------- 算法 ----------------
  std::unique_ptr<FeatureTracker> tracker;
  std::unique_ptr<Triangulator> triangulator;
  std::unique_ptr<GroundProjector> ground;
  std::unique_ptr<DynamicFilter> dynamic_filter;
  std::unique_ptr<PcdWriter> pcd;

  // ---------------- 输入缓存 ----------------
  cv::Mat frame_color;
  std::chrono::steady_clock::time_point frame_stamp{};
  double frame_sec = 0.0;
  bool have_frame = false;
  Eigen::Isometry3d T_yaw_cam = Eigen::Isometry3d::Identity();  // 相机 -> 大yaw

  std::vector<auto_aim::multithread::ObstaclePoint> obstacles;
  std::chrono::steady_clock::time_point obstacle_stamp{};
  bool have_obstacle_stamp = false;

  // 最近若干帧的导航位姿（按帧缓存：每帧只查一次 nav_pose，观测按 stamp 复用）
  struct FramePose
  {
    double stamp = 0.0;
    bool valid = false;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  // T_camera_init<-body
  };
  std::deque<FramePose> pose_ring;

  // ---------------- 建图累积 ----------------
  // C4 话题（publish_hz_map > 0）和离线 PCD 落盘共用这批点，任一路开启才累积
  std::vector<io::MapCloudPoint> map_pending;
  bool want_map = false;
  double last_map_emit = 0.0;
  bool map_emitted_once = false;

  // ---------------- 统计 ----------------
  size_t frames_ok = 0;
  size_t pose_miss = 0;
  size_t triangulated = 0;
  size_t ba_accepted = 0;
  size_t dynamic_rejected = 0;
  size_t ground_pts = 0;
  size_t rejected_this_frame = 0;
  double reproj_median = 0.0;
  double stamp_lag_ms = 0.0;
  bool last_pose_ok = false;

#ifdef SP_WITH_ROS2
  std::unique_ptr<io::Publish2NavPointCloud> publisher;
#endif

  void load(const std::string & config_path);
  void process_frame(const PoseLookup & pose_lookup);
  bool lookup_pose(double stamp, Eigen::Isometry3d & T) const;
  void publish_obstacles(std::chrono::steady_clock::time_point main_stamp);
  void publish_status(std::chrono::steady_clock::time_point main_stamp);
};

void VisualDepth::Impl::load(const std::string & config_path)
{
  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(config_path);
  } catch (const std::exception & e) {
    tools::logger()->warn("[VisualDepth] 读取配置失败（{}），视觉补盲关闭", e.what());
    return;
  }

  if (yaml["omni_camera_count"]) omni_count = yaml["omni_camera_count"].as<int>();

  // ---- 点云出口（C3/C4/C5）：整段缺失或 enable=false 就什么都不做 ----
  bool cloud_on = false;
  const auto cloud = yaml["obstacle_cloud"];
  if (cloud) {
    cloud_on = cloud["enable"] ? cloud["enable"].as<bool>() : false;
    if (cloud["publish_hz_map"]) map_hz = cloud["publish_hz_map"].as<double>();
    if (cloud["max_range_map"]) max_range_map = cloud["max_range_map"].as<double>();
  }

  // ---- 三角化（L1/L2） ----
  const auto tri = yaml["visual_triangulation"];
  if (tri) {
    tri_on = tri["enable"] ? tri["enable"].as<bool>() : false;
    if (tri["camera_id"]) camera_1based = tri["camera_id"].as<int>();
    if (tri["resolution"]) {
      const auto v = tri["resolution"].as<std::vector<int>>();
      if (v.size() == 2 && v[0] > 0 && v[1] > 0) resolution = cv::Size(v[0], v[1]);
    }
    if (tri["max_corners"]) tracker_cfg.max_corners = tri["max_corners"].as<int>();
    if (tri["roi_bottom_ratio"]) tracker_cfg.roi_bottom_ratio = tri["roi_bottom_ratio"].as<double>();
    if (tri["min_baseline"]) tri_cfg.min_baseline = tri["min_baseline"].as<double>();
    if (tri["min_parallax_deg"]) tri_cfg.min_parallax_deg = tri["min_parallax_deg"].as<double>();
    if (tri["max_reproj_px"]) tri_cfg.max_reproj_px = tri["max_reproj_px"].as<double>();
    if (tri["max_range"]) tri_cfg.max_depth = tri["max_range"].as<double>();
    if (tri["min_triangulated_points"]) {
      min_triangulated_points = tri["min_triangulated_points"].as<int>();
    }
  }

  const auto ba = yaml["visual_ba"];
  if (ba) {
    ba_on = ba["enable"] ? ba["enable"].as<bool>() : true;
    ba_cfg.enable = ba_on;
    if (ba["window_size"]) ba_cfg.window_size = ba["window_size"].as<int>();
    if (ba["max_iterations"]) ba_cfg.max_iterations = ba["max_iterations"].as<int>();
    if (ba["huber_threshold_px"]) ba_cfg.huber_threshold_px = ba["huber_threshold_px"].as<double>();
    if (ba["min_observations"]) tri_cfg.min_observations = ba["min_observations"].as<int>();
    if (ba["mode"]) {
      const auto mode = ba["mode"].as<std::string>();
      if (mode != "points_only") {
        tools::logger()->warn(
          "[VisualDepth] visual_ba.mode='{}' 不支持，本期只实现 points_only（M-A），按 M-A 运行",
          mode);
      }
    }
  }

  const auto gp = yaml["visual_ground_projection"];
  if (gp) {
    ground_cfg.enable = gp["enable"] ? gp["enable"].as<bool>() : true;
    if (gp["ground_z"]) ground_cfg.ground_z = gp["ground_z"].as<double>();
    if (gp["max_range"]) ground_cfg.max_range = gp["max_range"].as<double>();
  }

  const auto df = yaml["dynamic_filter"];
  if (df) {
    df_cfg.enable = df["enable"] ? df["enable"].as<bool>() : true;
    if (df["consistency_window"]) df_cfg.consistency_window = df["consistency_window"].as<int>();
    if (df["max_position_spread"]) df_cfg.max_spread = df["max_position_spread"].as<double>();
    if (df["reject_streak"]) df_cfg.reject_streak = df["reject_streak"].as<int>();
  }

  // 位姿链上的恒定外参（与导航 point_lio 的 extrinsic_T/R + 雷达安装位姿对齐）
  const auto nav_pose = yaml["nav_pose"];
  T_body_lidar = read_extrinsic(nav_pose, "body_lidar_t", "body_lidar_rpy_deg");
  T_lidar_yaw = read_extrinsic(nav_pose, "lidar_yaw_t", "lidar_yaw_rpy_deg");

  camera_0based = camera_1based - 1;

  if (!cloud_on && !tri_on) {
    tools::logger()->info("[VisualDepth] obstacle_cloud / visual_triangulation 均未开启，视觉补盲关闭");
    return;
  }
  on = true;

  // ---- C3/C4/C5 发布（独立线程，红线二） ----
#ifdef SP_WITH_ROS2
  publisher = std::make_unique<io::Publish2NavPointCloud>(config_path);
#else
  if (cloud_on) {
    tools::logger()->warn("[VisualDepth] 编译时未找到 ROS2，点云出口不可用");
  }
#endif

  if (!tri_on) {
    tools::logger()->info("[VisualDepth] 三角化关闭：只透传 PnP 点（比赛配置）");
    return;
  }

  if (camera_1based <= 0 || camera_1based > omni_count) {
    tools::logger()->warn(
      "[VisualDepth] visual_triangulation.camera_id={} 不在 1..{} 范围内，三角化关闭",
      camera_1based, omni_count);
    tri_on = false;
    return;
  }

  if (!read_intrinsics(yaml, camera_1based, K, camera_matrix, distort_coeffs)) {
    tools::logger()->warn(
      "[VisualDepth] 读不到 omni_camera_matrix_{}（需 9 个数），三角化关闭", camera_1based);
    tri_on = false;
    return;
  }
  have_intrinsics = true;

  tracker = std::make_unique<FeatureTracker>(tracker_cfg, camera_matrix, distort_coeffs);
  triangulator = std::make_unique<Triangulator>(K, tri_cfg, ba_cfg);
  ground = std::make_unique<GroundProjector>(ground_cfg);
  dynamic_filter = std::make_unique<DynamicFilter>(df_cfg);

  // 出口 C：离线 PCD 落盘
  if (cloud && cloud["pcd_path"]) {
    const auto path = cloud["pcd_path"].as<std::string>();
    if (!path.empty()) pcd = std::make_unique<PcdWriter>(true, path);
  }

  // C4 话题与 PCD 落盘任一路开启就要累积建图点；都关掉时完全不占内存
  want_map = (map_hz > 0.0) || (pcd != nullptr);

  tools::logger()->info(
    "[VisualDepth] 三角化开启：omni_cam_{}（{}x{}），BA {}，窗口 {} 帧，地面投影 {}",
    camera_1based, resolution.width, resolution.height, ba_on ? "on" : "off", ba_cfg.window_size,
    ground_cfg.enable ? "on" : "off");
}

bool VisualDepth::Impl::lookup_pose(double stamp, Eigen::Isometry3d & T) const
{
  // 精确匹配：观测的 stamp 与帧 stamp 来自同一个 double，不会出现浮点漂移
  for (const auto & p : pose_ring) {
    if (p.stamp == stamp && p.valid) {
      T = p.T;
      return true;
    }
  }
  return false;
}

void VisualDepth::Impl::process_frame(const PoseLookup & pose_lookup)
{
  if (!have_frame || !tri_on || !tracker) return;

  const double cur_sec = frame_sec;
  have_frame = false;

  // ---- 1) 本帧导航位姿 ----
  Eigen::Isometry3d T_ci_body = Eigen::Isometry3d::Identity();
  const bool pose_ok = pose_lookup && pose_lookup(frame_stamp, T_ci_body);

  FramePose fp;
  fp.stamp = cur_sec;
  fp.valid = pose_ok;
  fp.T = T_ci_body;
  pose_ring.push_back(fp);
  const size_t keep = static_cast<size_t>(std::max(4, ba_cfg.window_size + 2));
  while (pose_ring.size() > keep) pose_ring.pop_front();

  last_pose_ok = pose_ok;
  if (!pose_ok) pose_miss++;

  // ---- 2) 特征跟踪（每帧都要推，三角化失败也不能断轨迹） ----
  cv::Mat gray;
  if (frame_color.channels() == 3) {
    cv::cvtColor(frame_color, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = frame_color;
  }
  tracker->push(gray, camera_0based, cur_sec);
  frame_color.release();

  if (!pose_ok) {
    // §5.8：C1 中断时停止三角化，只发 PnP 点。轨迹继续跟，位姿回来就能立刻接上。
    return;
  }

  // ---- 3) 世界位姿链：T_ci<-cam = T_ci<-body · T_body<-lidar · T_lidar<-yaw · T_yaw<-cam ----
  // C4 的点直接以 camera_init 系发布（与导航 Point-LIO 同一套世界系），
  // 所以只有三角化需要这条链，输出不需要再转回 gimbal_yaw。
  const Eigen::Isometry3d T_ci_yaw = T_ci_body * T_body_lidar * T_lidar_yaw;
  const Eigen::Isometry3d T_ci_cam = T_ci_yaw * T_yaw_cam;

  // ---- 4) 退化检测：窗口内基线太小 => 纯旋转，切地面平面法 ----
  double max_baseline = 0.0;
  for (const auto & p : pose_ring) {
    const Eigen::Isometry3d T_ci_yaw_i = p.T * T_body_lidar * T_lidar_yaw;
    max_baseline = std::max(
      max_baseline, (T_ci_yaw_i.translation() - T_ci_yaw.translation()).norm());
  }
  const bool pure_rotation = max_baseline < tri_cfg.min_baseline;

  const int window = std::max(2, ba_cfg.window_size);

  if (pure_rotation) {
    // ---- 4a) 地面平面反投影兜底（不需要基线） ----
    ground_pts = 0;
    if (ground && ground->enabled()) {
      std::vector<cv::Point2f> cur_px;
      for (const auto & t : tracker->tracks()) {
        if (t.obs.empty() || t.obs.back().stamp != cur_sec) continue;
        cur_px.push_back(t.obs.back().px);
      }
      std::vector<Eigen::Vector3d> world;
      world.reserve(cur_px.size());
      ground->project(cur_px, K, T_ci_cam.linear(), T_ci_cam.translation(), world);
      ground_pts = world.size();

      for (const auto & p : world) {
        io::MapCloudPoint mp;
        mp.p = p.cast<float>();  // 已经是 camera_init 系（C4 的 frame_id）
        mp.quality = 0.2f;       // 地面法的深度误差比三角化大，质量分给低
        mp.observations = 1;
        mp.parallax_deg = 0.f;
        mp.consistency = 1.0f;
        if (want_map) map_pending.push_back(mp);
      }
    }
    triangulated = 0;
    ba_accepted = 0;
    rejected_this_frame = 0;
    return;
  }

  // ---- 4b) 三角化 + M-A 局部 BA（每点独立的 3×3 系统） ----
  std::vector<double> reproj_list;
  std::vector<io::MapCloudPoint> frame_points;  // 本帧产出，够数才并入 map_pending
  size_t valid = 0;
  size_t ba_count = 0;
  size_t rejected = 0;
  ground_pts = 0;

  for (const auto & track : tracker->tracks()) {
    if (track.obs.size() < 2) continue;
    if (track.obs.back().stamp != cur_sec) continue;  // 本帧没有新观测

    // 取窗口内最近 window 次观测，并配上各自的位姿
    const size_t begin = track.obs.size() > static_cast<size_t>(window)
                           ? track.obs.size() - static_cast<size_t>(window)
                           : 0;
    std::vector<cv::Point2f> px;
    std::vector<PoseSample> poses;
    px.reserve(track.obs.size() - begin);
    poses.reserve(track.obs.size() - begin);

    for (size_t i = begin; i < track.obs.size(); i++) {
      const auto & obs = track.obs[i];
      Eigen::Isometry3d T;
      if (!lookup_pose(obs.stamp, T)) continue;
      const Eigen::Isometry3d T_ci_yaw_i = T * T_body_lidar * T_lidar_yaw;
      const Eigen::Isometry3d T_ci_cam_i = T_ci_yaw_i * T_yaw_cam;

      PoseSample sample;
      sample.stamp = obs.stamp;
      sample.R = T_ci_cam_i.linear();
      sample.t = T_ci_cam_i.translation();
      px.push_back(obs.px);
      poses.push_back(sample);
    }
    if (px.size() < 2) continue;

    const TriResult r = triangulator->triangulate(px, poses);
    if (!r.valid) {
      rejected++;
      continue;
    }

    // 多帧一致性：动态点（落在敌人/队友身上）在这里被剔除
    double spread = 0.0;
    if (dynamic_filter->update(track.id, r.p_world, &spread)) {
      dynamic_rejected++;
      continue;
    }

    // 量程按「到最新观测的相机位置」算（camera_init 原点是上电位置，不能用 norm）
    if ((r.p_world - poses.back().t).norm() > max_range_map) continue;

    const double consistency = 1.0 / (1.0 + std::max(0.0, spread));
    io::MapCloudPoint mp;
    mp.p = r.p_world.cast<float>();  // camera_init 系（C4 的 frame_id）
    mp.quality = static_cast<float>(1.0 / std::max(0.1, r.reproj_px));
    mp.observations = r.observations;
    mp.parallax_deg = static_cast<float>(r.parallax_deg);
    mp.consistency = static_cast<float>(consistency);
    if (want_map) frame_points.push_back(mp);

    reproj_list.push_back(r.reproj_px);
    valid++;
    if (ba_on && r.observations >= tri_cfg.min_observations) ba_count++;
  }

  rejected_this_frame = rejected;

  // 有效点太少（噪声太大）时整帧丢弃，避免污染 PCD / 重定位先验
  if (valid >= static_cast<size_t>(std::max(0, min_triangulated_points))) {
    if (want_map) map_pending.insert(map_pending.end(), frame_points.begin(), frame_points.end());
    triangulated = valid;
    ba_accepted = ba_count;
  } else {
    triangulated = 0;
    ba_accepted = 0;
  }

  if (!reproj_list.empty()) {
    std::sort(reproj_list.begin(), reproj_list.end());
    reproj_median = reproj_list[reproj_list.size() / 2];
  }
}

void VisualDepth::Impl::publish_obstacles(std::chrono::steady_clock::time_point main_stamp)
{
  const auto stamp = have_obstacle_stamp ? obstacle_stamp : main_stamp;
  have_obstacle_stamp = false;

#ifdef SP_WITH_ROS2
  if (publisher) {
    std::vector<io::ObstacleCloudPoint> out;
    out.reserve(obstacles.size());
    for (const auto & p : obstacles) {
      io::ObstacleCloudPoint cp;
      cp.x = static_cast<float>(p.p_gimbal.x());
      cp.y = static_cast<float>(p.p_gimbal.y());
      cp.z = static_cast<float>(p.p_gimbal.z());
      cp.intensity = 1.0f;  // 具体值由 io 侧按 obstacle_cloud.intensity 覆盖
      out.push_back(cp);
    }
    publisher->submit_obstacles(std::move(out), io::rostime::to_ros(stamp));

    // C4：按 publish_hz_map 攒批（1 Hz），攒满一批发一次；同时喂 PCD 落盘
    const double now_s = to_sec(main_stamp);
    const double period = map_hz > 0.0 ? 1.0 / map_hz : 1.0;
    if (want_map && !map_pending.empty() &&
        (!map_emitted_once || now_s - last_map_emit >= period)) {
      // 点数上限由 io 侧统一裁剪
      if (map_hz > 0.0) publisher->submit_map(map_pending, io::rostime::to_ros(main_stamp));

      if (pcd) {
        // 落盘字段与 C4 话题同口径（x y z intensity normal_* curvature）
        std::vector<PcdPoint> file_points;
        file_points.reserve(map_pending.size());
        for (const auto & mp : map_pending) {
          PcdPoint q;
          q.p = mp.p;
          q.quality = mp.quality;
          q.observations = mp.observations;
          q.parallax_deg = mp.parallax_deg;
          q.consistency = mp.consistency;
          file_points.push_back(q);
        }
        if (pcd->full()) {
          tools::logger()->warn("[VisualDepth] 建图点数已达上限，后续点不再落盘");
        } else {
          pcd->add(file_points);
        }
      }

      map_pending.clear();
      last_map_emit = now_s;
      map_emitted_once = true;
    }

    // 位姿缺失时告诉导航「这一帧的点只有 PnP，没有三角化」
    publish_status(main_stamp);
  }
#else
  (void)main_stamp;
#endif
  obstacles.clear();
}

void VisualDepth::Impl::publish_status(std::chrono::steady_clock::time_point main_stamp)
{
#ifdef SP_WITH_ROS2
  if (!publisher) return;

  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
  stamp_lag_ms = static_cast<double>(now.nanoseconds() - io::rostime::to_ros(main_stamp).nanoseconds()) / 1e6;

  nlohmann::json j;
  j["cameras"] = omni_count;
  j["frames_ok"] = frames_ok;
  j["pose_miss"] = pose_miss;
  j["features_total"] = tri_on && tracker ? tracker->observation_count() : 0;
  j["triangulated"] = triangulated;
  j["ba_accepted"] = ba_accepted;
  j["dynamic_rejected"] = dynamic_rejected;
  j["ground_pts"] = ground_pts;
  j["reproj_px_median"] = reproj_median;
  j["publish_hz"] = publisher->obstacle_hz();
  j["stamp_lag_ms"] = stamp_lag_ms;
  j["degraded"] = tri_on && !last_pose_ok;
  j["degraded_reason"] = (tri_on && !last_pose_ok) ? "nav_pose_miss" : "";
  j["ba_mode"] = "points_only";
  j["triangulation"] = tri_on;

  publisher->submit_status(j.dump());
#else
  (void)main_stamp;
#endif
}

// =============================================================================
//  对外接口
// =============================================================================

VisualDepth::VisualDepth(const std::string & config_path) : impl_(std::make_unique<Impl>())
{
  impl_->load(config_path);
}

VisualDepth::~VisualDepth() = default;

bool VisualDepth::enabled() const { return impl_ && impl_->on; }

bool VisualDepth::triangulation_enabled() const { return impl_ && impl_->tri_on; }

int VisualDepth::keep_image_camera_id() const
{
  return (impl_ && impl_->tri_on) ? impl_->camera_0based : -1;
}

void VisualDepth::add_obstacle_points(
  const std::vector<auto_aim::multithread::ObstaclePoint> & points,
  std::chrono::steady_clock::time_point stamp)
{
  if (!impl_ || !impl_->on) return;
  impl_->obstacles.insert(impl_->obstacles.end(), points.begin(), points.end());
  if (!impl_->have_obstacle_stamp || stamp > impl_->obstacle_stamp) {
    impl_->obstacle_stamp = stamp;
    impl_->have_obstacle_stamp = true;
  }
}

void VisualDepth::add_frame(
  int camera_id, const cv::Mat & color, std::chrono::steady_clock::time_point stamp,
  const Eigen::Matrix3d & R_cam2yaw, const Eigen::Vector3d & t_cam2yaw)
{
  if (!impl_ || !impl_->tri_on) return;
  if (camera_id != impl_->camera_0based || color.empty()) return;

  // 统一到工作分辨率：内参按 resolution 标定（omni_camera_matrix_N），
  // 图像不是这个尺寸时缩放到它，K 不需要任何缩放修正。
  if (impl_->resolution.width > 0 && color.size() != impl_->resolution) {
    cv::Mat resized;
    cv::resize(color, resized, impl_->resolution, 0, 0, cv::INTER_AREA);
    impl_->frame_color = resized;
  } else {
    impl_->frame_color = color;  // 浅拷贝：mat 是引用计数，下一帧再覆盖
  }
  impl_->frame_stamp = stamp;
  impl_->frame_sec = to_sec(stamp);
  impl_->have_frame = true;

  impl_->T_yaw_cam = Eigen::Isometry3d::Identity();
  impl_->T_yaw_cam.linear() = R_cam2yaw;
  impl_->T_yaw_cam.translation() = t_cam2yaw;
}

void VisualDepth::finish(
  std::chrono::steady_clock::time_point main_stamp, const PoseLookup & pose_lookup)
{
  if (!impl_ || !impl_->on) return;

  impl_->frames_ok++;
  impl_->process_frame(pose_lookup);
  impl_->publish_obstacles(main_stamp);
}

}  // namespace visual_depth
