#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// 【改动】引入全向相机 PnP 胶水层（内部调用与主相机相同的 auto_aim::solve_pnp_armor）
#include "omni_pnp.hpp"

namespace omniperception
{
Decider::Decider(const std::string & config_path) : count_(0)
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ =
    (yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red : auto_aim::Color::blue;
  mode_ = yaml["mode"].as<double>();
}

io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos,
  const std::vector<OmniCameraConfig> & omni_configs)
{
  if (omni_configs.empty()) return {false, false, 0, 0};

  int n = static_cast<int>(omni_configs.size());
  if (count_ < 0 || count_ >= n) count_ = 0;

  auto & cfg = omni_configs[count_];

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  cfg.camera->read(img, timestamp);

  if (img.empty()) return io::Command{false, false, 0, 0};

  auto armors = yolo.detect(img);
  auto empty = armor_filter(armors);

  if (!empty) {
    // 【改动】传入当前帧尺寸：PnP 在无标定内参时需要用图像尺寸估算内参
    auto da = delta_angle(armors, cfg, img.size());

    tools::logger()->debug(
      "[omni camera {}] delta yaw:{:.2f}, target pitch:{:.2f}, armor number:{}, armor name:{}",
      count_, da[0], da[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    count_ = (count_ + 1) % n;

    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + da[0] / 57.3),
      tools::limit_rad(da[1] / 57.3)};
  }

  count_ = (count_ + 1) % n;
  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(const std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) return io::Command{false, false, 0, 0};

  for (const auto & dr : detection_queue) {
    if (dr.armors.empty()) continue;
    double yaw_deg = tools::limit_rad(dr.delta_yaw * 57.3);
    // if (std::abs(yaw_deg) > 360.0) {
    //   tools::logger()->warn("omni angle rejected: {:.1f}° out of range", yaw_deg);
    //   continue;
    // }
    tools::logger()->info(
      "omniperceptron find {}, delta yaw {:.2f}°",
      auto_aim::ARMOR_NAMES[dr.armors.front().name],
      tools::limit_rad(dr.delta_yaw * 57.3));
    return io::Command{true, false, dr.delta_yaw, dr.delta_pitch};
  }

  return io::Command{false, false, 0, 0};
}

void Decider::clear_angle_stack()
{
}

/*
  计算目标装甲板相对「云台正前方」的角度（单位：度）。
  输出约定与下位机的 command 一致：yaw 向左为正，pitch 向下为正
  （pitch 向下为正是因为主相机 Aimer 里 command.pitch = -世界系抬头 pitch）。
  下游 sentry_multithread 会把 yaw 加上云台当前世界系 yaw 得到世界系绝对角，
  因此本函数的接口约定没有改变，只是计算方式从「像素线性插值」换成了 PnP。

  【本次改动】
    旧实现（已废弃，保留在方案 B 作为兜底）：
      da[0] = mount_yaw + fov_h/2 - center_norm.x * fov_h
      da[1] = center_norm.y * fov_v - fov_v/2 + mount_pitch
    它的假设是「像素位置与角度成正比」，在视场边缘误差很大，而且完全没有
    利用装甲板的真实尺寸。现在改为调用主相机同款的 PnP：
      1. 装甲板 4 个角点 + 已知物理尺寸 -> solvePnP（SOLVEPNP_IPPE）
      2. 得到装甲板中心在「相机系」下的 3D 坐标
      3. 用相机->云台外参转到云台系
      4. atan2 求 yaw / pitch（与 tools::xyz2ypd 定义一致，向上为正）
      5. pitch 取负，转成「向下为正」的命令约定（与旧公式一致）

    内参/外参采用渐进式策略，方便「标定数据还没到位」时先跑起来：
      · 有 omni_camera_matrix_N        -> 用标定内参
      · 没有                           -> 用 fov_h/fov_v + 图像尺寸估一个针孔内参
      · 有 omni_R_camera2gimbal_N      -> 用标定外参
      · 没有                           -> 用 omni_mount_yaw/pitch_N 近似构造 R，t 置 0
    详见 omni_pnp.hpp 里的说明。
*/
Eigen::Vector2d Decider::delta_angle(
  const std::list<auto_aim::Armor> & armors, const OmniCameraConfig & cfg,
  const cv::Size & image_size)
{
  Eigen::Vector2d da = Eigen::Vector2d::Zero();
  if (armors.empty()) return da;

  const auto & armor = armors.front();

  // ------------------------- 方案 A：PnP -------------------------
  // 1) 内参：优先标定值，其次用 FOV 近似（占位方案）
  cv::Mat K = cfg.camera_matrix;
  bool using_approx_K = false;
  if (K.empty()) {
    K = approximate_camera_matrix(image_size, cfg.fov_h, cfg.fov_v);
    using_approx_K = true;
  }

  // 2) 畸变：没有标定值就当无畸变（空 Mat，solvePnP 会自动忽略）
  const cv::Mat D = cfg.distort_coeffs;

  // 3) 外参：优先标定值，其次用安装角近似（占位方案）
  Eigen::Matrix3d R_camera2gimbal = cfg.R_camera2gimbal;
  Eigen::Vector3d t_camera2gimbal = cfg.t_camera2gimbal;
  if (!cfg.has_extrinsic) {
    R_camera2gimbal = mount_to_R_camera2gimbal(cfg.mount_yaw, cfg.mount_pitch);
    t_camera2gimbal.setZero();
  }

  // 【大小yaw结构说明】本函数的外参 R_camera2gimbal / t_camera2gimbal 是
  // 「相机 -> 大yaw 旋转轴」，由配置的 tf.frames.omni_cam_N 查表得到（含平移）。
  // 大yaw 只有 yaw，所以这里算出的目标方向天然在大yaw 系，与 pitch/roll 无关；
  // 入口里叠加的必须是大yaw 的世界系 yaw（lower_yaw），不能用 Solver 的
  // R_gimbal2world（那是小yaw 板的姿态）。详见 omni_pnp.hpp 里的说明。

  // 4) 解算 + 几何合理性检查（检查在 solve_omni_armor 内部）
  if (!K.empty()) {
    const auto pnp = solve_omni_armor(armor, K, D, R_camera2gimbal, t_camera2gimbal);
    if (pnp.success) {
      // yaw：向左为正，与 command.yaw / 主相机 Aimer 的约定一致，直接用。
      da[0] = pnp.yaw_pitch_deg[0];

      // pitch：【符号转换】solve_omni_armor 给出的是「向上为正」的数学 pitch，
      // 而下位机 command.pitch 是「向下为正」（见 aimer.cpp 里的
      // `pitch = -(world_pitch + offset)`，注释写明「世界坐标系下 pitch 向上为负」）。
      // 旧 delta_angle 的兜底公式也是向下为正，所以这里取负号以保持行为一致。
      da[1] = -pnp.yaw_pitch_deg[1];

      // 调试日志：标定阶段用来对比 PnP 与旧线性方法、观察距离与重投影误差。
      // 注意日志里的 pitch 已经转换成了命令约定（向下为正）。
      tools::logger()->debug(
        "[omni pnp] dist={:.2f}m reproj={:.2f}px yaw={:.2f}° pitch_cmd={:.2f}° ({})", pnp.distance,
        pnp.reprojection_error, da[0], da[1], using_approx_K ? "approx-FOV" : "calibrated");
      return da;
    }
    tools::logger()->debug("[omni pnp] rejected by geometry check, fallback to FOV linear model");
  }

  // ------------------- 方案 B：旧的像素线性近似（兜底） -------------------
  // 只有在「连 fov 都没有」或「PnP 解被几何检查拒绝」时才会走到这里。
  // 这两个式子的输出本来就是 command 约定（yaw 左正、pitch 下正），保持不变。
  // TODO(可选): 标定完成后可以删除这个分支，避免两套角度来源混用。
  da[0] = cfg.mount_yaw + (cfg.fov_h / 2) - armor.center_norm.x * cfg.fov_h;
  da[1] = armor.center_norm.y * cfg.fov_v - cfg.fov_v / 2 + cfg.mount_pitch;
  return da;
}

/*
  装甲板的过滤，我们打3V3未必会遇见这么多的车，所以需要更改的处理。
  TODO: 里面的的得到无敌装甲板的逻辑并不明确，需要在后期更改。
*/
bool Decider::armor_filter(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return true;
  // 过滤非敌方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 25赛季没有5号装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::five; });
  // 不打工程
  // armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::two; });
  // 不打前哨站
  armors.remove_if(
    [&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::outpost; });

  // 过滤掉刚复活无敌的装甲板
  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(invincible_armor_.begin(), invincible_armor_.end(), a.name) !=
           invincible_armor_.end();
  });

  return armors.empty();
}

/*
  设置装甲板的优先级。
  TODO: 这里的优先级逻辑需要根据实际比赛情况进行调整，确保优先攻击最有价值的目标。
*/
void Decider::set_priority(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return;

  const PriorityMap & priority_map = (mode_ == MODE_ONE) ? mode1 : mode2;

  if (!armors.empty()) {
    for (auto & armor : armors) {
      armor.priority = priority_map.at(armor.name);
    }
  }
}

// armors 的过滤、优先级设置与排序已在 Perceptron 内完成（那里才有相机几何参数，
// 才能保证 delta_yaw/delta_pitch 与 armors.front() 始终指同一块装甲板；
// 若在这里再过滤一次，front() 可能被移除而使角度失效）。
// 这里只做两件事：丢弃空结果（否则下面的比较器会对空 list 取 front，属于未定义行为），
// 并按优先级对队列本身排序。
void Decider::sort(std::vector<DetectionResult> & detection_queue)
{
  detection_queue.erase(
    std::remove_if(
      detection_queue.begin(), detection_queue.end(),
      [](const DetectionResult & dr) { return dr.armors.empty(); }),
    detection_queue.end());

  std::sort(
    detection_queue.begin(), detection_queue.end(),
    [](const DetectionResult & a, const DetectionResult & b) {
      return a.armors.front().priority < b.armors.front().priority;
    });
}

Eigen::Vector4d Decider::get_target_info(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets)
{
  if (armors.empty() || targets.empty()) return Eigen::Vector4d::Zero();

  auto target = targets.front();

  for (const auto & armor : armors) {
    if (armor.name == target.name) {
      return Eigen::Vector4d{
        armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
        static_cast<double>(armor.name) + 1};  //避免歧义+1(详见通信协议)
    }
  }

  return Eigen::Vector4d::Zero();
}

void Decider::get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids)
{
  invincible_armor_.clear();

  if (invincible_enemy_ids.empty()) return;

  for (const auto & id : invincible_enemy_ids) {
    tools::logger()->info("invincible armor id: {}", id);
    invincible_armor_.push_back(auto_aim::ArmorName(id - 1));
  }
}

void Decider::get_auto_aim_target(
  std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target)
{
  if (auto_aim_target.empty()) return;

  std::vector<auto_aim::ArmorName> auto_aim_targets;

  for (const auto & target : auto_aim_target) {
    if (target <= 0 || static_cast<size_t>(target) > auto_aim::ARMOR_NAMES.size()) {
      tools::logger()->warn("Received invalid auto_aim target value: {}", int(target));
      continue;
    }
    auto_aim_targets.push_back(static_cast<auto_aim::ArmorName>(target - 1));
    tools::logger()->info("nav send auto_aim target is {}", auto_aim::ARMOR_NAMES[target - 1]);
  }

  if (auto_aim_targets.empty()) return;

  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(auto_aim_targets.begin(), auto_aim_targets.end(), a.name) ==
           auto_aim_targets.end();
  });
}

}  // namespace omniperception