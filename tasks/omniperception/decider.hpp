#ifndef OMNIPERCEPTION__DECIDER_HPP
#define OMNIPERCEPTION__DECIDER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <iostream>
#include <list>
#include <unordered_map>
#include <vector>

#include "detection.hpp"
#include "io/camera.hpp"
#include "io/command.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace omniperception
{
class Decider
{
public:
  Decider(const std::string & config_path);

  io::Command decide(
    auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos,
    const std::vector<OmniCameraConfig> & omni_configs);

  io::Command decide(const std::vector<DetectionResult> & detection_queue);

  // 【改动】新增 image_size 参数：PnP 需要知道图像尺寸才能在没有标定内参时
  //         用 FOV 估算内参（cx/cy 与焦距）。默认空 Size 时只影响近似内参的
  //         生成，不影响已标定内参的使用。
  // 返回值：相对云台正前方的 (yaw, pitch)，单位「度」，与下位机 command 同约定：
  //         yaw 向左为正；pitch 向下为正（内部会把 PnP 的「向上为正」取负）。
  //         调用方（omni_jobs 的推理线程）仍然会 /57.3 转成弧度。
  Eigen::Vector2d delta_angle(
    const std::list<auto_aim::Armor> & armors, const OmniCameraConfig & cfg,
    const cv::Size & image_size = cv::Size());

  bool armor_filter(std::list<auto_aim::Armor> & armors);

  void set_priority(std::list<auto_aim::Armor> & armors);
  //对队列中的每一个DetectionResult进行过滤，同时将DetectionResult排序
  void sort(std::vector<DetectionResult> & detection_queue);

  Eigen::Vector4d get_target_info(
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets);

  void get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids);

  void get_auto_aim_target(
    std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target);

  // Clear angle stack — call when main camera re-acquires target so stale
  // omni angles don't interfere with active tracking.
  void clear_angle_stack();

private:
  int mode_;
  int count_;

  auto_aim::Color enemy_color_;
  std::vector<auto_aim::ArmorName> invincible_armor_;  //无敌状态机器人编号,英雄为1，哨兵为6

  // 定义ArmorName到ArmorPriority的映射类型
  using PriorityMap = std::unordered_map<auto_aim::ArmorName, auto_aim::ArmorPriority>;

  const PriorityMap mode1 = {
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::forth},
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::fifth},
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::fifth},
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::fifth}};

  const PriorityMap mode2 = {
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::third}};
};

enum PriorityMode
{
  MODE_ONE = 1,
  MODE_TWO
};

}  // namespace omniperception

#endif