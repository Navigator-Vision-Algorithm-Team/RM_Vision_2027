#ifndef TASKS__SPIN_DECISION__SPIN_DECISION_HPP
#define TASKS__SPIN_DECISION__SPIN_DECISION_HPP

/*
 * 自旋决策：当没有任何有效瞄准角时，统一产出一条"边转边点头"的搜索姿态。
 *
 * 数据流：
 *   状态机判定本帧无有效瞄准角 → SpinDecision::update() → {yaw, pitch}
 *   → 状态机 finalize() 写入最终 Command → 中转进程下发 0x0402。
 *
 * 依赖与边界：
 *   只依赖标准库、Eigen 与 tools 的角度工具；不依赖 Tracker / Decider / Aimer，
 *   也不读配置文件——自旋是无条件兜底行为，不设开关。
 *
 * 线程：无状态共享锁，实例由状态机独占，只在主循环线程调用。
 *
 * 使用注意：
 *   1) 命中任何有效目标后必须调用 reset()，否则下次自旋会接着上次的点头相位；
 *   2) yaw 基于"当前世界系 yaw + 固定增量"，与下位机无条件执行角度的语义一致；
 *   3) pitch 用正弦波绕平视上下点头，幅度与频率是本文件的常量，调参改这里；
 *   4) 5 帧短滞回是本模块内的常量，不设开关、不进配置。
 */

#include <chrono>
#include <cstdint>

#include <Eigen/Geometry>

namespace spin_decision
{

// 一次自旋决策的输出：世界系绝对角，单位 rad
struct SpinOutput
{
  double yaw = 0;    // 世界系绝对 yaw，rad；确认期内保持当前值不更新
  double pitch = 0;  // 世界系绝对 pitch，rad；确认期内保持当前值不更新
};

class SpinDecision
{
public:
  // 功能：推进一帧自旋决策，内含 5 帧短滞回
  // 输入：
  //   now         当前帧采集时刻，steady_clock；用于驱动 pitch 波形相位
  //   gimbal_pos  当前世界系绝对姿态 yaw/pitch/roll，rad
  //   has_aim     本帧状态机是否产出了有效瞄准角
  // 输出：本帧应使用的 {yaw, pitch}；连续丢失不足 5 帧时原样返回 gimbal_pos 的角度
  // 备注：连续 5 帧 has_aim=false 才启动自旋更新；一旦 has_aim=true 立即退出并清零计数。
  //       **所有关于"是否更新"的判定都在本函数内完成**，调用方不做帧数检查。
  //       非对称设计：进入慢、退出快，避免目标出现后仍混入自旋角造成抖动。
  SpinOutput update(
    std::chrono::steady_clock::time_point now, const Eigen::Vector3d & gimbal_pos, bool has_aim);

private:
  using TimePoint = std::chrono::steady_clock::time_point;  // 单调时钟

  TimePoint start_{};    // 本轮自旋起点；默认构造表示尚未开始
  int miss_count_ = 0;   // 连续无有效瞄准角的帧数，达到 kConfirmFrames 才启动自旋
  bool spinning_ = false;  // 当前是否处于自旋状态
};

}  // namespace spin_decision

#endif  // TASKS__SPIN_DECISION__SPIN_DECISION_HPP
