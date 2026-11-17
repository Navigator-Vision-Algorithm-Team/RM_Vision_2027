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
 *   3) pitch 用正弦波绕平视上下点头，幅度与频率是本文件的常量，调参改这里。
 */

#include <chrono>
#include <cstdint>

#include <Eigen/Geometry>

namespace spin_decision
{

// 一次自旋决策的输出：世界系绝对角，单位 rad
struct SpinOutput
{
  double yaw = 0;    // 世界系绝对 yaw，rad
  double pitch = 0;  // 世界系绝对 pitch，rad；向上为负
};

class SpinDecision
{
public:
  // 功能：推进一帧自旋，产出搜索姿态
  // 输入：
  //   now         当前帧采集时刻，steady_clock；用于驱动 pitch 波形相位
  //   gimbal_pos  当前世界系绝对姿态 yaw/pitch/roll，rad
  // 输出：本帧要下发的 {yaw, pitch}
  // 备注：首次调用以 now 为波形起点； yaw 每帧在前一帧实际角度上叠加固定增量
  SpinOutput update(
    std::chrono::steady_clock::time_point now, const Eigen::Vector3d & gimbal_pos);

  // 功能：结束本轮自旋，复位波形相位
  // 输入：无
  // 输出：无
  // 备注：命中有效目标时由状态机调用；下次 update() 会从 pitch=0 重新开始
  void reset() { start_ = TimePoint{}; }

private:
  using TimePoint = std::chrono::steady_clock::time_point;  // 单调时钟

  TimePoint start_{};  // 本轮自旋起点；默认构造表示尚未开始
};

}  // namespace spin_decision

#endif  // TASKS__SPIN_DECISION__SPIN_DECISION_HPP
