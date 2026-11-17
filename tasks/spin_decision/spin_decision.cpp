/*
 * 自旋决策的实现：固定步长的 yaw 扫描 + 正弦波 pitch 点头。
 *
 * 数据流：update() 用当前实际 yaw 作为基准叠加扫描增量，用本轮起点算点头相位。
 *
 * 注意：契约见 spin_decision.hpp，本文件只写实现要点。
 */
#include "spin_decision.hpp"

#include <cmath>

#include "tools/math_tools.hpp"

namespace spin_decision
{
namespace
{
// 每帧 yaw 增量，rad；实际转速 ≈ 增量 × 帧率（120fps 下约 0.4 圈/秒）
constexpr double kSpinYawStep = 0.2;

// pitch 点头幅度，rad；0.12 rad ≈ 6.9°，足够覆盖装甲板高度而不扫地面
constexpr double kPitchAmplitude = 0.12;

// pitch 波形频率，Hz；0.5 Hz 表示 2 秒一个完整上下周期
constexpr double kPitchFrequencyHz = 0.5;

constexpr double kTwoPi = 6.28318530717958647692;

// 进入自旋前的确认帧数：连续这么多帧没有有效瞄准角才启动
constexpr int kConfirmFrames = 5;
}  // namespace

SpinOutput SpinDecision::update(TimePoint now, const Eigen::Vector3d & gimbal_pos, bool has_aim)
{
  SpinOutput out;

  if (has_aim) {
    // 命中有效目标：退出自旋、清零确认计数，并把**当前姿态记为失瞄锚点**。
    // 下一轮自旋将从这里接续（yaw 继续转、pitch 围绕这里点头），不回零位。
    anchor_yaw_ = gimbal_pos[0];
    anchor_pitch_ = gimbal_pos[1];
    has_anchor_ = true;
    miss_count_ = 0;
    spinning_ = false;
    start_ = TimePoint{};
    // 本帧角度由状态机的目标角决定，这里只返回当前姿态占位
    out.yaw = gimbal_pos[0];
    out.pitch = gimbal_pos[1];
    return out;
  }

  // 无有效瞄准角：累计确认帧。不足 5 帧时**不更新角度**，原样返回当前姿态，
  // 调用方拿到的 yaw/pitch 与上一拍一致，不会看到任何自旋增量或波形推进。
  ++miss_count_;
  if (miss_count_ < kConfirmFrames) {
    out.yaw = gimbal_pos[0];
    out.pitch = gimbal_pos[1];
    return out;
  }

  if (!spinning_) {
    spinning_ = true;
    start_ = now;  // 波形从第 5 帧开始，避免确认期内相位空转

    // 首次启动且从未命中过目标：用当前姿态兜底建立锚点
    if (!has_anchor_) {
      anchor_yaw_ = gimbal_pos[0];
      anchor_pitch_ = gimbal_pos[1];
      has_anchor_ = true;
    }
    // 从失瞄点接续，不从世界系零位重新开始
    yaw_cursor_ = anchor_yaw_;
  }

  const double elapsed = std::chrono::duration<double>(now - start_).count();
  const double phase = kTwoPi * kPitchFrequencyHz * elapsed;

  // yaw 从锚点逐帧累加；确认期内下发的就是锚点姿态，因此与云台实际朝向连续
  yaw_cursor_ = tools::limit_rad(yaw_cursor_ + kSpinYawStep);
  out.yaw = yaw_cursor_;
  // pitch 围绕失瞄角度上下点头；第一帧相位为 0，正好停在锚点 pitch 上
  out.pitch = tools::limit_rad(anchor_pitch_ + kPitchAmplitude * std::sin(phase));
  return out;
}

}  // namespace spin_decision
