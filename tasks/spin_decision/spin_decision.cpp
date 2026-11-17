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
    // 命中有效目标：立即退出自旋并清空确认计数，不把"停不下来"的滞回留给调用方
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
  }

  const double elapsed = std::chrono::duration<double>(now - start_).count();
  const double phase = kTwoPi * kPitchFrequencyHz * elapsed;

  // 基于当前实际 yaw 叠加增量：下位机无条件执行角度，因此下一帧的 gimbal_pos 已经跟上
  out.yaw = tools::limit_rad(kSpinYawStep + gimbal_pos[0]);
  // 绕平视上下点头；pitch=0 附近摆动，避免扫地面或抬头看天
  out.pitch = kPitchAmplitude * std::sin(phase);
  return out;
}

}  // namespace spin_decision
