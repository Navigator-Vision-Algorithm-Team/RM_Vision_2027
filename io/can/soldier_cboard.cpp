#include "soldier_cboard.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace io
{

SoldierCBoard::SoldierCBoard(Writer writer, Now now)
: writer_(std::move(writer)), now_(std::move(now))
{
}

bool SoldierCBoard::fresh(TimePoint received_at, TimePoint now)
{
  return now >= received_at && now - received_at < kTimeout;
}

bool SoldierCBoard::forward(std::uint8_t previous, std::uint8_t next)
{
  const auto distance = static_cast<std::uint8_t>(next - previous);
  return distance > 0 && distance < 128;
}

void SoldierCBoard::reset_locked()
{
  ++session_;
  pose_.reset();
  history_.clear();
  chassis_.reset();
  robot_.reset();
  heat_.reset();
}

void SoldierCBoard::reset()
{
  std::lock_guard lock(state_mutex_);
  reset_locked();
}

std::uint64_t SoldierCBoard::session() const
{
  std::lock_guard lock(state_mutex_);
  return session_;
}

bool SoldierCBoard::receive(std::uint32_t can_id, std::span<const std::uint8_t> payload)
{
  const auto telemetry = minipc::decode_telemetry(can_id, payload);
  if (!telemetry) return false;
  const auto now = now_();
  std::lock_guard lock(state_mutex_);

  if (const auto * pose = std::get_if<minipc::Pose>(&*telemetry)) {
    if (pose_) {
      const auto clock_step = static_cast<std::uint16_t>(pose->reference_ms - pose_->value.reference_ms);
      const bool timed_out = !fresh(pose_->received_at, now);
      const bool clock_restarted = clock_step >= 32768;
      if (!timed_out && !clock_restarted && !forward(pose_->value.sequence, pose->sequence))
        return false;
      // 65535 -> 0 的正常回绕视为前进。时钟倒退会清空旧会话，
      // 但仅凭这两个循环计数无法唯一识别每一次电控重启。
      if (timed_out || clock_restarted ||
          pose_->value.unfolded != pose->unfolded ||
          (pose_->value.pose_valid && !pose->pose_valid)) {
        reset_locked();
      }
    }
    pose_ = PoseSample{*pose, now, session_};
    if (pose->pose_valid) history_.push_back(*pose_);
    while (!history_.empty() &&
           (history_.size() > 128 || !fresh(history_.front().received_at, now))) {
      history_.pop_front();
    }
    return true;
  }
  if (const auto * chassis = std::get_if<minipc::Chassis>(&*telemetry))
    return store_newer(chassis_, *chassis, now);
  if (const auto * robot = std::get_if<minipc::RobotStatus>(&*telemetry)) {
    if (robot_ && fresh(robot_->received_at, now) &&
        !forward(robot_->value.sequence, robot->sequence)) return false;
    if (robot_ && (robot_->value.enabled != robot->enabled ||
                   robot_->value.autoaim_requested != robot->autoaim_requested)) {
      // 解锁/自瞄请求变化后，等待新的姿态与图像解算。
      reset_locked();
    }
    robot_ = Timed<minipc::RobotStatus>{*robot, now};
    return true;
  }
  return store_newer(heat_, std::get<minipc::Heat>(*telemetry), now);
}

std::optional<SoldierCBoard::PoseSample> SoldierCBoard::pose_at(TimePoint image_timestamp) const
{
  const auto now = now_();
  std::lock_guard lock(state_mutex_);
  if (!pose_ || !pose_->value.pose_valid || !fresh(pose_->received_at, now) ||
      image_timestamp > now) return std::nullopt;
  for (auto sample = history_.rbegin(); sample != history_.rend(); ++sample) {
    if (sample->received_at <= image_timestamp && fresh(sample->received_at, now) &&
        image_timestamp - sample->received_at < kTimeout) return *sample;
  }
  return std::nullopt;
}

std::optional<SoldierCBoard::Snapshot> SoldierCBoard::snapshot() const
{
  const auto now = now_();
  std::lock_guard lock(state_mutex_);
  if (!pose_ || !fresh(pose_->received_at, now)) return std::nullopt;
  const auto sequence = pose_->value.sequence;
  return Snapshot{*pose_, matching(chassis_, sequence, now), matching(robot_, sequence, now),
                  matching(heat_, sequence, now)};
}

bool SoldierCBoard::reference_valid(const PoseSample & reference, TimePoint now) const
{
  if (reference.session != session_ || !fresh(reference.received_at, now) || !pose_ ||
      !pose_->value.pose_valid || !fresh(pose_->received_at, now)) return false;
  const auto board_age = static_cast<std::uint16_t>(
    pose_->value.reference_ms - reference.value.reference_ms);
  if (std::chrono::milliseconds(board_age) + (now - pose_->received_at) >= kTimeout) return false;
  return std::any_of(history_.begin(), history_.end(), [&](const PoseSample & sample) {
    return sample.received_at == reference.received_at &&
           sample.value.sequence == reference.value.sequence &&
           sample.value.reference_ms == reference.value.reference_ms;
  });
}

SoldierCBoard::SendResult SoldierCBoard::send(
  minipc::Command command, const std::optional<PoseSample> & reference)
{
  std::lock_guard send_lock(send_mutex_);
  const auto now = now_();
  std::lock_guard state_lock(state_mutex_);
  auto result = SendResult::sent;
  if (command.tracking && (!reference || !reference_valid(*reference, now))) {
    command.tracking = false;
    result = SendResult::stale_reference;
  }
  command.sequence = next_sequence_;
  command.reference_ms = command.tracking ? reference->value.reference_ms :
    (pose_ && fresh(pose_->received_at, now) ? pose_->value.reference_ms : 0);

  minipc::Payload payload;
  try {
    payload = minipc::encode_command(command);
  } catch (const std::invalid_argument &) {
    command.tracking = false;
    payload = minipc::encode_command(command);
    result = SendResult::invalid_command;
  }
  try {
    if (!writer_) return SendResult::transport_error;
    writer_(minipc::kCommandId, payload);
  } catch (const std::exception &) {
    return SendResult::transport_error;
  }
  ++next_sequence_;
  return result;
}

}  // namespace io
