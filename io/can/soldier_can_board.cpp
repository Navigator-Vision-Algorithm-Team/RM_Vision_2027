#include "soldier_can_board.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace io
{

SoldierCanBoard::SoldierCanBoard(Writer writer, Now now)
: writer_(std::move(writer)), now_(std::move(now))
{
}

bool SoldierCanBoard::fresh(TimePoint received_at, TimePoint now)
{
  return now >= received_at && now - received_at < kTimeout;
}

bool SoldierCanBoard::forward(std::uint8_t previous, std::uint8_t next)
{
  const auto distance = static_cast<std::uint8_t>(next - previous);
  return distance > 0 && distance < 128;
}

void SoldierCanBoard::reset_locked(SessionStatus status)
{
  status.id = session_.id + 1;
  status.rejected_pose_sequences = rejected_pose_sequences_;
  session_ = status;
  rejected_pose_sequences_ = 0;
  pose_.reset();
  history_.clear();
  chassis_.reset();
  robot_.reset();
  heat_.reset();
}

void SoldierCanBoard::reset()
{
  std::lock_guard lock(state_mutex_);
  SessionStatus status;
  status.reason = "explicit_reset";
  reset_locked(status);
}

std::uint64_t SoldierCanBoard::session() const
{
  return session_status().id;
}

SoldierCanBoard::SessionStatus SoldierCanBoard::session_status() const
{
  std::lock_guard lock(state_mutex_);
  return session_;
}

SoldierCanBoard::PoseRxStats SoldierCanBoard::take_pose_rx_stats()
{
  std::lock_guard lock(state_mutex_);
  return std::exchange(pose_rx_stats_, PoseRxStats{});
}

bool SoldierCanBoard::receive(std::uint32_t can_id, std::span<const std::uint8_t> payload)
{
  const auto telemetry = soldier_can::decode_telemetry(can_id, payload);
  if (!telemetry) return false;
  const auto now = now_();
  std::lock_guard lock(state_mutex_);

  if (const auto * pose = std::get_if<soldier_can::Pose>(&*telemetry)) {
    ++pose_rx_stats_.frames;
    if (last_pose_rx_ && now >= *last_pose_rx_) {
      const auto gap = now - *last_pose_rx_;
      if (gap < std::chrono::milliseconds(2)) ++pose_rx_stats_.gaps_under_2ms;
      if (gap >= kTimeout) ++pose_rx_stats_.gaps_at_least_100ms;
      pose_rx_stats_.max_gap_ms = std::max(
        pose_rx_stats_.max_gap_ms, std::chrono::duration<double, std::milli>(gap).count());
    }
    last_pose_rx_ = now;

    if (pose_) {
      const auto clock_step = static_cast<std::uint16_t>(pose->reference_ms - pose_->value.reference_ms);
      const bool timed_out = !fresh(pose_->received_at, now);
      const bool clock_restarted = clock_step >= 32768;
      if (!timed_out && !clock_restarted && !forward(pose_->value.sequence, pose->sequence)) {
        ++rejected_pose_sequences_;
        return false;
      }
      // 65535 -> 0 的正常回绕视为前进。时钟倒退会清空旧会话，
      // 但仅凭这两个循环计数无法唯一识别每一次电控重启。
      const char * reason = nullptr;
      if (timed_out) reason = "pose_timeout";
      else if (clock_restarted) reason = "pose_clock_backwards";
      else if (pose_->value.unfolded != pose->unfolded) reason = "unfolded_changed";
      else if (pose_->value.pose_valid && !pose->pose_valid) reason = "pose_became_invalid";
      if (reason) {
        SessionStatus status;
        status.reason = reason;
        status.rx_gap_ms =
          std::chrono::duration<double, std::milli>(now - pose_->received_at).count();
        status.previous_seq = pose_->value.sequence;
        status.incoming_seq = pose->sequence;
        status.previous_reference_ms = pose_->value.reference_ms;
        status.incoming_reference_ms = pose->reference_ms;
        reset_locked(status);
      }
    }
    pose_ = PoseSample{*pose, now, session_.id};
    if (pose->pose_valid) history_.push_back(*pose_);
    while (!history_.empty() &&
           (history_.size() > 128 || !fresh(history_.front().received_at, now))) {
      history_.pop_front();
    }
    return true;
  }
  if (const auto * chassis = std::get_if<soldier_can::Chassis>(&*telemetry))
    return store_newer(chassis_, *chassis, now);
  if (const auto * robot = std::get_if<soldier_can::RobotStatus>(&*telemetry)) {
    if (robot_ && fresh(robot_->received_at, now) &&
        !forward(robot_->value.sequence, robot->sequence)) return false;
    if (robot_) {
      const char * reason = nullptr;
      if (robot_->value.enabled != robot->enabled) reason = "robot_enabled_changed";
      else if (robot_->value.autoaim_requested != robot->autoaim_requested)
        reason = "autoaim_request_changed";
      else if (robot_->value.robot_id != robot->robot_id) reason = "robot_id_changed";
      if (reason) {
        // 解锁、自瞄请求或阵营变化后，等待新的姿态与图像解算。
        SessionStatus status;
        status.reason = reason;
        status.rx_gap_ms =
          std::chrono::duration<double, std::milli>(now - robot_->received_at).count();
        status.previous_seq = robot_->value.sequence;
        status.incoming_seq = robot->sequence;
        reset_locked(status);
      }
    }
    robot_ = Timed<soldier_can::RobotStatus>{*robot, now};
    return true;
  }
  return store_newer(heat_, std::get<soldier_can::Heat>(*telemetry), now);
}

std::optional<SoldierCanBoard::FramePose> SoldierCanBoard::pose_at(TimePoint image_timestamp) const
{
  std::lock_guard lock(state_mutex_);
  const auto now = now_();
  if (!pose_ || !pose_->value.pose_valid || !fresh(pose_->received_at, now) ||
      image_timestamp > now) return std::nullopt;
  const PoseSample * after = nullptr;
  for (auto sample = history_.rbegin(); sample != history_.rend(); ++sample) {
    if (!fresh(sample->received_at, now)) break;
    if (sample->received_at > image_timestamp) {
      after = &*sample;
      continue;
    }

    FramePose estimate{*sample, sample->value.yaw_rad, sample->value.pitch_raw_rad};
    if (after && sample->received_at < image_timestamp) {
      const double fraction =
        std::chrono::duration<double>(image_timestamp - sample->received_at).count() /
        std::chrono::duration<double>(after->received_at - sample->received_at).count();
      constexpr double turn = 2.0 * std::numbers::pi;
      // yaw 走最短弧，避免 +π 到 -π 时插到正前方。
      const double yaw_step = std::remainder(after->value.yaw_rad - estimate.yaw_rad, turn);
      estimate.yaw_rad = std::remainder(estimate.yaw_rad + fraction * yaw_step, turn);
      estimate.pitch_raw_rad = std::lerp(estimate.pitch_raw_rad, after->value.pitch_raw_rad, fraction);
    }
    return estimate;
  }
  return std::nullopt;
}

std::optional<SoldierCanBoard::Snapshot> SoldierCanBoard::snapshot() const
{
  std::lock_guard lock(state_mutex_);
  const auto now = now_();
  if (!pose_ || !fresh(pose_->received_at, now)) return std::nullopt;
  const auto sequence = pose_->value.sequence;
  return Snapshot{*pose_, matching(chassis_, sequence, now), matching(robot_, sequence, now),
                  matching(heat_, sequence, now)};
}

bool SoldierCanBoard::reference_valid(const PoseSample & reference, TimePoint now) const
{
  if (reference.session != session_.id || !fresh(reference.received_at, now) || !pose_ ||
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

SoldierCanBoard::SendResult SoldierCanBoard::send(
  soldier_can::Command command, const std::optional<PoseSample> & reference, SendAttempt * attempt)
{
  std::lock_guard send_lock(send_mutex_);
  std::lock_guard state_lock(state_mutex_);
  const auto now = now_();
  auto result = SendResult::sent;
  if (command.tracking && (!reference || !reference_valid(*reference, now))) {
    command.tracking = false;
    result = SendResult::stale_reference;
  }
  if (command.tracking &&
      (!pose_->value.autoaim_active || !pose_->value.unfolded ||
       !robot_ || !fresh(robot_->received_at, now) || !robot_->value.robot_status_valid ||
       !robot_->value.enabled || !robot_->value.autoaim_requested ||
       !robot_->value.bullet_speed_valid)) {
    command.tracking = false;
    result = SendResult::control_unavailable;
  }
  command.sequence = next_sequence_;
  command.reference_ms = command.tracking ? reference->value.reference_ms :
    (pose_ && fresh(pose_->received_at, now) ? pose_->value.reference_ms : 0);

  soldier_can::Payload payload;
  try {
    payload = soldier_can::encode_command(command);
  } catch (const std::invalid_argument &) {
    command.tracking = false;
    payload = soldier_can::encode_command(command);
    result = SendResult::invalid_command;
  }
  if (attempt) *attempt = SendAttempt{payload, now_()};
  try {
    if (!writer_) return SendResult::transport_error;
    writer_(soldier_can::kCommandId, payload);
  } catch (const std::exception &) {
    return SendResult::transport_error;
  }
  ++next_sequence_;
  return result;
}

}  // namespace io
