#ifndef IO__CAN__SOLDIER_CAN_BOARD_HPP
#define IO__CAN__SOLDIER_CAN_BOARD_HPP

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>

#include "soldier_can_protocol.hpp"

namespace io
{
class SoldierCanBoard
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Writer = std::function<void(std::uint32_t, const soldier_can::Payload &)>;
  using Now = std::function<TimePoint()>;
  static constexpr auto kTimeout = soldier_can::config::kTimeout;

  struct PoseSample
  {
    soldier_can::Pose value;
    TimePoint received_at;
    std::uint64_t session;
  };

  struct FramePose
  {
    PoseSample reference;  // 回传给 C 板的真实姿态样本
    double yaw_rad;
    double pitch_raw_rad;
  };

  struct Snapshot
  {
    PoseSample pose;
    // 只合并与姿态序号相同、且未过期的状态；丢帧时对应字段为空。
    std::optional<soldier_can::Chassis> chassis;
    std::optional<soldier_can::RobotStatus> robot;
    std::optional<soldier_can::Heat> heat;
  };

  enum class SendResult
  {
    sent,               // Queued to the CAN interface; not an MCU acknowledgement.
    stale_reference,    // Replaced the stale target with an idle command.
    invalid_command,    // Replaced invalid target angles with an idle command.
    transport_error,
    control_unavailable  // CAN 控制条件在发送前失效，已改发空闲帧。
  };

  struct SendAttempt
  {
    soldier_can::Payload payload;
    TimePoint started_at;
  };

  struct SessionStatus
  {
    std::uint64_t id = 0;
    const char * reason = "startup";
    double rx_gap_ms = 0;
    int previous_seq = -1;
    int incoming_seq = -1;
    int previous_reference_ms = -1;
    int incoming_reference_ms = -1;
    std::uint64_t rejected_pose_sequences = 0;
  };

  struct PoseRxStats
  {
    std::uint64_t frames = 0;
    std::uint64_t gaps_under_2ms = 0;
    std::uint64_t gaps_at_least_100ms = 0;
    double max_gap_ms = 0;
  };

  explicit SoldierCanBoard(const std::string & config_path);
  // 虚拟收发和确定性测试使用这个构造函数；writer 不应回调本对象的方法。
  explicit SoldierCanBoard(Writer writer, Now now = [] { return Clock::now(); });

  // 按本机接收时刻插值，并保留图像时刻之前的真实样本作为协议参考。
  std::optional<FramePose> pose_at(TimePoint image_timestamp) const;
  // 返回同组最新快照；调用方检查 pose_valid 和各状态有效位。
  std::optional<Snapshot> snapshot() const;
  std::uint64_t session() const;
  SessionStatus session_status() const;
  PoseRxStats take_pose_rx_stats();

  // reference 必须是本次图像解算使用的样本，序号和回显时间戳由通信类填写。
  // 上层以 50-100Hz 提交最新结果。pitch_raw_rad 应先完成编码器零点/方向转换。
  SendResult send(
    soldier_can::Command command, const std::optional<PoseSample> & reference = {},
    SendAttempt * attempt = nullptr);

  bool receive(std::uint32_t can_id, std::span<const std::uint8_t> payload);
  // 确认电控重启时调用；协议中没有明确的启动标识。
  void reset();

private:
  template <typename T>
  struct Timed
  {
    T value;
    TimePoint received_at;
  };

  template <typename T>
  static bool store_newer(std::optional<Timed<T>> & previous, const T & value, TimePoint now)
  {
    if (previous && fresh(previous->received_at, now) &&
        !forward(previous->value.sequence, value.sequence)) return false;
    previous = Timed<T>{value, now};
    return true;
  }

  template <typename T>
  static std::optional<T> matching(
    const std::optional<Timed<T>> & sample, std::uint8_t sequence, TimePoint now)
  {
    if (sample && sample->value.sequence == sequence && fresh(sample->received_at, now))
      return sample->value;
    return std::nullopt;
  }

  static bool fresh(TimePoint received_at, TimePoint now);
  static bool forward(std::uint8_t previous, std::uint8_t next);
  void reset_locked(SessionStatus status);
  bool reference_valid(const PoseSample & reference, TimePoint now) const;
  class DamiaoCan;

  Writer writer_;
  Now now_;
  mutable std::mutex state_mutex_;
  std::mutex send_mutex_;
  std::optional<PoseSample> pose_;
  std::deque<PoseSample> history_;
  std::optional<Timed<soldier_can::Chassis>> chassis_;
  std::optional<Timed<soldier_can::RobotStatus>> robot_;
  std::optional<Timed<soldier_can::Heat>> heat_;
  SessionStatus session_;
  std::uint64_t rejected_pose_sequences_ = 0;
  PoseRxStats pose_rx_stats_;
  std::optional<TimePoint> last_pose_rx_;
  std::uint8_t next_sequence_ = 0;
  // 最先销毁传输层，确保接收线程退出时缓存和锁仍然有效。
  std::shared_ptr<DamiaoCan> can_;
};

}  // namespace io

#endif  // IO__CAN__SOLDIER_CAN_BOARD_HPP
