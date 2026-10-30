#ifndef IO__SERIAL_BOARD_HPP
#define IO__SERIAL_BOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "io/command.hpp"
#include "io/serial/gimbal/packet.hpp"
#include "io/sim/common.hpp"
#include "io/sim/recorder.hpp"
#include "io/sim/transport.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{

// 0x502 一帧姿态量（弧度）
//   大yaw(lower) 只有 yaw；pitch/roll 属于小yaw(upper)，即主相机所在的那块板。
struct ImuState
{
  double pitch = 0.0;      // 小yaw 板俯仰，低头为正
  double roll = 0.0;       // 小yaw 板横滚
  double upper_yaw = 0.0;  // 小yaw 世界系绝对 yaw，[0, 2π)
  double lower_yaw = 0.0;  // 大yaw 世界系绝对 yaw，[0, 2π)
};

class SerialBoard
{
public:
  double bullet_speed;

  explicit SerialBoard(const std::string & config_path);
  ~SerialBoard();

  // 一帧 IMU 姿态（按图像时间戳插值）。双yaw 平台的各坐标系由调用方用 tf 树复合。
  ImuState imu_state_at(std::chrono::steady_clock::time_point timestamp);

  // 兼容接口（单yaw 兵种）：Rz(lower_yaw)·Ry(pitch)·Rx(roll)。
  // 双yaw 平台请改用 imu_state_at()，因为大yaw 只有 yaw、pitch/roll 属于小yaw。
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;
  void send(NavCommand command) const;

  // 云台关节反馈回调：收到一帧 CRC 正确的 0x502 就回调一次（yaw_rad 为已转成
  // (-π, π] 的关节角，pitch_rad 恒为 0，因为大yaw 轴没有俯仰自由度）。
  // 回调发生在接收线程内，按真实帧率触发；不会把上一帧旧值重复喂给上层。
  using GimbalJointCallback = std::function<void(double yaw_rad, double pitch_rad)>;
  void set_gimbal_joint_callback(GimbalJointCallback callback);

  // Game / match state from MCU
  GameStatus game_status() const;
  RobotStatus robot_status() const;
  bool is_game_started() const;

  // Reset signal from MCU (bassPacket.reset)
  bool reset_pending();

private:
  struct IMUData
  {
    ImuState state;  // 四个原始角度，按时间戳插值后再复合出各坐标系姿态
    std::chrono::steady_clock::time_point timestamp;
  };

  // 把队列里的新样本搬进 data_ahead_ / data_behind_
  void update_imu_samples();

  // 数据源：真实串口 / 内置虚拟下位机 / 原始字节回放，由配置决定，解析逻辑完全一致
  sim::IByteTransport::Ptr transport_;
  sim::SourceMode source_mode_ = sim::SourceMode::Real;
  std::shared_ptr<sim::SerialRecorder> recorder_;

  std::string device_name_;
  uint32_t baud_rate_ = 921600;

  std::thread receive_thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex send_mutex_;
  mutable std::mutex status_mutex_;

  // 关节反馈订阅方（目前是 io::ROS2 的 JointState 发布节点）
  mutable std::mutex joint_callback_mutex_;
  GimbalJointCallback joint_callback_;

  tools::ThreadSafeQueue<IMUData> queue_{1000};
  IMUData data_ahead_;
  IMUData data_behind_;

  // Robot state from MCU
  GameStatus game_status_{};
  RobotStatus robot_status_{};
  std::atomic<bool> game_started_{false};
  std::atomic<bool> reset_pending_{false};
  bool use_referee_system_ = false;
  int game_start_progress_threshold_ = 4;

  bool read(uint8_t * buffer, size_t size);
  void receiveThread();
  void reconnect();
  void handleGameStatus(const GameStatusPacket & pkt);
  void handleGameRobotStatus(const GameRobotStatusPacket & pkt);
  void handleBass(const bassPacket & pkt);
};

}  // namespace io

#endif  // IO__SERIAL_BOARD_HPP
