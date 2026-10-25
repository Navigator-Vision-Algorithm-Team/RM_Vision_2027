#ifndef IO__SERIAL_BOARD_HPP
#define IO__SERIAL_BOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
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

class SerialBoard
{
public:
  double bullet_speed;

  explicit SerialBoard(const std::string & config_path);
  ~SerialBoard();

  // 大yaw（= gimbal 系，IMU 与全向相机所在轴）的世界系姿态，按时间戳 slerp 对齐图像
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  // 小yaw 相对大yaw 的转角 psi（弧度，(-π, π]），同样按时间戳插值。
  // 仅哨兵（双yaw）有意义；单yaw 兵种的 0x502 里 upper_yaw 不使用，返回 0。
  // 用法：solver.set_small_yaw_offset(psi_now - psi_calib)，见 auto_aim::Solver。
  double small_yaw_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;
  void send(NavCommand command) const;

  // Game / match state from MCU
  GameStatus game_status() const;
  RobotStatus robot_status() const;
  bool is_game_started() const;

  // Reset signal from MCU (bassPacket.reset)
  bool reset_pending();

private:
  struct IMUData
  {
    Eigen::Quaterniond q;  // 大yaw -> 世界（由 lower_yaw / pitch / roll 构造）
    double psi = 0.0;      // 小yaw 相对大yaw 的转角（(-π, π]），单yaw 兵种恒为 0
    std::chrono::steady_clock::time_point timestamp;
  };

  // 把队列里的新样本搬进 data_ahead_ / data_behind_（imu_at 与 small_yaw_at 共用）
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
