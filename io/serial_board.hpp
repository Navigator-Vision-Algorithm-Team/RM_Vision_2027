#ifndef IO__SERIAL_BOARD_HPP
#define IO__SERIAL_BOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "io/command.hpp"
#include "io/gimbal/packet.hpp"
#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{

class SerialBoard
{
public:
  double bullet_speed;

  explicit SerialBoard(const std::string & config_path);
  ~SerialBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  void send(Command command) const;

  // Game / match state from MCU
  GameStatus game_status() const;
  RobotStatus robot_status() const;
  bool is_game_started() const;

  // Reset signal from MCU (bassPacket.reset)
  bool reset_pending();

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  mutable serial::Serial serial_;
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
