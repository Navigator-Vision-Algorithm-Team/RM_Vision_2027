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
#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class SerialBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;

  explicit SerialBoard(const std::string & config_path);
  ~SerialBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  void send(Command command) const;

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

  tools::ThreadSafeQueue<IMUData> queue_{1000};
  IMUData data_ahead_;
  IMUData data_behind_;

  bool read(uint8_t * buffer, size_t size);
  void receiveThread();
  void reconnect();
};

}  // namespace io

#endif  // IO__SERIAL_BOARD_HPP
