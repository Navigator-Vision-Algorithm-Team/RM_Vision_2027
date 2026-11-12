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

/*
 * 0x502 一帧姿态量（弧度）。
 *
 * 大yaw(lower) 只有 yaw；pitch/roll 属于小yaw(upper)，即主相机所在的那块板。
 */
struct ImuState
{
  double pitch = 0.0;      // 小yaw 板俯仰，低头为正
  double roll = 0.0;       // 小yaw 板横滚
  double upper_yaw = 0.0;  // 小yaw 世界系绝对 yaw，[0, 2π)
  double lower_yaw = 0.0;  // 大yaw 世界系绝对 yaw，[0, 2π)
};

/*
 * 与下位机（MCU）的串口通信封装。
 *
 * 上行：0xA5 帧头分帧，CRC8 校验帧头、CRC16 校验帧体。解析 0x502 IMU 姿态、
 *       0x503 底盘速度、0x504 按键/拨杆、0x505 基础状态，以及裁判系统转发帧。
 * 下行：0x0402 自瞄指令（pitch / yaw / shoot）、0x0405 导航速度指令。
 *
 * 数据源 real / fake / replay 统一走 sim::IByteTransport，分帧、校验、打包逻辑完全相同，
 * 上层业务不需要区分当前是实车还是仿真。
 *
 * 线程模型：构造时启动接收线程，析构时置 quit_ 并 join。接收线程把 IMU 样本压入
 * queue_，由 imu_state_at() 取用并插值；状态类 getter 均加锁，可在任意线程调用。
 */
class SerialBoard
{
public:
  /* 弹速（m/s），构造时从配置读取，供弹道解算使用；不随 0x0402 下发 */
  double bullet_speed;

  /* 读取配置、打开数据源并启动接收线程 */
  explicit SerialBoard(const std::string & config_path);
  ~SerialBoard();

  /* 一帧 IMU 姿态（按图像时间戳插值）。双yaw 平台的各坐标系由调用方用 tf 树复合 */
  ImuState imu_state_at(std::chrono::steady_clock::time_point timestamp);

  /*
   * 兼容接口（单yaw 兵种）：Rz(lower_yaw)·Ry(pitch)·Rx(roll)。
   * 双yaw 平台请改用 imu_state_at()，因为大yaw 只有 yaw、pitch/roll 属于小yaw。
   */
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  /* 组 0x0402 帧下发自瞄指令；报文中的 shoot 位 = control && shoot */
  void send(Command command) const;

  /* 组 0x0405 帧下发导航速度指令；vy 在写入报文前取反 */
  void send(NavCommand command) const;

  /*
   * 云台关节反馈回调：收到一帧 CRC 正确的 0x502 就回调一次（yaw_rad 为已转成
   * (-π, π] 的关节角，pitch_rad 恒为 0，因为大yaw 轴没有俯仰自由度）。
   * 回调发生在接收线程内，按真实帧率触发；不会把上一帧旧值重复喂给上层。
   */
  using GimbalJointCallback = std::function<void(double yaw_rad, double pitch_rad)>;
  void set_gimbal_joint_callback(GimbalJointCallback callback);

  /* 最近一帧比赛状态（裁判系统转发帧） */
  GameStatus game_status() const;

  /* 最近一帧机器人状态：血量、热量上限、底盘功率、三路供电 */
  RobotStatus robot_status() const;

  /*
   * 导航转发用的裁判状态快照：一次加锁取出 0x0001 / 0x0201 中导航需要的三个量，
   * 附带「是否收到过对应帧」的有效位（用于避免把未初始化值当成真实数据发布）。
   */
  RefereeState referee_state() const;

  /* 比赛是否已开始；未启用裁判系统时恒为 true */
  bool is_game_started() const;

  /*
   * 读取并清除「下位机请求复位」标志（由 0x505 的 reset 位置位）。
   * 语义是读后清零，不要连续调用两次。
   */
  bool reset_pending();

private:
  struct IMUData
  {
    ImuState state;  // 四个原始角度，按时间戳插值后再复合出各坐标系姿态
    std::chrono::steady_clock::time_point timestamp;
  };

  /* 把队列里的新样本搬进 data_ahead_ / data_behind_ */
  void update_imu_samples();

  /* 数据源：真实串口 / 内置虚拟下位机 / 原始字节回放，由配置决定，解析逻辑完全一致 */
  sim::IByteTransport::Ptr transport_;
  sim::SourceMode source_mode_ = sim::SourceMode::Real;
  std::shared_ptr<sim::SerialRecorder> recorder_;

  std::string device_name_;
  uint32_t baud_rate_ = 921600;

  std::thread receive_thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex send_mutex_;
  mutable std::mutex status_mutex_;

  /* 关节反馈订阅方（目前是 io::ROS2 的 JointState 发布节点） */
  mutable std::mutex joint_callback_mutex_;
  GimbalJointCallback joint_callback_;

  tools::ThreadSafeQueue<IMUData> queue_{1000};
  IMUData data_ahead_;
  IMUData data_behind_;

  /* 下位机上报的比赛与机器人状态，接收线程写入，getter 读取 */
  GameStatus game_status_{};
  RobotStatus robot_status_{};
  /* 「是否收到过对应帧」：只置位不清零，供裁判状态转发侧判断数据是否可用 */
  std::atomic<bool> game_status_received_{false};
  std::atomic<bool> robot_status_received_{false};
  std::atomic<bool> game_started_{false};
  std::atomic<bool> reset_pending_{false};
  bool use_referee_system_ = false;
  int game_start_progress_threshold_ = 4;

  /* 从数据源读满 size 字节；超时或失败返回 false */
  bool read(uint8_t * buffer, size_t size);

  /* 接收线程主循环：分帧 -> CRC 校验 -> 按 cmd_id 分发 */
  void receiveThread();

  /* 数据源异常时重建连接 */
  void reconnect();

  /* 0x0001 比赛状态 */
  void handleGameStatus(const GameStatusPacket & pkt);

  /* 0x0201 机器人状态 */
  void handleGameRobotStatus(const GameRobotStatusPacket & pkt);

  /* 0x505 基础状态，目前只取其中的复位位 */
  void handleBass(const bassPacket & pkt);
};

}  // namespace io

#endif  // IO__SERIAL_BOARD_HPP
