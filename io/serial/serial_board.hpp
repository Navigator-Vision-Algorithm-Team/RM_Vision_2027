#ifndef IO__SERIAL_BOARD_HPP
#define IO__SERIAL_BOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
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
   * 下位机上报的机器人 ID，**唯一来源是 0x0502 的 roboid**。
   * 0x0505 的 roboid 与它是下位机侧的同一份数据，0x0201 的 robot_id 是裁判转发字段，
   * 两者都不参与阵营判定。0 表示还没收到。
   * 编号表见下面的 is_red_robot_id / is_blue_robot_id。
   */
  uint8_t robot_id() const;

  /*
   * 诊断用：已收到的 0x0502 帧数，以及最近一帧原始 roboid（含 0，不做校验）。
   * 用途是把「ID 拿不到」拆成两种可区分的情况：
   *   frame_count == 0        -> 0x0502 帧根本没到（固件版本 / CRC / 包长）
   *   frame_count > 0 且 raw == 0 -> 帧到了但下位机没填 roboid
   */
  uint64_t imu_frame_count() const { return imu_frame_count_.load(); }
  uint8_t last_raw_roboid() const { return last_raw_roboid_.load(); }

  // 已成功解析的 0x0001/0x0100 帧数（CRC 通过才计数）。
  // 用来判断「启用裁判系统但下位机根本没转发」——那会让 is_game_started()
  // 永远为 false，主相机与全向都不推理。
  uint64_t game_status_frame_count() const { return game_status_frame_count_.load(); }

  /*
   * 导航转发用的裁判状态快照：一次加锁取出 0x0001 / 0x0201 中导航需要的三个量，
   * 附带「是否收到过对应帧」的有效位（用于避免把未初始化值当成真实数据发布）。
   *
   * 配置 `test_nav: true` 时不做解析，直接返回固定的「比赛已开始 + 默认血量」，
   * 用于没有裁判系统时联调导航决策（只影响本函数，不影响 is_game_started()）。
   */
  RefereeState referee_state() const;

  /* 比赛是否已开始；未启用裁判系统时恒为 true */
  bool is_game_started() const;

  /* 配置里是否启用了裁判系统（use_referee_system），供启动日志区分两种「常开」 */
  bool referee_enabled() const { return use_referee_system_; }

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

  /*
   * 0x0402 报文里没有 control 位（见 gimbal/packet.hpp 的 SendPacket），
   * 下位机对 pitch / yaw 无条件执行。control=false 时必须沿用上一次的有效目标角，
   * 否则会把 io::Command 默认构造的 0 当作目标发下去，云台会突然归零。
   */
  mutable std::mutex last_command_mutex_;
  mutable double last_yaw_ = 0.0;
  mutable double last_pitch_ = 0.0;
  mutable bool has_last_command_ = false;

  /* 关节反馈订阅方（目前是 io::ROS2 的 JointState 发布节点） */
  mutable std::mutex joint_callback_mutex_;
  GimbalJointCallback joint_callback_;

  tools::ThreadSafeQueue<IMUData> queue_{1000};
  IMUData data_ahead_;
  IMUData data_behind_;

  /*
   * test_nav 测试开关：为 true 时 referee_state() 直接返回 test_nav_state_，
   * 完全忽略下位机上报（没接裁判系统也能让导航决策节点跑起来）。
   */
  bool test_nav_ = false;
  RefereeState test_nav_state_{};

  /* 下位机上报的比赛与机器人状态，接收线程写入，getter 读取 */
  GameStatus game_status_{};
  RobotStatus robot_status_{};
  /* 0x0502 的 roboid（唯一 ID 来源），0 = 还没收到 */
  std::atomic<uint8_t> robot_id_{0};
  /* 诊断用：0x0502 帧计数与最近一次原始 roboid（含 0） */
  std::atomic<uint64_t> imu_frame_count_{0};
  std::atomic<uint8_t> last_raw_roboid_{0};
  std::atomic<uint64_t> game_status_frame_count_{0};
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

  /* 0x505 基础状态：只取复位位（roboid 与 0x0502 同源，不参与阵营判定） */
  void handleBass(const bassPacket & pkt);

  /*
   * 阵营 ID 更新，**只由 0x0502 的 roboid 调用**。
   * 严格按附录二编号表校验：0 表示固件没填（保持上一次有效值），
   * 表外值直接忽略并告警，避免把非法编码当成敌方阵营。
   */
  void update_robot_id(uint8_t id, const char * source);
};

/*
 * 附录二 兵种 ID 编号表（红方 1~11，蓝方 101~111，蓝方 = 红方 + 100）：
 *
 *     1 英雄    2 工程    3/4/5 步兵    6 空中    7 哨兵
 *     8 飞镖    9 雷达    10 基地       11 前哨站
 *
 * is_red_robot_id / is_blue_robot_id 只判阵营，返回 false 表示 ID 不在表内
 * （含 0 = 还没收到）；具体兵种用 robot_role() 查。
 */
inline bool is_red_robot_id(uint8_t id) { return id >= 1 && id <= 11; }
inline bool is_blue_robot_id(uint8_t id) { return id >= 101 && id <= 111; }

/* 是否是编号表里的合法 ID（红 1~11 / 蓝 101~111）；0 与其它值都不算 */
inline bool is_valid_robot_id(uint8_t id) { return is_red_robot_id(id) || is_blue_robot_id(id); }

enum class RobotRole
{
  hero,      // 1 / 101
  engineer,  // 2 / 102
  infantry,  // 3~5 / 103~105
  aerial,    // 6 / 106
  sentry,    // 7 / 107
  dart,      // 8 / 108
  radar,     // 9 / 109
  base,      // 10 / 110
  outpost,   // 11 / 111
  unknown
};

/* 去掉队伍位（蓝方 -100）后查表；非法 ID 返回 RobotRole::unknown */
inline RobotRole robot_role(uint8_t id)
{
  const int base = is_blue_robot_id(id) ? static_cast<int>(id) - 100 : static_cast<int>(id);
  switch (base) {
    case 1:
      return RobotRole::hero;
    case 2:
      return RobotRole::engineer;
    case 3:
    case 4:
    case 5:
      return RobotRole::infantry;
    case 6:
      return RobotRole::aerial;
    case 7:
      return RobotRole::sentry;
    case 8:
      return RobotRole::dart;
    case 9:
      return RobotRole::radar;
    case 10:
      return RobotRole::base;
    case 11:
      return RobotRole::outpost;
    default:
      return RobotRole::unknown;
  }
}

/* 日志用：兵种名；非法 ID 返回 "unknown" */
inline const char * robot_role_name(uint8_t id)
{
  switch (robot_role(id)) {
    case RobotRole::hero:
      return "hero";
    case RobotRole::engineer:
      return "engineer";
    case RobotRole::infantry:
      return "infantry";
    case RobotRole::aerial:
      return "aerial";
    case RobotRole::sentry:
      return "sentry";
    case RobotRole::dart:
      return "dart";
    case RobotRole::radar:
      return "radar";
    case RobotRole::base:
      return "base";
    case RobotRole::outpost:
      return "outpost";
    case RobotRole::unknown:
      break;
  }
  return "unknown";
}

/*
 * 打印一次比赛信息快照，供各个入口（sentry / debug / 仿真）统一复用。
 *
 * 推荐调用时机：
 *   · 程序启动（reason="启动"）—— 裁判帧通常还没到，主要确认裁判系统是否启用、
 *     robot_id 有没有上报、弹速是多少
 *   · 裁判帧首次到齐（reason="裁判就绪"）—— 补打真实血量与剩余时间
 *   · is_game_started() 翻转（reason="比赛开始" / "比赛结束"）
 *
 * 数据未就绪时打印「未知」，不会用默认 0 冒充真实值。
 */
void log_match_info(const SerialBoard & board, const std::string & reason);

}  // namespace io

#endif  // IO__SERIAL_BOARD_HPP
