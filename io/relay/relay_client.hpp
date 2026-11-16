#ifndef IO__RELAY__RELAY_CLIENT_HPP
#define IO__RELAY__RELAY_CLIENT_HPP

/*
 * 视觉侧的中转客户端：订阅 info_relay 发布的上行数据，发布下行的云台指令。
 *
 * 数据流：info_relay（独占串口）→ iceoryx2 主题 → 本类接收线程 → 快照 + IMU 样本队列
 *         → 主循环按需调用 imu_state_at() / referee_state() 等；
 *         主循环 send() → iceoryx2 cmd_aim 主题 → info_relay → 串口。
 *
 * 线程：本类内部起一条接收线程，订阅侧数据写进 mutex 保护的快照；
 *       send() 可在主循环线程直接调用（非阻塞）。
 *
 * 注意：1) 视觉进程不再打开串口，也不必知道串口号与波特率，只需要 relay 的主题前缀；
 *       2) relay 未启动或链路中断时，link_ok() 为 false，can_shoot() 恒为 false，
 *          imu_state_at() 返回最后一次收到的姿态（从未收到时返回零姿态），不阻塞主循环；
 *       3) 时间戳一律使用 relay 打的 t_ns（同机单调时钟），不得在本侧重新打点。
 */

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "io/command.hpp"
#include "io/mode.hpp"

namespace io
{

class RelayClient
{
public:
  double bullet_speed;  // 弹速，m/s；来自配置，可被下位机数据覆盖（当前仅取配置值）

  // 功能：连接 iceoryx2 主题并启动接收线程
  // 输入：
  //   config_path  YAML 配置路径；读取 relay_prefix 与 relay_timeout_ms
  // 输出：无
  // 备注：relay 未启动也能正常构造，只是 link_ok() 为 false；不抛异常
  explicit RelayClient(const std::string & config_path);

  // 功能：停止接收线程并释放 iceoryx2 资源
  // 输入：无
  // 输出：无
  // 备注：会 join 接收线程，通常几毫秒内返回
  ~RelayClient();

  // 功能：取指定时刻的云台姿态（在已收到的样本间线性插值）
  // 输入：
  //   t  目标时刻，steady_clock 时间点
  // 输出：ImuState，角度单位 rad；没有可用样本时返回零姿态
  // 备注：只读快照，不阻塞；样本时间戳来自 relay
  ImuState imu_state_at(std::chrono::steady_clock::time_point t);

  // 功能：把姿态合成成云台到世界的旋转（Rz(lower)·Ry(pitch)·Rx(roll)）
  // 输入：
  //   t  目标时刻
  // 输出：单位四元数或对应姿态
  // 备注：与原先直接读串口时的约定一致，单 yaw 构型下 upper==lower
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point t);

  // 功能：下发云台瞄准指令
  // 输入：
  //   command  指令，yaw/pitch 单位 rad（世界系绝对角）
  // 输出：无
  // 备注：非阻塞；链路失效时仍然发送，但 shoot 位会被强制清零
  void send(Command command) const;

  // 功能：下发导航速度指令
  // 输入：
  //   command  速度指令，vx/vy 单位 m/s、wz 单位 rad/s
  // 输出：无
  // 备注：非阻塞；正常情况下导航链路在 relay 内部闭环，本接口仅供调试
  void send(NavCommand command) const;

  // 返回值：relay 链路是否健康（timeout 内有 IMU 样本）
  bool link_ok() const;

  // 返回值：是否允许开火（链路健康 + 热量未超限）
  bool can_shoot() const;

  // 返回值：比赛是否已开始（裁判 0x0001 的 game_progress 达到阈值）
  bool is_game_started() const;

  // 返回值：己方是否红方；ID 非法或未收到时为 nullopt
  std::optional<bool> own_team_red() const;

  // 返回值：本机机器人 ID；0 表示未收到
  uint8_t robot_id() const;

  // 返回值：裁判状态快照（血量/比赛进度与有效位）
  RefereeState referee_state() const;

  // 返回值：17mm 当前热量（热量单位）
  uint16_t barrel_heat() const;

  // 返回值：热量上限（热量单位）；0 表示未知
  uint16_t heat_limit() const;

  // 功能：取走 MCU 重启事件
  // 输入：无
  // 输出：true 表示自上次调用以来检测到 MCU 重启
  // 备注：靠 Imu 里的 uptime_ms 回落判定，与 relay 侧的判定口径一致
  bool reset_pending();

private:
  // 一帧带时间戳的姿态样本，用于插值
  struct ImuSample
  {
    int64_t t_ns = 0;  // relay 采集时刻，ns
    ImuState state;    // 该时刻的姿态，rad
  };

  // 功能：接收线程主体，轮询各订阅并更新快照
  // 输入：无
  // 输出：无
  // 备注：退出由 quit_ 控制；每轮先排空 IMU，再取状态类主题的最新值
  void receive_loop();

  // 功能：按时间戳插值出姿态
  // 输入：
  //   t_ns  目标时刻，ns
  // 输出：插值后的姿态；样本不足时返回最近一次
  // 备注：调用方必须已持有 mutex_
  ImuState interpolate_locked(int64_t t_ns) const;

  std::string prefix_;  // 主题前缀，来自配置 relay_prefix
  // send() 声明为 const（与原先串口接口一致），因此待发槽必须可变
  mutable std::optional<Command> pending_cmd_;     // 主循环写入、接收线程取走的瞄准指令
  mutable std::optional<NavCommand> pending_nav_;  // 主循环写入、接收线程取走的导航指令

  mutable std::mutex mutex_;              // 保护下面所有快照字段
  std::deque<ImuSample> imu_samples_;     // 最近的姿态样本，容量固定，满了丢最旧
  int64_t last_imu_ns_ = 0;               // 最近一次收到 IMU 的时刻，ns
  uint32_t last_uptime_ms_ = 0;           // 最近一次 MCU 上电时间，ms
  bool reset_pending_ = false;            // MCU 重启事件标志，读取后清零

  RefereeState referee_;                  // 裁判状态快照
  uint8_t robot_id_ = 0;                  // 本机 ID
  std::optional<bool> own_red_;           // 己方是否红方
  uint16_t barrel_heat_ = 0;              // 当前 17mm 热量
  uint16_t heat_limit_ = 0;               // 热量上限
  bool game_started_ = false;             // 比赛是否已开始

  int timeout_ms_ = 200;                  // 链路失效判定阈值，ms；来自配置
  std::atomic<bool> schema_ok_{true};     // 契约版本是否与中转进程一致，不一致则链路判失效
  std::thread thread_;                    // 接收线程
  std::atomic<bool> quit_{false};         // 接收线程退出标志
};

}  // namespace io

#endif  // IO__RELAY__RELAY_CLIENT_HPP
