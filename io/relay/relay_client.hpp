#ifndef IO__RELAY__RELAY_CLIENT_HPP
#define IO__RELAY__RELAY_CLIENT_HPP

/*
 * 视觉侧唯一的下位机数据入口：订阅中转进程发布的上行数据，发布云台瞄准指令。
 *
 * 数据流：
 *   中转进程 → 共享内存 imu / robot / game / health → 本类接收线程 → 快照 + 姿态样本队列
 *   → 主循环按需取用（imu_state_at / referee_state / can_shoot …）；
 *   主循环 send() → 待发槽 → 接收线程下一拍发布 cmd_aim → 中转进程写串口/CAN。
 *
 * 线程：
 *   本类自建一条接收线程，负责全部订阅取样与指令发布；
 *   对外的读取接口加锁取快照，send() 只写待发槽，不阻塞主循环。
 *
 * 依赖与边界：
 *   只依赖 io/relay 下的线格式副本与 iceoryx2，不引用 info_relay 仓库的任何源码或产物；
 *   不打开任何设备、不做协议解析，这些都属于中转进程。
 *
 * 使用注意：
 *   1) 本类不订阅 chassis、不发布 cmd_nav —— 本项目不参与导航；
 *   2) 链路失效（无 IMU 超时 / transport_ok=0 / 契约版本不符）时 can_shoot() 恒为 false，
 *      且下发的 shoot 位被强制清零：云台继续跟踪，但绝不发弹；
 *   3) 时间戳一律用中转给的 t_ns（同机单调时钟），本侧不得重新打点。
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
#include "io/relay/relay_contract.hpp"

namespace io
{

class RelayClient
{
public:
  double bullet_speed;  // 弹速，m/s；来自配置，仅用于弹道解算

  // 功能：连接 iceoryx2 主题并启动接收线程
  // 输入：
  //   config_path  YAML 配置路径；读取 relay_prefix 与 relay_timeout_ms
  // 输出：无
  // 备注：中转未启动也能正常构造，只是 link_ok() 为 false；不抛异常
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
  // 备注：只读快照，不阻塞；样本时间戳来自中转
  ImuState imu_state_at(std::chrono::steady_clock::time_point t);

  // 功能：把姿态合成成云台到世界的旋转（Rz(lower)·Ry(pitch)·Rx(roll)）
  // 输入：
  //   t  目标时刻
  // 输出：单位四元数或对应姿态
  // 备注：与原先直连串口时的约定一致；单 yaw 构型下 upper == lower
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point t);

  // 功能：下发云台瞄准指令
  // 输入：
  //   command  指令，yaw/pitch 单位 rad（世界系绝对角）
  // 输出：无
  // 备注：非阻塞，只保留最新一条；链路失效时 shoot 位被强制清零
  void send(Command command) const;

  // 返回值：链路是否健康（有 IMU 样本、未超时、契约版本一致）
  bool link_ok() const;

  // 返回值：是否允许开火（链路健康且热量未到上限）
  bool can_shoot() const;

  // 返回值：比赛是否已开始（Game.in_battle 为 1）
  bool is_game_started() const;

  // 返回值：己方是否红方；ID 非法或未收到时为 nullopt
  std::optional<bool> own_team_red() const;

  // 返回值：本机机器人 ID；0 表示未收到
  uint8_t robot_id() const;

  // 返回值：裁判状态快照（血量 / 比赛进度 / 有效位）
  RefereeState referee_state() const;

  // 返回值：17mm 当前热量（热量单位）
  uint16_t barrel_heat() const;

  // 返回值：热量上限（热量单位）；0 表示未知
  uint16_t heat_limit() const;

  // 功能：取走 MCU 重启事件
  // 输入：无
  // 输出：true 表示自上次调用以来检测到 uptime 回落
  // 备注：靠 Imu.uptime_ms 判定，与中转侧口径一致
  bool reset_pending();

  // 返回值：与中转的契约版本是否一致
  // 用途：部署自检时可直接读取，不一致说明两侧头文件不同步
  bool schema_ok() const { return schema_ok_; }

private:
  // 一帧带时间戳的姿态样本，用于按图像时间戳插值
  struct ImuSample
  {
    int64_t t_ns = 0;  // 中转采集时刻，ns
    ImuState state;    // 该时刻的姿态，rad
  };

  // 功能：接收线程主体，轮询各订阅并维护快照
  // 输入：无
  // 输出：无
  // 备注：由 quit_ 控制退出；每轮先排空 IMU 队列，再取状态类主题的最新值
  void receive_loop();

  // 功能：按时间戳插值出姿态
  // 输入：
  //   t_ns  目标时刻，ns
  // 输出：插值后的姿态；样本不足时返回最近一次
  // 备注：调用方必须已持有 mutex_
  ImuState interpolate_locked(int64_t t_ns) const;

  std::string prefix_;                   // 主题前缀，来自配置 relay_prefix
  mutable std::optional<Command> pending_cmd_;  // 主循环写入、接收线程取走的指令

  mutable std::mutex mutex_;             // 保护下面所有快照字段
  std::deque<ImuSample> imu_samples_;    // 最近姿态样本，容量固定，满了丢最旧
  int64_t last_imu_ns_ = 0;              // 最近一次收到 IMU 的时刻，ns
  uint32_t last_uptime_ms_ = 0;          // 最近一次 MCU 上电时间，ms
  bool reset_pending_ = false;           // MCU 重启事件标志，读取后清零

  RefereeState referee_;                 // 裁判状态快照
  uint8_t robot_id_ = 0;                 // 本机 ID
  std::optional<bool> own_red_;          // 己方是否红方
  uint16_t barrel_heat_ = 0;             // 当前 17mm 热量
  uint16_t heat_limit_ = 0;              // 热量上限
  bool game_started_ = false;            // 比赛是否已开始

  int timeout_ms_ = 200;                 // 链路失效判定阈值，ms；来自配置
  std::atomic<bool> schema_ok_{true};    // 契约版本是否与中转一致
  std::thread thread_;                   // 接收线程
  std::atomic<bool> quit_{false};        // 接收线程退出标志
};

}  // namespace io

#endif  // IO__RELAY__RELAY_CLIENT_HPP
