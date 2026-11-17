#ifndef SRC__SENTRY_FSM_HPP
#define SRC__SENTRY_FSM_HPP

/*
 * 哨兵瞄准模式的状态机：把原先散在主循环里的 if-else 分支收成 4 个显式状态。
 *
 * 数据流：
 *   主循环每帧组一个 Context（相机帧、检测、姿态、各算法组件）→ Fsm::tick()
 *   → 按当前状态调用对应的 on_xxx() → 该函数算出本帧指令，并在末尾调用 check()
 *   → check() 用「当前状态 × 事件」查表得到下一状态，校验合法后落定。
 *
 * 依赖与边界：
 *   只决定"这一帧该用哪条瞄准路径、输出什么指令"，不碰串口、不碰录像、不碰绘图；
 *   下发与录像由主循环末尾统一完成（见 sentry_multithread.cpp 的收尾点）。
 *
 * 线程：
 *   仅在主循环线程使用，无内部锁；Context 里的组件由调用方保证线程安全。
 *
 * 使用注意：
 *   1) 状态转移发生在**帧末**，因此切换瞬间会晚一帧生效；各 on_xxx 内部对
 *      "tracker 状态与当前状态不一致"做了短路，避免用错路径算出一帧坏角。
 *   2) 非法转移（表里标为 kCount）不抛异常，只计数 + 限流报错并保持原状态，
 *      主循环不能因为一次状态异常停摆。
 */

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <functional>
#include <list>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "io/command.hpp"
#include "io/relay/relay_client.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/multithread/yolo_pipeline.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/omni_jobs.hpp"

namespace sentry_fsm
{

// 瞄准模式：任一时刻恰好处于其中一个
enum class State : uint8_t
{
  kIdleNoOmni = 0,  // 无全向相机 + 主相机无检测 + tracker 丢失：自旋或原地保持
  kSearch,          // 主相机未锁定，用全向候选搜索
  kSwitching,       // tracker 已决定换目标，等待新目标观测
  kTracking,        // 主相机锁定，由 aimer 出角
  kCount,           // 哨兵值：既表示状态数量，也用作"非法转移"的占位
};

// 触发转移的事件：由状态函数在帧末根据 tracker 状态与实际观测给出
enum class Event : uint8_t
{
  kStay = 0,  // 本帧无变化
  kIdle,      // 满足"无全向且主相机持续无检测"
  kLost,      // tracker 丢失
  kSwitch,    // tracker 进入 switching
  kTrack,     // tracker 进入跟踪（tracking / detecting / temp_lost）
  kCount,
};

// 功能：取状态的稳定名字
// 输入：s 状态
// 输出：英文短名，用于日志与 HUD
// 备注：与枚举顺序一致，新增状态时同步补充
const char * to_string(State s);

// 功能：取事件名字
// 输入：e 事件
// 输出：英文短名，用于转移日志
// 备注：同上
const char * to_string(Event e);

// 一帧的输入输出集合：引用外部对象，不持有所有权
struct Context
{
  // ---- 本帧输入 ----
  const cv::Mat & img;                                       // 主相机帧，只读
  std::chrono::steady_clock::time_point timestamp;           // 采集时刻
  std::list<auto_aim::Armor> & armors;                       // 主相机检测结果，会被过滤
  Eigen::Vector3d gimbal_pos;                                // 世界系绝对 yaw/pitch/roll，rad
  double omni_base_yaw = 0;                                  // 大yaw 世界系 yaw，rad
  bool has_omni = false;                                     // 是否启用全向相机
  bool spin_when_no_omni = true;                             // 无全向且丢失时是否自旋

  // ---- 算法组件 ----
  auto_aim::multithread::YoloPipeline & pipeline;            // 取全向推理结果
  auto_aim::Tracker & tracker;                               // 目标跟踪与状态
  omniperception::Decider & decider;                         // 全向候选排序与角度解算
  auto_aim::Aimer & aimer;                                   // 主相机弹道解算
  auto_aim::Shooter & shooter;                               // 开火判据
  io::RelayClient & board;                                   // 只为取弹速与热量门限

  // ---- 跨帧缓存：全向目标丢失后回放 ----
  bool & has_omni_cache;
  double & cached_omni_yaw;
  double & cached_omni_pitch;

  // 刷新敌我颜色（依赖裁判 robot_id），由入口注入，避免状态机直接依赖串口
  std::function<void()> & refresh_enemy_color;

  // ---- 本帧中间结果 ----
  std::vector<omniperception::DetectionResult> detection_queue;  // 全向候选，已排序
  const auto_aim::Target * target = nullptr;                     // 主相机跟踪目标，可为空
  omniperception::DetectionResult switch_target{};               // 切换中的新目标

  // ---- 本帧输出 ----
  io::Command cmd{false, 0, 0};  // 待下发指令，未产出瞄准角时由调用方回填姿态
  bool has_aim = false;          // 是否产出了可用瞄准角
};

class Fsm
{
public:
  // 功能：推进一帧
  // 输入：
  //   ctx  本帧上下文，调用方负责填充输入区；输出区由本函数写入
  // 输出：true 表示本帧走了主链路（已调用 tracker.track），调用方据此决定是否
  //       执行开火判据与绘图；false 表示处于 kIdleNoOmni
  // 备注：无论走哪条路，返回前一定会调用一次 check() 落定下一状态
  bool tick(Context & ctx);

  // 返回值：当前状态
  State state() const { return state_; }

  // 返回值：当前状态名，供 HUD 与日志使用
  const char * state_name() const { return to_string(state_); }

  // 返回值：累计非法转移次数，部署自检时可读
  uint64_t illegal_transitions() const { return illegal_; }

private:
  // 功能：kIdleNoOmni 状态的动作
  // 输入：ctx 本帧上下文
  // 输出：写 ctx.cmd / ctx.has_aim
  // 备注：自旋时主动给目标角，保持时不产出瞄准角；末尾 check(kIdle)
  void on_idle(Context & ctx);

  // 功能：kSearch 状态的动作
  // 输入：ctx 本帧上下文
  // 输出：写 ctx.cmd / ctx.has_aim / 全向缓存
  // 备注：tracker 已不在 lost 时短路，把动作留给下一帧的新状态
  void on_search(Context & ctx);

  // 功能：kSwitching 状态的动作
  // 输入：ctx 本帧上下文
  // 输出：写 ctx.cmd / ctx.has_aim
  // 备注：新目标尚未观测到时不给角度，避免下发默认 0 把云台拽回原点
  void on_switching(Context & ctx);

  // 功能：kTracking 状态的动作
  // 输入：ctx 本帧上下文
  // 输出：写 ctx.cmd / ctx.has_aim
  // 备注：aimer 返回空值表示弹道无解，此时不产出瞄准角
  void on_tracking(Context & ctx);

  // 功能：做本帧的公共准备（刷新颜色、过滤、取全向结果、排序、跟踪一次）
  // 输入：ctx 本帧上下文
  // 输出：填充 ctx.detection_queue / ctx.target / ctx.switch_target
  // 备注：tracker.track() 每帧只允许调用一次，因此集中在这里
  void prepare(Context & ctx);

  // 功能：统一的转移出口，每个状态函数末尾调用
  // 输入：ev 本帧观察到的事件
  // 输出：无；内部更新 state_
  // 备注：表的格子为 kCount 时视为非法转移，计数并保持原状态，不抛异常
  void check(Event ev);

  // 功能：由 tracker 状态推导事件
  // 输入：ctx 本帧上下文
  // 输出：事件
  // 备注：只读 tracker，不产生副作用
  static Event observe_event(const Context & ctx);

  State state_ = State::kIdleNoOmni;  // 当前状态
  uint64_t illegal_ = 0;              // 非法转移累计次数，用于限流报错与自检
};

}  // namespace sentry_fsm

#endif  // SRC__SENTRY_FSM_HPP
