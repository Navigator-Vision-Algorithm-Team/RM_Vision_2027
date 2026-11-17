/*
 * 哨兵瞄准模式状态机的实现：状态动作 + 转移表 + 统一 check。
 *
 * 数据流：tick() → （空闲则直接走 on_idle）→ prepare() 更新 tracker → 按 state_ 分派
 *         → 状态函数算指令 → 帧末 check(event) 查表落定下一状态。
 *
 * 注意：契约见 sentry_fsm.hpp，本文件只写实现要点。
 */
#include "sentry_fsm.hpp"

#include <algorithm>
#include <utility>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace sentry_fsm
{
namespace
{
// 状态数量与事件数量：用于把枚举下标算进一维转移表
constexpr size_t kStateCount = static_cast<size_t>(State::kCount);
constexpr size_t kEventCount = static_cast<size_t>(Event::kCount);

// 转移表：[当前状态][事件] -> 下一状态；kCount 表示该转移非法
// 行序：kIdleNoOmni, kSearch, kSwitching, kTracking
// 列序：kStay, kIdle, kLost, kSwitch, kTrack
constexpr State kTable[kStateCount][kEventCount] = {
  // kIdleNoOmni：空闲只在"仍满足空闲条件"时停留。
  // 若在空闲态观察到 switching/tracking，说明 tracker 与空闲条件矛盾（逻辑错误），
  // 这两格标为非法，由 check() 报错并保持原状态，而不是悄悄跳到一个不该去的状态。
  {State::kIdleNoOmni, State::kIdleNoOmni, State::kIdleNoOmni, State::kCount, State::kCount},
  // kSearch：丢失时继续搜索；看到跟踪态就上交主相机
  {State::kSearch, State::kIdleNoOmni, State::kSearch, State::kSwitching, State::kTracking},
  // kSwitching：换目标期间仍按 switching 处理，直到 tracker 给出明确的新状态
  {State::kSwitching, State::kIdleNoOmni, State::kSearch, State::kSwitching, State::kTracking},
  // kTracking：一旦丢失就退回搜索（有全向）或空闲
  {State::kTracking, State::kIdleNoOmni, State::kSearch, State::kSwitching, State::kTracking},
};

// 功能：把枚举转成一维表下标
// 输入：s 状态 / e 事件
// 输出：下标
// 备注：越界（kCount）按最后一行/列处理不会发生——调用方只传合法枚举
constexpr size_t idx(State s) { return static_cast<size_t>(s); }
constexpr size_t idx(Event e) { return static_cast<size_t>(e); }
}  // namespace

const char * to_string(State s)
{
  switch (s) {
    case State::kIdleNoOmni:
      return "idle";
    case State::kSearch:
      return "search";
    case State::kSwitching:
      return "switching";
    case State::kTracking:
      return "tracking";
    case State::kCount:
      return "count";
  }
  return "unknown";
}

const char * to_string(Event e)
{
  switch (e) {
    case Event::kStay:
      return "stay";
    case Event::kIdle:
      return "idle";
    case Event::kLost:
      return "lost";
    case Event::kSwitch:
      return "switch";
    case Event::kTrack:
      return "track";
    case Event::kCount:
      return "count";
  }
  return "unknown";
}

Event Fsm::observe_event(const Context & ctx)
{
  const std::string s = ctx.tracker.state();
  if (s == "switching") return Event::kSwitch;
  if (s == "lost") return Event::kLost;
  return Event::kTrack;
}

void Fsm::check(Event ev)
{
  const State next = kTable[idx(state_)][idx(ev)];
  if (next == State::kCount) {
    // 主循环不能因为一次状态异常停摆：计数 + 限流报错 + 保持原状态
    ++illegal_;
    if (illegal_ == 1 || illegal_ % 100 == 0) {
      tools::logger()->error(
        "[sentry][fsm] 非法转移 {} --{}--> 忽略，保持原状态（累计 {} 次）", to_string(state_),
        to_string(ev), illegal_);
    }
    return;
  }

  if (next != state_) {
    tools::logger()->debug("[sentry][fsm] {} -> {}", to_string(state_), to_string(next));
    state_ = next;
  }
}

void Fsm::prepare(Context & ctx)
{
  // 敌我颜色以裁判 robot_id 为准，必须在过滤之前刷新
  if (ctx.refresh_enemy_color) ctx.refresh_enemy_color();

  ctx.decider.armor_filter(ctx.armors);
  ctx.detection_queue = omniperception::take_detection_results(ctx.pipeline);
  ctx.decider.sort(ctx.detection_queue);

  // tracker.track() 每帧只允许调用一次，返回的指针只在本帧内有效
  auto [switch_target, target] =
    ctx.tracker.track(ctx.detection_queue, ctx.armors, ctx.timestamp, true, ctx.img.size());
  ctx.switch_target = std::move(switch_target);
  ctx.target = target;
}

void Fsm::on_idle(Context & ctx)
{
  // 无全向且主相机丢失：本帧不产出瞄准角，统一交给 finalize() 调用自旋决策。
  // 不再有"自旋开关"——自旋是无条件兜底行为。
  ctx.has_aim = false;
  ctx.cmd.shoot = false;
  check(Event::kIdle);
}

void Fsm::on_search(Context & ctx)
{
  // tracker 已不在 lost：本帧不动作，交给下一帧的新状态，避免用过期的搜索路径算角
  if (ctx.tracker.state() != "lost") {
    check(observe_event(ctx));
    return;
  }

  auto decided = ctx.decider.decide(ctx.detection_queue);
  if (decided) {
    ctx.cmd = *decided;
    ctx.has_aim = true;
    // decide() 给的是大yaw 系角，转成世界系绝对角后才能下发
    ctx.cmd.yaw = tools::limit_rad(ctx.cmd.yaw + ctx.omni_base_yaw);
    ctx.cached_omni_yaw = ctx.cmd.yaw;
    ctx.cached_omni_pitch = ctx.cmd.pitch;
    ctx.has_omni_cache = true;
  } else if (ctx.has_omni_cache) {
    // 全向暂时丢失：回放缓存，保持云台继续转向目标
    ctx.has_aim = true;
    ctx.cmd.shoot = false;
    ctx.cmd.yaw = ctx.cached_omni_yaw;
    ctx.cmd.pitch = ctx.cached_omni_pitch;
  } else {
    // 既无新结果也无缓存：不产出瞄准角，由 finalize() 统一执行自旋点头
    ctx.has_aim = false;
  }

  check(observe_event(ctx));
}

void Fsm::on_switching(Context & ctx)
{
  if (ctx.tracker.state() != "switching") {
    check(observe_event(ctx));
    return;
  }

  // 换目标期间不允许开火
  ctx.cmd.shoot = false;
  // 新目标尚未观测到时 delta 为默认 0，直接下发会让云台归零，因此不产出角度
  ctx.has_aim = !ctx.switch_target.armors.empty();
  if (ctx.has_aim) {
    ctx.cmd.pitch = tools::limit_rad(ctx.switch_target.delta_pitch);
    ctx.cmd.yaw = tools::limit_rad(ctx.switch_target.delta_yaw + ctx.omni_base_yaw);
  }

  check(observe_event(ctx));
}

void Fsm::on_tracking(Context & ctx)
{
  // tracker 已丢失或正在切换：本帧不动作，交给下一帧的新状态
  const std::string s = ctx.tracker.state();
  if (s == "lost" || s == "switching") {
    check(observe_event(ctx));
    return;
  }

  // aimer 输出已是世界系绝对角，不得再叠加 gimbal_pos[0]
  auto aimed = ctx.aimer.aim(ctx.target, ctx.timestamp, ctx.board.bullet_speed);
  if (aimed) {
    ctx.cmd = *aimed;
    ctx.has_aim = true;
  } else {
    // EKF 未就绪或弹道无解：不产出瞄准角
    ctx.has_aim = false;
  }
  ctx.has_omni_cache = false;

  check(observe_event(ctx));
}

void Fsm::finalize(Context & ctx)
{
  // 唯一出口：所有状态函数只负责设置 has_aim，最终姿态在这里一次结算。
  if (ctx.has_aim) {
    // 命中有效目标：复位自旋相位，下次丢失时从头开始点头
    spin_.reset();
    return;
  }

  // 无有效瞄准角：无条件自旋（不区分有无全向相机），并禁止开火
  const auto spin = spin_.update(ctx.timestamp, ctx.gimbal_pos);
  ctx.cmd.yaw = spin.yaw;
  ctx.cmd.pitch = spin.pitch;
  ctx.cmd.shoot = false;
}

bool Fsm::tick(Context & ctx)
{
  ctx.cmd = io::Command{false, 0, 0};
  ctx.has_aim = false;

  // 空闲条件：没有全向相机、主相机无检测、tracker 丢失
  const bool idle = !ctx.has_omni && ctx.armors.empty() && ctx.tracker.state() == "lost";
  if (idle) {
    on_idle(ctx);
    finalize(ctx);
    return false;
  }

  // 离开空闲：按是否启用全向决定先进入搜索还是跟踪。
  // 这一步是"观察驱动的纠正"，与帧末的 check() 互补：
  // 空闲态不跑 tracker，因此只能靠本帧观察决定出去后的起点。
  if (state_ == State::kIdleNoOmni) {
    state_ = ctx.has_omni ? State::kSearch : State::kTracking;
    tools::logger()->debug("[sentry][fsm] idle -> {}", to_string(state_));
  }

  prepare(ctx);

  switch (state_) {
    case State::kSearch:
      on_search(ctx);
      break;
    case State::kSwitching:
      on_switching(ctx);
      break;
    case State::kTracking:
      on_tracking(ctx);
      break;
    case State::kIdleNoOmni:
    case State::kCount:
      // 理论上不可达：idle 已在上面提前返回，kCount 不是合法状态
      on_idle(ctx);
      break;
  }

  finalize(ctx);
  return true;
}

}  // namespace sentry_fsm
