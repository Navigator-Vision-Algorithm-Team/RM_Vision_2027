#include "color_gate.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "tools/logger.hpp"

namespace omniperception
{
namespace
{

using Clock = std::chrono::steady_clock;

/*
 * 读取配置：先查 <key>_omniN，再查 <key>，都没有就用内置默认值。
 * 解析失败（类型写错）时静默回退到默认值，不让一个手误的参数卡住启动。
 */
template <typename T>
T pick(const YAML::Node & yaml, const std::string & key, int camera_index, const T & fallback)
{
  const std::string suffix = key + "_omni" + std::to_string(camera_index + 1);
  if (yaml[suffix]) {
    try {
      return yaml[suffix].as<T>();
    } catch (const std::exception &) {
    }
  }
  if (yaml[key]) {
    try {
      return yaml[key].as<T>();
    } catch (const std::exception &) {
    }
  }
  return fallback;
}

}  // namespace

const char * gate_state_name(GateState state)
{
  switch (state) {
    case GateState::Idle:
      return "idle";
    case GateState::Active:
      return "active";
    case GateState::Cooldown:
      return "cooldown";
  }
  return "unknown";
}

ColorGate::ColorGate(
  const std::string & config_path, int camera_index, std::atomic<auto_aim::Color> * enemy_color,
  std::atomic<bool> * main_tracking, std::atomic<std::uint8_t> * robot_id)
: camera_index_(camera_index),
  enemy_color_(enemy_color),
  main_tracking_(main_tracking),
  robot_id_(robot_id)
{
  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(config_path);
  } catch (const std::exception & e) {
    tools::logger()->warn(
      "[ColorGate] cam{} 读取配置失败（{}），全部使用默认参数", camera_index_, e.what());
  }

  cfg_.enable = pick(yaml, "omni_gate_enable", camera_index_, cfg_.enable);
  cfg_.adaptive = pick(yaml, "omni_gate_adaptive", camera_index_, cfg_.adaptive);
  cfg_.debug = pick(yaml, "omni_gate_debug", camera_index_, cfg_.debug);
  cfg_.suspend_on_main_tracking = pick(
    yaml, "omni_gate_suspend_on_main_tracking", camera_index_, cfg_.suspend_on_main_tracking);

  cfg_.divisor = std::max(1, pick(yaml, "omni_gate_divisor", camera_index_, cfg_.divisor));
  cfg_.scale = std::clamp(pick(yaml, "omni_gate_scale", camera_index_, cfg_.scale), 0.1, 1.0);

  const std::vector<int> default_red_h = {0, 8, 170, 179};
  const auto red_h = pick(yaml, "omni_gate_red_h", camera_index_, default_red_h);
  if (red_h.size() >= 4) {
    cfg_.red_h1_lo = std::clamp(red_h[0], 0, 179);
    cfg_.red_h1_hi = std::clamp(red_h[1], 0, 179);
    cfg_.red_h2_lo = std::clamp(red_h[2], 0, 179);
    cfg_.red_h2_hi = std::clamp(red_h[3], 0, 179);
  }

  const std::vector<int> default_blue_h = {98, 132};
  const auto blue_h = pick(yaml, "omni_gate_blue_h", camera_index_, default_blue_h);
  if (blue_h.size() >= 2) {
    cfg_.blue_h_lo = std::clamp(blue_h[0], 0, 179);
    cfg_.blue_h_hi = std::clamp(blue_h[1], 0, 179);
  }

  cfg_.s_min = pick(yaml, "omni_gate_s_min", camera_index_, cfg_.s_min);
  cfg_.v_min = pick(yaml, "omni_gate_v_min", camera_index_, cfg_.v_min);
  cfg_.v_floor = pick(yaml, "omni_gate_v_floor", camera_index_, cfg_.v_floor);
  cfg_.v_ceil = pick(yaml, "omni_gate_v_ceil", camera_index_, cfg_.v_ceil);

  cfg_.min_area_ratio = pick(yaml, "omni_gate_min_area_ratio", camera_index_, cfg_.min_area_ratio);
  cfg_.max_area_ratio = pick(yaml, "omni_gate_max_area_ratio", camera_index_, cfg_.max_area_ratio);
  cfg_.max_color_ratio =
    pick(yaml, "omni_gate_max_color_ratio", camera_index_, cfg_.max_color_ratio);
  cfg_.min_fill_ratio = pick(yaml, "omni_gate_min_fill_ratio", camera_index_, cfg_.min_fill_ratio);
  cfg_.min_aspect = pick(yaml, "omni_gate_min_aspect", camera_index_, cfg_.min_aspect);
  cfg_.max_aspect = pick(yaml, "omni_gate_max_aspect", camera_index_, cfg_.max_aspect);
  cfg_.min_len_px = pick(yaml, "omni_gate_min_len_px", camera_index_, cfg_.min_len_px);

  cfg_.pair_len_ratio_min =
    pick(yaml, "omni_gate_pair_len_ratio_min", camera_index_, cfg_.pair_len_ratio_min);
  cfg_.pair_len_ratio_max =
    pick(yaml, "omni_gate_pair_len_ratio_max", camera_index_, cfg_.pair_len_ratio_max);
  cfg_.pair_dy_ratio = pick(yaml, "omni_gate_pair_dy_ratio", camera_index_, cfg_.pair_dy_ratio);
  cfg_.pair_gap_ratio_min =
    pick(yaml, "omni_gate_pair_gap_ratio_min", camera_index_, cfg_.pair_gap_ratio_min);
  cfg_.pair_gap_ratio_max =
    pick(yaml, "omni_gate_pair_gap_ratio_max", camera_index_, cfg_.pair_gap_ratio_max);

  cfg_.debounce_hits = std::max(1, pick(yaml, "omni_gate_debounce_hits", camera_index_, cfg_.debounce_hits));
  cfg_.miss_frames = std::max(1, pick(yaml, "omni_gate_miss_frames", camera_index_, cfg_.miss_frames));
  cfg_.hold_ms = std::max(0, pick(yaml, "omni_gate_hold_ms", camera_index_, cfg_.hold_ms));
  cfg_.cooldown_ms = std::max(0, pick(yaml, "omni_gate_cooldown_ms", camera_index_, cfg_.cooldown_ms));
  cfg_.min_active_ms =
    std::max(0, pick(yaml, "omni_gate_min_active_ms", camera_index_, cfg_.min_active_ms));
  cfg_.max_active_ms =
    std::max(cfg_.min_active_ms, pick(yaml, "omni_gate_max_active_ms", camera_index_, cfg_.max_active_ms));
  cfg_.heartbeat_ms = std::max(0, pick(yaml, "omni_gate_heartbeat_ms", camera_index_, cfg_.heartbeat_ms));

  // 颜色：优先跟随主线程共享状态，没有则用 yaml
  if (enemy_color_) {
    color_ = enemy_color_->load(std::memory_order_relaxed);
  } else {
    const auto name = pick(yaml, "enemy_color", camera_index_, std::string("red"));
    color_ = (name == "red") ? auto_aim::Color::red : auto_aim::Color::blue;
  }
  // robot_id 还没上报时，enemy_color 只是配置兜底值，可能与实际阵营相反。
  // 此时按「颜色未知」处理（红蓝都检测），开赛初期不会因为配置写反而全向失效。
  const std::uint8_t initial_id = robot_id_ ? robot_id_->load(std::memory_order_relaxed) : 0;
  const bool id_known = robot_id_ ? (initial_id != 0) : true;
  color_known_ = id_known && (color_ == auto_aim::Color::red || color_ == auto_aim::Color::blue);

  v_min_ = static_cast<double>(cfg_.v_min);
  s_min_ = cfg_.s_min;
  last_log_ = Clock::now();

  tools::logger()->info(
    "[ColorGate] cam{} {}：scale={:.2f} divisor={} 敌方颜色={}（{}），"
    "红H=[{},{},{} ,{}] 蓝H=[{},{}] S>={} V>={}（自适应 {}）",
    camera_index_, cfg_.enable ? "启用" : "禁用", cfg_.scale, cfg_.divisor,
    color_known_ ? auto_aim::COLORS[static_cast<int>(color_)] : "unknown",
    color_known_ ? "串口 robot_id / 配置" : "未同步，检测红蓝两色", cfg_.red_h1_lo, cfg_.red_h1_hi,
    cfg_.red_h2_lo, cfg_.red_h2_hi, cfg_.blue_h_lo, cfg_.blue_h_hi, cfg_.s_min, cfg_.v_min,
    cfg_.adaptive ? "on" : "off");
}

void ColorGate::sync_color()
{
  if (!enemy_color_) return;

  const auto color = enemy_color_->load(std::memory_order_relaxed);
  const std::uint8_t id = robot_id_ ? robot_id_->load(std::memory_order_relaxed) : 0;
  const bool id_known = robot_id_ ? (id != 0) : true;
  const bool known = id_known && (color == auto_aim::Color::red || color == auto_aim::Color::blue);

  if (color == color_ && known == color_known_) return;

  color_ = color;
  color_known_ = known;

  // 换色后旧的触发窗口不再可信，直接回到 Idle 并清空去抖计数
  state_ = GateState::Idle;
  hit_streak_ = 0;
  miss_streak_ = 0;
  relax_streak_ = 0;
  active_since_ = Clock::time_point{};
  cooldown_since_ = Clock::time_point{};

  tools::logger()->info(
    "[ColorGate] cam{} 敌方颜色 -> {}（{}）", camera_index_,
    color_known_ ? auto_aim::COLORS[static_cast<int>(color_)] : "unknown",
    color_known_ ? "来自串口 robot_id" : "robot_id 未到，临时检测红蓝两色");
}

bool ColorGate::should_infer(const cv::Mat & bgr)
{
  if (!cfg_.enable) return true;
  if (bgr.empty()) return false;

  stats_.frames++;
  frame_counter_++;
  sync_color();

  const auto now = Clock::now();
  const bool first_frame = (stats_.frames == 1);

  // 主相机已锁定目标：本路不做 YOLO。颜色层照常运行并把命中计数保底到
  // debounce 阈值，主相机一旦丢失（tracking -> temp_lost/lost），下一帧就能
  // 直接进入推理，不会出现「先等 133ms 去抖」的空窗。
  if (
    cfg_.suspend_on_main_tracking && main_tracking_ &&
    main_tracking_->load(std::memory_order_relaxed)) {
    const auto color_begin = Clock::now();
    const bool hit = detect(bgr);
    stats_.color_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - color_begin).count();
    hit_this_frame_ = hit;
    if (hit) stats_.color_hits++;

    state_ = GateState::Idle;
    miss_streak_ = 0;
    hit_streak_ = hit ? std::max(hit_streak_, cfg_.debounce_hits) : 0;

    // 主相机锁定期间的「有意不推理」也算已处理，避免心跳在这里补一帧 YOLO
    last_infer_ = now;
    stats_.skipped++;
    stats_.suspended++;
    stats_.skip_suspended++;
    stats_.state = state_;
    maybe_log(now);
    return false;
  }

  // 心跳兜底：灯条熄灭（extinguish）、紫色基地等目标没有任何红蓝证据，
  // 只靠颜色层会永久漏检。用极低频的全帧推理兜住，成本 ~4ms/s。
  if (
    first_frame ||
    (cfg_.heartbeat_ms > 0 && now - last_infer_ >= std::chrono::milliseconds(cfg_.heartbeat_ms))) {
    last_infer_ = now;
    hit_this_frame_ = false;  // 心跳帧没有颜色证据，允许 observe_yolo 反哺
    stats_.heartbeats++;
    stats_.state = state_;
    maybe_log(now);
    return true;
  }

  // 空闲期抽帧：divisor > 1 时每 divisor 帧才做一次颜色检测
  if (state_ == GateState::Idle && cfg_.divisor > 1 && frame_counter_ % cfg_.divisor != 0) {
    hit_this_frame_ = false;
    stats_.skipped++;
    stats_.skip_divisor++;
    stats_.state = state_;
    maybe_log(now);
    return false;
  }

  const auto color_begin = Clock::now();
  const bool hit = detect(bgr);
  stats_.color_ms = std::chrono::duration<double, std::milli>(Clock::now() - color_begin).count();
  hit_this_frame_ = hit;
  if (hit) stats_.color_hits++;

  switch (state_) {
    case GateState::Idle:
      if (hit) {
        if (++hit_streak_ >= cfg_.debounce_hits) {
          enter_active(now);
          stats_.triggers++;
        }
      } else {
        hit_streak_ = 0;
      }
      break;

    case GateState::Active: {
      if (hit) {
        miss_streak_ = 0;
      } else {
        miss_streak_++;
      }
      const auto dwell = now - active_since_;
      const bool hold_expired = dwell >= std::chrono::milliseconds(cfg_.hold_ms) &&
                                miss_streak_ >= cfg_.miss_frames;
      const bool timeout = dwell >= std::chrono::milliseconds(cfg_.max_active_ms);
      const bool min_dwell_ok = dwell >= std::chrono::milliseconds(cfg_.min_active_ms);
      if (timeout || (hold_expired && min_dwell_ok)) enter_cooldown(now);
      break;
    }

    case GateState::Cooldown:
      if (hit) {
        // 冷却期内再次命中：立即回到推理，不额外等去抖
        enter_active(now);
        stats_.triggers++;
      } else if (now - cooldown_since_ >= std::chrono::milliseconds(cfg_.cooldown_ms)) {
        state_ = GateState::Idle;
      }
      break;
  }

  const bool infer = (state_ == GateState::Active);
  if (infer) {
    last_infer_ = now;
  } else {
    stats_.skipped++;
    // 按「为什么没推理」分类，日志里能直接看出是颜色层没命中还是别的原因
    if (state_ == GateState::Idle) {
      if (hit) {
        stats_.skip_debounce++;
      } else {
        stats_.skip_no_color++;
      }
    } else if (state_ == GateState::Cooldown) {
      stats_.skip_cooldown++;
    }
  }
  stats_.state = state_;
  maybe_log(now);
  return infer;
}

void ColorGate::observe_yolo(const std::list<auto_aim::Armor> & armors)
{
  if (!cfg_.enable || armors.empty()) return;

  const bool any_color = !color_known_;
  const bool has_enemy = std::any_of(
    armors.begin(), armors.end(), [&](const auto_aim::Armor & armor) {
      if (any_color) {
        return armor.color == auto_aim::Color::red || armor.color == auto_aim::Color::blue;
      }
      return armor.color == color_;
    });
  if (!has_enemy) return;

  stats_.yolo_hits++;
  if (hit_this_frame_) {
    relax_streak_ = 0;
    return;
  }

  // YOLO 检出了敌方装甲板，而颜色层这一帧没有命中：阈值偏严
  stats_.color_miss_yolo_hit++;
  if (!cfg_.adaptive) return;
  if (++relax_streak_ < 3) return;
  relax_streak_ = 0;

  const auto now = Clock::now();
  if (now - last_relax_ < std::chrono::seconds(30)) return;
  last_relax_ = now;

  v_min_ = std::max(static_cast<double>(cfg_.v_floor), v_min_ - 10.0);
  s_min_ = std::max(50, s_min_ - 10);

  // 几何门同样会把目标卡掉（远处灯条降采样后只有 1~2 px 宽），必须一起放宽，
  // 否则「YOLO 能检出、颜色层却永远不放行」的矛盾无法自愈。
  // 每个量都有下限，连续放宽收敛到「主要靠成对约束」，不会退化成全通过。
  cfg_.min_area_ratio = std::max(1e-5, cfg_.min_area_ratio * 0.6);
  cfg_.min_len_px = std::max(1.5, cfg_.min_len_px * 0.8);
  cfg_.min_fill_ratio = std::max(0.25, cfg_.min_fill_ratio * 0.9);
  cfg_.pair_dy_ratio = std::min(0.8, cfg_.pair_dy_ratio * 1.1);

  tools::logger()->info(
    "[ColorGate] cam{} 颜色层漏检但 YOLO 命中，单向放宽：vmin={:.0f} smin={} "
    "area_ratio={:.2e} len={:.1f} fill={:.2f} dy={:.2f}",
    camera_index_, v_min_, s_min_, cfg_.min_area_ratio, cfg_.min_len_px, cfg_.min_fill_ratio,
    cfg_.pair_dy_ratio);
}

ColorGateStats ColorGate::stats() const
{
  auto copy = stats_;
  copy.state = state_;
  return copy;
}

bool ColorGate::detect(const cv::Mat & bgr)
{
  cv::Mat small;
  cv::resize(bgr, small, cv::Size(), cfg_.scale, cfg_.scale, cv::INTER_LINEAR);

  // 自适应量（白平衡 / V 下限 / H 偏移）每 30 帧更新一次，避免逐帧抖动
  if (cfg_.adaptive && (frame_counter_ <= 1 || frame_counter_ % 30 == 0)) update_adaptive(small);

  cv::Mat balanced;
  if (std::abs(wb_r_ - 1.0) > 0.03 || std::abs(wb_b_ - 1.0) > 0.03) {
    apply_white_balance(small, balanced);
  } else {
    balanced = small;
  }

  cv::Mat hsv;
  cv::cvtColor(balanced, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask = color_mask(hsv);

  static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  const double total = static_cast<double>(mask.total());
  const double color_ratio = static_cast<double>(cv::countNonZero(mask)) / total;

  // 大面积环境色（大屏、红蓝地毯、旗帜）：本帧不作为触发证据
  if (color_ratio > cfg_.max_color_ratio) {
    if (suppression_streak_++ % 30 == 0) {
      tools::logger()->warn(
        "[ColorGate] cam{} 疑似大面积干扰色（{:.1f}% > {:.1f}%），本帧不触发", camera_index_,
        color_ratio * 100.0, cfg_.max_color_ratio * 100.0);
    }
    return false;
  }
  suppression_streak_ = 0;

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 绝对下限取 3 px²：12 px² 的旧值对细长灯条太严——1080p 降采样 0.5 后，
  // 5 m 处的灯条只有约 5×1 px（≈5 px²），会在 1 m 之外全部被滤掉。
  // 噪点通常只有 1~2 px²，仍被挡住；后续还有长宽比 / 填充率 / 成对约束。
  const double area_min = std::max(3.0, cfg_.min_area_ratio * total);
  const double area_max = cfg_.max_area_ratio * total;

  // 首次进入时把阈值换算成像素打印出来，现场调参不用再反推
  if (cfg_.debug && frame_counter_ <= 1) {
    tools::logger()->info(
      "[ColorGate] cam{} detect 输入 {}x{} 面积门 {:.1f}~{:.0f} px² 长度门 {:.1f} px "
      "成对 dy<{:.2f}×len",
      camera_index_, mask.cols, mask.rows, area_min, area_max, cfg_.min_len_px,
      cfg_.pair_dy_ratio);
  }

  struct Bar
  {
    cv::Point2f center;
    double length;
  };
  std::vector<Bar> bars;
  bars.reserve(contours.size());

  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < area_min || area > area_max) continue;

    const cv::RotatedRect rect = cv::minAreaRect(contour);
    const double w = rect.size.width;
    const double h = rect.size.height;
    // 放宽到 0.5：降采样后远处灯条宽度就在 1 px 边缘，卡 1.0 会把它们全丢掉
    if (w < 0.5 || h < 0.5) continue;

    const double len = std::max(w, h);
    const double wid = std::min(w, h);
    const double aspect = len / wid;
    if (aspect < cfg_.min_aspect || aspect > cfg_.max_aspect) continue;
    if (len < cfg_.min_len_px) continue;
    if (area / (w * h) < cfg_.min_fill_ratio) continue;

    bars.push_back({rect.center, len});
  }

  // 成对约束：装甲板一定由两条近似等长、近似水平排列的灯条组成。
  // 大屏是一整片低长宽比区域、衣服/旗帜没有成对竖直细长结构，都会在这里被拒。
  bool paired = false;
  for (std::size_t i = 0; i < bars.size(); i++) {
    for (std::size_t j = i + 1; j < bars.size(); j++) {
      const double mean_len = 0.5 * (bars[i].length + bars[j].length);
      if (mean_len < 1.0) continue;

      const double len_ratio = bars[i].length / bars[j].length;
      if (len_ratio < cfg_.pair_len_ratio_min || len_ratio > cfg_.pair_len_ratio_max) continue;

      const double dx = std::abs(bars[i].center.x - bars[j].center.x);
      const double dy = std::abs(bars[i].center.y - bars[j].center.y);
      if (dy > cfg_.pair_dy_ratio * mean_len) continue;

      const double gap = dx / mean_len;
      if (gap < cfg_.pair_gap_ratio_min || gap > cfg_.pair_gap_ratio_max) continue;

      paired = true;
      break;
    }
    if (paired) break;
  }

  // 调试开关：每 30 帧打印一次中间量，用来判断是「颜色没过」还是「形状/成对没过」。
  // 颜色占比 ≈ 0 且轮廓 0  -> 阈值或 enemy_color 不对
  // 有轮廓但灯条 0        -> 面积/长宽比/填充率太严
  // 有灯条但不成对        -> 灯条方向不对（旋转没配对）或成对阈值太严
  if (cfg_.debug && frame_counter_ % 30 == 0) {
    tools::logger()->info(
      "[ColorGate] cam{} debug：颜色占比 {:.2f}% 轮廓 {} 灯条 {} 成对 {}", camera_index_,
      color_ratio * 100.0, contours.size(), bars.size(), paired ? "是" : "否");
  }
  return paired;
}

cv::Mat ColorGate::color_mask(const cv::Mat & hsv) const
{
  if (color_known_) {
    return color_ == auto_aim::Color::red ? red_mask(hsv, h_shift_red_)
                                          : blue_mask(hsv, h_shift_blue_);
  }

  // 颜色未同步：红蓝都查，宁多触发一次 YOLO，也不能让全向失效
  cv::Mat red = red_mask(hsv, 0);
  cv::Mat blue = blue_mask(hsv, 0);
  cv::Mat mask;
  cv::bitwise_or(red, blue, mask);
  return mask;
}

cv::Mat ColorGate::red_mask(const cv::Mat & hsv, int shift) const
{
  // 自适应 H 偏移只「扩展」窗口，绝不整体平移：平移会把窗口另一侧的
  // 真实目标挤出去（H=0 的纯红曾被 shift=+2 排除在 [2,10] 之外）。
  const int vmin = static_cast<int>(std::lround(v_min_));
  const int lo1 = std::clamp(cfg_.red_h1_lo + std::min(shift, 0), 0, 179);
  const int hi1 = std::clamp(cfg_.red_h1_hi + std::max(shift, 0), 0, 179);
  const int lo2 = std::clamp(cfg_.red_h2_lo + std::min(shift, 0), 0, 179);
  const int hi2 = std::clamp(cfg_.red_h2_hi + std::max(shift, 0), 0, 179);

  cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8U);
  cv::Mat part;

  const auto add_segment = [&](int lo, int hi) {
    if (lo > hi) return;
    cv::inRange(hsv, cv::Scalar(lo, s_min_, vmin), cv::Scalar(hi, 255, 255), part);
    cv::bitwise_or(mask, part, mask);
    // 过曝白芯：灯条中心 V 很高时 S 会掉到 40~70，S 下限要放宽
    cv::inRange(hsv, cv::Scalar(lo, 40, 245), cv::Scalar(hi, 255, 255), part);
    cv::bitwise_or(mask, part, mask);
  };

  add_segment(lo1, hi1);
  add_segment(lo2, hi2);
  return mask;
}

cv::Mat ColorGate::blue_mask(const cv::Mat & hsv, int shift) const
{
  const int lo = std::clamp(cfg_.blue_h_lo + std::min(shift, 0), 0, 179);
  const int hi = std::clamp(cfg_.blue_h_hi + std::max(shift, 0), 0, 179);
  if (lo > hi) return cv::Mat::zeros(hsv.size(), CV_8U);

  const int vmin = static_cast<int>(std::lround(v_min_));

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(lo, s_min_, vmin), cv::Scalar(hi, 255, 255), mask);

  cv::Mat core;
  cv::inRange(hsv, cv::Scalar(lo, 40, 245), cv::Scalar(hi, 255, 255), core);
  cv::bitwise_or(mask, core, mask);
  return mask;
}

void ColorGate::update_adaptive(const cv::Mat & small)
{
  cv::Mat tiny;
  cv::resize(small, tiny, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
  if (tiny.empty()) return;

  // 1) 灰度世界白平衡：整幅画面的通道均值当作光源色，限幅后一阶低通
  const cv::Scalar mean = cv::mean(tiny);
  const double mg = std::max(1.0, mean[1]);
  const double target_r = std::clamp(mg / std::max(1.0, mean[2]), 0.85, 1.18);
  const double target_b = std::clamp(mg / std::max(1.0, mean[0]), 0.85, 1.18);
  wb_r_ = 0.9 * wb_r_ + 0.1 * target_r;
  wb_b_ = 0.9 * wb_b_ + 0.1 * target_b;

  cv::Mat hsv;
  cv::cvtColor(tiny, hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);

  // 2) V 的 95 分位 -> 自适应下限。单帧变化限幅 4，避免闪光/阴影导致阈值跳变
  int hist_size = 32;
  float v_range[2] = {0, 256};
  const float * v_ranges[] = {v_range};
  int v_channels[] = {0};
  cv::Mat v_hist;
  cv::calcHist(&channels[2], 1, v_channels, cv::noArray(), v_hist, 1, &hist_size, v_ranges);

  const double total = static_cast<double>(tiny.total());
  double acc = 0.0;
  double p95 = 255.0;
  for (int i = 0; i < hist_size; i++) {
    acc += v_hist.at<float>(i);
    if (acc >= total * 0.95) {
      p95 = (i + 0.5) * 8.0;
      break;
    }
  }

  const double target_v =
    std::clamp(0.45 * p95, static_cast<double>(cfg_.v_floor), static_cast<double>(cfg_.v_ceil));
  v_min_ = std::clamp(0.9 * v_min_ + 0.1 * target_v, v_min_ - 4.0, v_min_ + 4.0);
  v_min_ = std::clamp(v_min_, static_cast<double>(cfg_.v_floor), static_cast<double>(cfg_.v_ceil));

  // 3) H 偏移：高饱和像素的 H 直方图峰值相对「红 0°」「蓝 112°」的偏移。
  //    场馆灯光色温造成的整体偏色主要体现为 H 漂移，比调 V 更对症。
  cv::Mat sat_mask;
  cv::inRange(hsv, cv::Scalar(0, 120, 100), cv::Scalar(179, 255, 255), sat_mask);
  const double sat_count = static_cast<double>(cv::countNonZero(sat_mask));
  if (sat_count < 0.001 * total) return;

  int h_bins = 36;  // 5° 一档
  float h_range[2] = {0, 180};
  const float * h_ranges[] = {h_range};
  int h_channels[] = {0};
  cv::Mat h_hist;
  cv::calcHist(&channels[0], 1, h_channels, sat_mask, h_hist, 1, &h_bins, h_ranges);

  const auto peak_of = [&](std::initializer_list<int> bins) {
    int best = -1;
    float best_value = 0.0f;
    for (int b : bins) {
      if (b < 0 || b >= h_bins) continue;
      const float value = h_hist.at<float>(b);
      if (value > best_value) {
        best_value = value;
        best = b;
      }
    }
    return std::pair<int, float>(best, best_value);
  };

  // 峰值占比 > 5% 说明是大面积色块（大屏/围栏），不能用来标定色温
  const float max_trust = static_cast<float>(0.05 * total);
  const float min_trust = static_cast<float>(0.002 * total);

  const auto red_peak = peak_of({0, 1, 2, 33, 34, 35});
  if (red_peak.first >= 0 && red_peak.second > min_trust && red_peak.second < max_trust) {
    int deg = static_cast<int>((red_peak.first + 0.5) * 5.0);
    if (deg > 90) deg -= 180;
    h_shift_red_ = std::clamp(deg, -8, 8);
  }

  const auto blue_peak = peak_of({19, 20, 21, 22, 23, 24, 25, 26});
  if (blue_peak.first >= 0 && blue_peak.second > min_trust && blue_peak.second < max_trust) {
    const int deg = static_cast<int>((blue_peak.first + 0.5) * 5.0);
    h_shift_blue_ = std::clamp(deg - 112, -8, 8);
  }
}

void ColorGate::apply_white_balance(const cv::Mat & src, cv::Mat & dst) const
{
  cv::Mat m = cv::Mat::eye(3, 3, CV_32F);
  m.at<float>(0, 0) = static_cast<float>(wb_b_);
  m.at<float>(2, 2) = static_cast<float>(wb_r_);
  cv::transform(src, dst, m);
}

void ColorGate::enter_active(std::chrono::steady_clock::time_point now)
{
  state_ = GateState::Active;
  active_since_ = now;
  hit_streak_ = 0;
  miss_streak_ = 0;
  if (cfg_.debug) {
    tools::logger()->info("[ColorGate] cam{} 进入推理窗口", camera_index_);
  }
}

void ColorGate::enter_cooldown(std::chrono::steady_clock::time_point now)
{
  state_ = GateState::Cooldown;
  cooldown_since_ = now;
  hit_streak_ = 0;
  miss_streak_ = 0;
  if (cfg_.debug) {
    tools::logger()->info("[ColorGate] cam{} 退出推理，进入冷却", camera_index_);
  }
}

void ColorGate::maybe_log(std::chrono::steady_clock::time_point now)
{
  if (now - last_log_ < std::chrono::seconds(5)) return;
  last_log_ = now;

  const double hit_ratio =
    stats_.frames > 0 ? 100.0 * static_cast<double>(stats_.color_hits) / stats_.frames : 0.0;
  const double skip_ratio =
    stats_.frames > 0 ? 100.0 * static_cast<double>(stats_.skipped) / stats_.frames : 0.0;

  const auto part = [&](std::uint64_t value) {
    return stats_.frames > 0
             ? 100.0 * static_cast<double>(value) / static_cast<double>(stats_.frames)
             : 0.0;
  };

  // skip 的原因分解是排查「总是 skip」最快的手段：susp 高 = 主相机在 tracking；
  // 无色高 = 颜色阈值/方向/颜色配置不对；去抖高 = 命中了但没连续够帧。
  tools::logger()->info(
    "[ColorGate] cam{} {} hit={:.1f}% skip={:.0f}% [susp {:.0f} | 无色 {:.0f} | 去抖 {:.0f} | "
    "冷却 {:.0f} | 抽帧 {:.0f}] trig={} hb={} yolo/miss={}/{} vmin={:.0f} smin={} hshift={}/{} "
    "{:.2f}ms",
    camera_index_, gate_state_name(state_), hit_ratio, skip_ratio, part(stats_.skip_suspended),
    part(stats_.skip_no_color), part(stats_.skip_debounce), part(stats_.skip_cooldown),
    part(stats_.skip_divisor), stats_.triggers, stats_.heartbeats, stats_.yolo_hits,
    stats_.color_miss_yolo_hit, v_min_, s_min_, h_shift_red_, h_shift_blue_, stats_.color_ms);
}

}  // namespace omniperception
