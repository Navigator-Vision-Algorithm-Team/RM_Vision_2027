#ifndef OMNIPERCEPTION__COLOR_GATE_HPP
#define OMNIPERCEPTION__COLOR_GATE_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <list>
#include <string>

#include <opencv2/core.hpp>

#include "tasks/auto_aim/armor.hpp"

namespace omniperception
{

// 两级流水线的 L0 状态机：
//   Idle     只跑颜色检测（空闲期可抽帧），YOLO 不执行
//   Active   命中敌方颜色后持续推理
//   Cooldown 退出后冷却，期间再次命中可立即回到 Active
enum class GateState
{
  Idle,
  Active,
  Cooldown
};

const char * gate_state_name(GateState state);

/*
 * 颜色门控参数。全部带默认值：yaml 缺项时按默认值运行，整段删掉也能跑。
 * 每个键都支持 omni_gate_<key>_omniN 覆盖（N 从 1 开始，与相机序号一致）。
 */
struct ColorGateConfig
{
  bool enable = true;
  bool adaptive = true;
  bool debug = false;

  // ---- 频率 ----
  int divisor = 1;     // 空闲期颜色检测抽帧倍数（1 = 每帧，2 = 半帧率）
  double scale = 0.5;  // 颜色检测降采样比例（0.5 才能保住远端小灯条）

  // ---- HSV 阈值（OpenCV 的 H 范围是 0~179）----
  int red_h1_lo = 0;
  int red_h1_hi = 8;
  int red_h2_lo = 170;  // 红色跨 0 点，必须两段
  int red_h2_hi = 179;
  int blue_h_lo = 98;  // 蓝灯条实际偏青，中心约 112
  int blue_h_hi = 132;
  int s_min = 90;
  int v_min = 90;    // 自适应初值
  int v_floor = 35;  // 自适应下限
  int v_ceil = 130;  // 自适应上限

  // ---- 面积 / 形状（比例化，适配任意分辨率）----
  double min_area_ratio = 0.00015;
  double max_area_ratio = 0.015;
  double max_color_ratio = 0.12;  // 全屏命中占比闸门（大屏 / 地毯）
  double min_fill_ratio = 0.55;
  double min_aspect = 1.6;
  double max_aspect = 18.0;
  double min_len_px = 4.0;

  // ---- 灯条成对约束：把「有颜色」升级成「有装甲板」----
  double pair_len_ratio_min = 0.65;
  double pair_len_ratio_max = 1.5;
  double pair_dy_ratio = 0.35;
  double pair_gap_ratio_min = 1.2;
  double pair_gap_ratio_max = 5.0;

  // ---- 时序（毫秒）----
  int debounce_hits = 2;
  int miss_frames = 5;
  int hold_ms = 1000;
  int cooldown_ms = 400;
  int min_active_ms = 300;
  int max_active_ms = 5000;
  int heartbeat_ms = 4000;  // 兜底心跳推理，0 = 关闭
};

struct ColorGateStats
{
  GateState state = GateState::Idle;
  std::uint64_t frames = 0;
  std::uint64_t skipped = 0;  // 跳过了 YOLO 的帧数
  std::uint64_t color_hits = 0;
  std::uint64_t triggers = 0;
  std::uint64_t heartbeats = 0;
  std::uint64_t yolo_hits = 0;
  std::uint64_t color_miss_yolo_hit = 0;  // 颜色层漏检但 YOLO 检出的帧
  double color_ms = 0.0;
};

/*
 * 常驻的低功耗颜色门控（两级流水线的 L0）。
 *
 * 只有画面里出现「敌方颜色的成对灯条」时才允许本帧执行 YOLO，否则整帧跳过。
 * YOLO 实例本身常驻内存，跳帧只是不调用 detect()，没有模型加载 / 卸载开销。
 *
 * enemy_color 指向主线程的 OmniSharedState::enemy_color 时会跟随串口 robot_id
 * 自动换色；为空时退化为 yaml 的 enemy_color。
 *
 * 线程模型：每路相机一个实例，只在它自己的推理线程里调用，无需加锁。
 */
class ColorGate
{
public:
  ColorGate(
    const std::string & config_path, int camera_index,
    std::atomic<auto_aim::Color> * enemy_color = nullptr);

  // 本帧是否执行 YOLO。
  bool should_infer(const cv::Mat & bgr);

  // 推理结束后回报 YOLO 结果（必须在 armor_filter 之前调用）：
  // 检出敌方装甲板、但颜色层本帧没命中时，说明阈值偏严，单向放宽。
  void observe_yolo(const std::list<auto_aim::Armor> & armors);

  ColorGateStats stats() const;
  bool enabled() const { return cfg_.enable; }

private:
  void sync_color();
  bool detect(const cv::Mat & bgr);
  cv::Mat color_mask(const cv::Mat & hsv) const;
  cv::Mat red_mask(const cv::Mat & hsv, int shift) const;
  cv::Mat blue_mask(const cv::Mat & hsv, int shift) const;
  void update_adaptive(const cv::Mat & small);
  void apply_white_balance(const cv::Mat & src, cv::Mat & dst) const;
  void enter_active(std::chrono::steady_clock::time_point now);
  void enter_cooldown(std::chrono::steady_clock::time_point now);
  void maybe_log(std::chrono::steady_clock::time_point now);

  ColorGateConfig cfg_;
  int camera_index_ = 0;
  std::atomic<auto_aim::Color> * enemy_color_ = nullptr;

  auto_aim::Color color_ = auto_aim::Color::red;
  bool color_known_ = true;

  GateState state_ = GateState::Idle;
  int hit_streak_ = 0;
  int miss_streak_ = 0;
  int relax_streak_ = 0;
  int suppression_streak_ = 0;
  bool hit_this_frame_ = false;
  std::uint64_t frame_counter_ = 0;

  std::chrono::steady_clock::time_point active_since_{};
  std::chrono::steady_clock::time_point cooldown_since_{};
  std::chrono::steady_clock::time_point last_infer_{};
  std::chrono::steady_clock::time_point last_relax_{};
  std::chrono::steady_clock::time_point last_log_{};

  // 自适应量
  double wb_r_ = 1.0;
  double wb_b_ = 1.0;
  double v_min_ = 90.0;
  int s_min_ = 90;
  int h_shift_red_ = 0;
  int h_shift_blue_ = 0;

  ColorGateStats stats_;
};

}  // namespace omniperception

#endif  // OMNIPERCEPTION__COLOR_GATE_HPP
