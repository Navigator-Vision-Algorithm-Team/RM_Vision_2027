#ifndef OMNIPERCEPTION__STITCHED_OMNI_HPP
#define OMNIPERCEPTION__STITCHED_OMNI_HPP

#include <atomic>
#include <chrono>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "decider.hpp"
#include "detection.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace omniperception
{

// =============================================================================
//  全向相机「all in one」融合：四路画面按 2×2 拼成一张大图，只做一次 YOLO 推理。
//
//  布局（索引 = row * cols + col，与 omni 相机序号一致）：
//
//      +---------+---------+
//      | omni 0  | omni 1  |     col  = index % cols
//      +---------+---------+     row  = index / cols
//      | omni 2  | omni 3  |
//      +---------+---------+
//
//  分区反推：检测框中心 (x, y) 落在哪个 cell 就归属哪一路相机：
//      col = clamp(x / cell_w, 0, cols-1)
//      row = clamp(y / cell_h, 0, rows-1)
//
//  坐标映射（大图坐标 -> 该相机原始图像坐标）：
//      x_cam = (x_big - col * cell_w) * scale_x
//      y_cam = (y_big - row * cell_h) * scale_y
//      scale_x = 相机原始宽度 / cell_w
//      scale_y = 相机原始高度 / cell_h
//  因为每路画面在拼接时被 resize 到 cell 尺寸，反推必须乘回该缩放比。
//
//  ratio / side_ratio / rectangular_error 都是无量纲比值和夹角，缩放不变，
//  映射后无需重算；points / center / box / center_norm 必须逐项映射。
//
//  两条与「拼接」强绑定的边界规则：
//    · 空相机通道：该路本帧没有画面时对应分区保持纯黑，并且这一格里的任何检测
//      都判为误检直接丢弃（不拿上一帧的残影当结果）。
//    · 跨拼接分界线：分区边界是硬裁剪，跨线目标只有一部分像素，角点/灯条几何
//      全部失真，PnP 会解出明显偏掉的角度，因此整条检测直接丢弃。
// =============================================================================
struct StitchLayout
{
  int cols = 2;
  int rows = 2;
  cv::Size cell{320, 240};  // 单个分区尺寸 = 每路画面 resize 后的尺寸

  // 跨分界线的判定容差 (px)：角点必须落在分区边界【内缩 margin】以内才算安全。
  // 0 表示贴边即算跨线；边界同时也是每路画面的边缘，贴边目标一律视为被裁断。
  float boundary_margin = 2.0f;

  int index_of(int col, int row) const { return row * cols + col; }
  int col_of(int index) const { return index % cols; }
  int row_of(int index) const { return index / cols; }
  cv::Size canvas_size() const { return {cell.width * cols, cell.height * rows}; }
  cv::Rect cell_rect(int index) const
  {
    return {col_of(index) * cell.width, row_of(index) * cell.height, cell.width, cell.height};
  }
};

// 四路全向相机的融合检测线程：
//   * 单个线程依次读取 4 路相机（各自队列里只保留最新帧，不会累积延迟）；
//   * 拼成一张大图后交给**一个** YOLO 实例做一次推理；
//   * 按框中心分区、把坐标映射回对应相机的原始图像；
//   * 每路复用独立的 Decider 完成「敌方过滤 -> 排序 -> PnP 解算」，
//     产出与 YoloPipeline 全向路径完全相同的 DetectionResult（armors + delta_yaw/delta_pitch）。
//
// 这样 4 路相机的推理开销从「4 个 YOLO 实例」降为「1 个」，但每路的有效分辨率
// 也从原始帧降到 cell 尺寸（默认 320×240），远距离小目标的召回会下降。
class StitchedOmni
{
public:
  StitchedOmni(
    const std::string & config_path, const std::vector<OmniCameraConfig> & configs,
    bool debug = false);
  ~StitchedOmni();

  StitchedOmni(const StitchedOmni &) = delete;
  StitchedOmni & operator=(const StitchedOmni &) = delete;

  // 比赛开始前关闭推理：相机照常读数（保持队列新鲜），但不再做拼接与推理。
  void set_inference_enabled(bool enabled) { inference_enabled_ = enabled; }

  // 运行时切换敌方颜色（由串口上报的 robot_id 决定），下一帧生效。
  void set_enemy_color(auto_aim::Color color);

  // 取走当前积压的所有全向结果（与 YoloPipeline::drain_omni 语义一致）。
  void drain(std::vector<DetectionResult> & results);

  const StitchLayout & layout() const { return layout_; }

private:
  void run();

  // 把一次大图推理的结果按分区拆开：分区反推 -> 黑格/跨线剔除 -> 坐标映射
  // -> 敌方过滤 -> 排序 -> PnP。
  void dispatch(
    const std::list<auto_aim::Armor> & armors, const std::vector<cv::Mat> & frames,
    const std::vector<std::chrono::steady_clock::time_point> & timestamps,
    std::vector<DetectionResult> & results);

  // 大图坐标 -> 第 index 路相机的原始图像坐标；该路分辨率未知时返回 false。
  bool map_to_camera(
    auto_aim::Armor & armor, int index, const cv::Size & source_size) const;

  void push(std::vector<DetectionResult> && results);

  std::string config_path_;
  bool debug_;
  std::vector<OmniCameraConfig> configs_;
  std::vector<std::unique_ptr<Decider>> deciders_;  // 每路一个，避免 count_ 等状态混用
  StitchLayout layout_;

  cv::Mat canvas_;                              // 复用拼接缓冲
  std::vector<cv::Size> source_sizes_;          // 各路原始分辨率（首帧到达时确定）
  auto_aim::YOLO yolo_;                         // 单实例：不绑核，OpenVINO 自主用满所有核

  std::thread thread_;
  std::atomic<bool> quit_{false};
  std::atomic<bool> inference_enabled_{true};

  // 敌方颜色：主线程写、推理线程读
  std::mutex color_mutex_;
  auto_aim::Color pending_color_ = auto_aim::Color::red;
  std::atomic<bool> color_dirty_{false};

  std::mutex result_mutex_;
  std::deque<DetectionResult> results_;
  static constexpr std::size_t kMaxResults = 32;
};

}  // namespace omniperception

#endif  // OMNIPERCEPTION__STITCHED_OMNI_HPP
