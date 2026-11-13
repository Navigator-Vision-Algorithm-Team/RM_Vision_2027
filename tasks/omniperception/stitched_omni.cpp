#include "stitched_omni.hpp"

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/recorder.hpp"

namespace omniperception
{
namespace
{

StitchLayout load_layout(const YAML::Node & yaml)
{
  StitchLayout layout;
  if (yaml["omni_stitch_cols"]) layout.cols = yaml["omni_stitch_cols"].as<int>();
  if (yaml["omni_stitch_rows"]) layout.rows = yaml["omni_stitch_rows"].as<int>();
  if (yaml["omni_stitch_cell_width"]) {
    layout.cell.width = yaml["omni_stitch_cell_width"].as<int>();
  }
  if (yaml["omni_stitch_cell_height"]) {
    layout.cell.height = yaml["omni_stitch_cell_height"].as<int>();
  }
  if (yaml["omni_stitch_boundary_margin"]) {
    layout.boundary_margin = yaml["omni_stitch_boundary_margin"].as<float>();
  }
  layout.cols = std::max(1, layout.cols);
  layout.rows = std::max(1, layout.rows);
  layout.cell.width = std::max(1, layout.cell.width);
  layout.cell.height = std::max(1, layout.cell.height);
  layout.boundary_margin = std::max(0.0f, layout.boundary_margin);
  return layout;
}

// 检测框是否跨越了拼接分界线。
// 拼接时每个分区都是从原图硬裁剪出来的，跨线的目标只有一部分像素落在本分区内，
// 模型补出来的另外半块必然越界；此时角点、灯条长度、中心点全部失真，PnP 解出的
// 角度会明显偏离真实方向，宁可这一帧不报也不能报一个错的角度。
// 判定方式：四个角点必须全部落在「分区边界内缩 margin」的矩形里。
bool crosses_seam(const auto_aim::Armor & armor, const cv::Rect & cell, float margin)
{
  // 没有角点（非 YOLO 路径的装甲板）无法判断归属边界，按跨线处理，直接丢弃。
  if (armor.points.empty()) return true;

  const float x0 = static_cast<float>(cell.x) + margin;
  const float y0 = static_cast<float>(cell.y) + margin;
  const float x1 = static_cast<float>(cell.x + cell.width) - margin;
  const float y1 = static_cast<float>(cell.y + cell.height) - margin;

  // 分区尺寸被 margin 吃到没有有效区域时，任何检测都不可能通过
  if (x1 <= x0 || y1 <= y0) return true;

  for (const auto & point : armor.points) {
    if (point.x < x0 || point.x > x1 || point.y < y0 || point.y > y1) return true;
  }
  return false;
}

// 灯条几何量随坐标一起映射。ratio/angle_error 由 length/width 与方向算出，
// 纯缩放不改变它们，但 center/top/bottom/points 必须逐项换算。
void remap_lightbar(auto_aim::Lightbar & lightbar, float ox, float oy, float sx, float sy)
{
  auto map = [&](const cv::Point2f & p) {
    return cv::Point2f((p.x - ox) * sx, (p.y - oy) * sy);
  };

  lightbar.center = map(lightbar.center);
  lightbar.top = map(lightbar.top);
  lightbar.bottom = map(lightbar.bottom);
  lightbar.points.clear();
  lightbar.points.push_back(lightbar.top);
  lightbar.points.push_back(lightbar.bottom);
  lightbar.top2bottom = lightbar.bottom - lightbar.top;
  lightbar.length = cv::norm(lightbar.top2bottom);
  lightbar.width *= (sx + sy) * 0.5f;
  lightbar.angle = std::atan2(lightbar.top2bottom.y, lightbar.top2bottom.x);
  lightbar.angle_error = std::abs(lightbar.angle - static_cast<float>(CV_PI / 2));
}

}  // namespace

StitchedOmni::StitchedOmni(
  const std::string & config_path, const std::vector<OmniCameraConfig> & configs, bool debug)
: config_path_(config_path), debug_(debug), configs_(configs), layout_(load_layout(YAML::LoadFile(config_path))),
  yolo_(config_path, debug, false)
{
  if (configs_.empty()) throw std::runtime_error("[StitchedOmni] 至少需要一路全向相机");

  const std::size_t cells = static_cast<std::size_t>(layout_.cols) * layout_.rows;
  if (configs_.size() > cells) {
    throw std::runtime_error(fmt::format(
      "[StitchedOmni] 相机数 {} 超过拼接分区数 {}（cols={} rows={}）", configs_.size(), cells,
      layout_.cols, layout_.rows));
  }

  deciders_.reserve(configs_.size());
  for (std::size_t i = 0; i < configs_.size(); i++) {
    deciders_.push_back(std::make_unique<Decider>(config_path));
  }
  source_sizes_.assign(configs_.size(), cv::Size());

  canvas_ = cv::Mat(layout_.canvas_size(), CV_8UC3, cv::Scalar(0, 0, 0));

  // 布局与映射关系在启动时打印一次，方便现场核对分区和相机的对应关系。
  tools::logger()->info(
    "[StitchedOmni] 拼接布局 {}×{}，单分区 {}×{}，大图 {}×{}；"
    "分区索引 = row*{} + col（col = 索引%{}，row = 索引/{}）",
    layout_.cols, layout_.rows, layout_.cell.width, layout_.cell.height,
    layout_.canvas_size().width, layout_.canvas_size().height, layout_.cols, layout_.cols,
    layout_.cols);
  for (std::size_t i = 0; i < configs_.size(); i++) {
    tools::logger()->info(
      "[StitchedOmni] omni{} -> 分区 (col={}, row={})，大图区域 x:[{}, {}) y:[{}, {})",
      i + 1, layout_.col_of(static_cast<int>(i)), layout_.row_of(static_cast<int>(i)),
      layout_.col_of(static_cast<int>(i)) * layout_.cell.width,
      (layout_.col_of(static_cast<int>(i)) + 1) * layout_.cell.width,
      layout_.row_of(static_cast<int>(i)) * layout_.cell.height,
      (layout_.row_of(static_cast<int>(i)) + 1) * layout_.cell.height);
  }

  // 大图尺寸 vs 模型输入尺寸：两者不一致时 letterbox 会整体缩放，
  // 每路在模型里的有效分辨率恒为「(cell × min(输入/大图))」，把 cell 调大也没用。
  // 这里把结论一次性打印清楚，避免现场调 omni_stitch_cell_* 却看不出效果。
  {
    const cv::Size model_input = yolo_.input_size();
    const cv::Size canvas_size = layout_.canvas_size();
    const double fit = std::min(
      static_cast<double>(model_input.width) / canvas_size.width,
      static_cast<double>(model_input.height) / canvas_size.height);
    const int eff_w = static_cast<int>(std::lround(layout_.cell.width * fit));
    const int eff_h = static_cast<int>(std::lround(layout_.cell.height * fit));

    if (fit < 1.0) {
      tools::logger()->warn(
        "[StitchedOmni] 大图 {}×{} 超过 YOLO 输入 {}×{}，letterbox 会整体缩小 {:.3f} 倍，"
        "每路在模型里只剩 {}×{} —— 调大 omni_stitch_cell_* 不会提升精度。"
        "要用满拼接分辨率，请把模型重新导出为 {}×{}（32 的倍数，正好 1:1 无黑边）"
        "并替换 yolov5_model_path",
        canvas_size.width, canvas_size.height, model_input.width, model_input.height, fit, eff_w,
        eff_h, canvas_size.width, canvas_size.height);
    } else {
      tools::logger()->info(
        "[StitchedOmni] 每路在模型里占 {}×{}（大图 {}×{} / 输入 {}×{}，缩放 {:.3f}）", eff_w,
        eff_h, canvas_size.width, canvas_size.height, model_input.width, model_input.height, fit);
    }
  }

  thread_ = std::thread(&StitchedOmni::run, this);
}

StitchedOmni::~StitchedOmni()
{
  quit_ = true;
  // join 前先置全局退出标志：推理线程可能正阻塞在 camera->read() 里等一帧，
  // HikRobot / 虚拟数据源读到该标志后会立即返回空帧，线程随即看到 quit_。
  tools::request_exit();
  if (thread_.joinable()) thread_.join();
  tools::logger()->info("[StitchedOmni] 已停止。");
}

void StitchedOmni::set_enemy_color(auto_aim::Color color)
{
  {
    std::lock_guard<std::mutex> lock(color_mutex_);
    pending_color_ = color;
  }
  color_dirty_ = true;
}

void StitchedOmni::drain(std::vector<DetectionResult> & results)
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  results.reserve(results.size() + results_.size());
  while (!results_.empty()) {
    results.push_back(std::move(results_.front()));
    results_.pop_front();
  }
}

void StitchedOmni::push(std::vector<DetectionResult> && results)
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  for (auto & result : results) results_.push_back(std::move(result));
  while (results_.size() > kMaxResults) results_.pop_front();
}

void StitchedOmni::run()
{
  const std::size_t n = configs_.size();
  std::vector<cv::Mat> frames(n);
  std::vector<std::chrono::steady_clock::time_point> timestamps(n);
  bool size_logged = false;

  while (!quit_) {
    // 1) 依次读取 4 路。各相机队列只保留最新帧，阻塞等待新帧不会累积延迟
    //    （15 fps 时最慢约 66 ms）。
    bool any = false;
    for (std::size_t i = 0; i < n && !quit_; i++) {
      cv::Mat img;
      std::chrono::steady_clock::time_point timestamp;
      configs_[i].camera->read(img, timestamp);
      if (img.empty()) {
        // 该路本帧没有画面（掉线 / 数据源结束）：清空缓存，让对应分区真正变黑。
        // 不清空的话上一帧会被一直重新拼进大图，YOLO 会在冻结画面上反复检出同一个
        // 目标，而 dispatch 又拿不到「这帧是旧的」这个信息，云台会被它一直拽着走。
        frames[i].release();
        continue;
      }

      frames[i] = img;
      timestamps[i] = timestamp;
      any = true;

      if (source_sizes_[i].width <= 0) {
        source_sizes_[i] = img.size();
        size_logged = true;
      }
    }

    if (quit_) break;
    if (!any) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }

    if (size_logged) {
      size_logged = false;
      for (std::size_t i = 0; i < n; i++) {
        if (source_sizes_[i].width <= 0) continue;
        const double sx = static_cast<double>(source_sizes_[i].width) / layout_.cell.width;
        const double sy = static_cast<double>(source_sizes_[i].height) / layout_.cell.height;
        tools::logger()->info(
          "[StitchedOmni] omni{} 原始 {}×{} -> 分区 {}×{}（scale_x={:.3f} scale_y={:.3f}）",
          i + 1, source_sizes_[i].width, source_sizes_[i].height, layout_.cell.width,
          layout_.cell.height, sx, sy);

        // 分区与原始画面长宽比不一致时 cv::resize 会直接拉伸，装甲板被压扁/拉长，
        // ratio/side_ratio/rectangular_error 跟着失真（那几个量只在等比缩放下不变），
        // 传统的角点二次矫正也会失配。这里只告警，不改变布局。
        if (std::abs(sx - sy) > 0.05 * std::max(sx, sy)) {
          tools::logger()->warn(
            "[StitchedOmni] omni{} 分区 {}×{} 与原始 {}×{} 长宽比不一致（scale_x={:.3f} "
            "scale_y={:.3f}），画面会被拉伸；请把 omni_stitch_cell_* 改成与相机同长宽比",
            i + 1, layout_.cell.width, layout_.cell.height, source_sizes_[i].width,
            source_sizes_[i].height, sx, sy);
        }
      }
    }

    // 赛前不做拼接与推理：仍读帧保持队列新鲜，恢复比赛时不会吃到陈旧画面。
    if (!inference_enabled_.load(std::memory_order_relaxed)) continue;

    // 2) 拼接：每路缩放到分区尺寸，写入 2×2 大图的对应区域。
    //    空相机通道（frames[i] 已空）所在分区保持纯黑：先整幅清 0，再只覆盖有帧的分区。
    canvas_.setTo(cv::Scalar(0, 0, 0));
    for (std::size_t i = 0; i < n; i++) {
      if (frames[i].empty()) continue;
      cv::Mat cell = canvas_(layout_.cell_rect(static_cast<int>(i)));
      if (frames[i].size() == layout_.cell) {
        // 分区尺寸 = 原始分辨率（cell 设成 640×480 时）：1:1 直拷，省掉一次无效重采样
        frames[i].copyTo(cell);
      } else {
        cv::resize(frames[i], cell, layout_.cell, 0, 0, cv::INTER_LINEAR);
      }
    }

    // 3) 一次推理：输入是整张大图，输出框坐标已经是「大图坐标」。
    auto armors = yolo_.detect(canvas_);

    // 4) 分区反推 + 坐标映射 + 每路 PnP 解算。
    std::vector<DetectionResult> results;
    dispatch(armors, frames, timestamps, results);
    if (!results.empty()) push(std::move(results));
  }
}

bool StitchedOmni::map_to_camera(
  auto_aim::Armor & armor, int index, const cv::Size & source_size) const
{
  if (source_size.width <= 0 || source_size.height <= 0) return false;

  // 分区左上角在大图中的位置
  const float ox = static_cast<float>(layout_.col_of(index) * layout_.cell.width);
  const float oy = static_cast<float>(layout_.row_of(index) * layout_.cell.height);

  // 相机原始分辨率 / 分区尺寸：拼接时做了 resize，反推必须乘回该比例
  const float sx = static_cast<float>(source_size.width) / layout_.cell.width;
  const float sy = static_cast<float>(source_size.height) / layout_.cell.height;

  auto map = [&](const cv::Point2f & p) {
    return cv::Point2f((p.x - ox) * sx, (p.y - oy) * sy);
  };

  for (auto & point : armor.points) point = map(point);
  armor.center = map(armor.center);
  remap_lightbar(armor.left, ox, oy, sx, sy);
  remap_lightbar(armor.right, ox, oy, sx, sy);

  // box 用映射后的角点重建，保证下游画框/录像与原图对齐。
  // 用 min/max 而不是直接 Rect(tl, br)：YOLO 的关键点顺序异常时 tl/br 可能颠倒，
  // Rect(Point,Point) 会构造出负宽高，下游 cv::rectangle 会画到画面外。
  const auto tl = map(cv::Point2f(armor.box.tl()));
  const auto br = map(cv::Point2f(armor.box.br()));
  const int x1 = static_cast<int>(std::lround(std::min(tl.x, br.x)));
  const int y1 = static_cast<int>(std::lround(std::min(tl.y, br.y)));
  const int x2 = static_cast<int>(std::lround(std::max(tl.x, br.x)));
  const int y2 = static_cast<int>(std::lround(std::max(tl.y, br.y)));
  armor.box = cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));

  // 归一化坐标必须按「该相机原始图像」重算：大图上的归一化值对本路没有意义。
  armor.center_norm = {
    std::clamp(armor.center.x / source_size.width, 0.0f, 1.0f),
    std::clamp(armor.center.y / source_size.height, 0.0f, 1.0f)};

  // ratio / side_ratio / rectangular_error 是无量纲比值与夹角，缩放不变，不重算。
  return true;
}

void StitchedOmni::dispatch(
  const std::list<auto_aim::Armor> & armors, const std::vector<cv::Mat> & frames,
  const std::vector<std::chrono::steady_clock::time_point> & timestamps,
  std::vector<DetectionResult> & results)
{
  const std::size_t n = configs_.size();

  // 敌方颜色切换（主线程异步下发），在每帧开头同步到各路的 Decider。
  if (color_dirty_.exchange(false)) {
    std::lock_guard<std::mutex> lock(color_mutex_);
    for (auto & decider : deciders_) decider->set_enemy_color(pending_color_);
  }

  std::vector<std::list<auto_aim::Armor>> buckets(n);
  int dropped_blank = 0;  // 落在黑格（空相机通道）里的检测
  int dropped_seam = 0;   // 跨越拼接分界线的检测

  for (const auto & armor : armors) {
    // 分区反推：只看检测框中心落在哪个 cell（跨边界的框归属不可靠，这里以中心为准）。
    const cv::Point2f center = armor.center;
    if (center.x < 0 || center.y < 0) continue;

    const int col = std::min<int>(
      static_cast<int>(center.x) / layout_.cell.width, layout_.cols - 1);
    const int row = std::min<int>(
      static_cast<int>(center.y) / layout_.cell.height, layout_.rows - 1);
    const int index = layout_.index_of(col, row);
    if (index < 0 || static_cast<std::size_t>(index) >= n) continue;

    // 空相机通道：这一格本帧是纯黑填充，任何检测都是误检，直接丢弃。
    // 不能用 source_sizes_ 判断——那是「这一路曾经出过图」，不等于「本帧有画面」。
    if (frames[index].empty()) {
      dropped_blank++;
      continue;
    }

    // 跨拼接分界线：目标在拼接时被硬裁剪，几何量不可信，整条丢弃，不参与后续处理。
    if (crosses_seam(armor, layout_.cell_rect(index), layout_.boundary_margin)) {
      dropped_seam++;
      continue;
    }

    // 映射回该相机原始图像坐标；分辨率未知（该路从未出图）时丢弃。
    const cv::Size source_size =
      source_sizes_[index].width > 0 ? source_sizes_[index] : frames[index].size();

    auto mapped = armor;
    if (!map_to_camera(mapped, index, source_size)) continue;
    buckets[index].push_back(std::move(mapped));
  }

  if (dropped_blank > 0 || dropped_seam > 0) {
    tools::logger()->debug(
      "[StitchedOmni] 丢弃：跨分界线 {} 条、黑格误检 {} 条（margin={:.1f}px）", dropped_seam,
      dropped_blank, static_cast<double>(layout_.boundary_margin));
  }

  // 录像：与旧的全向路径一致，每路都把「原图 + 未做敌方颜色过滤的检测框」录一份。
  // 空相机通道没有原图可录；黑格误检与跨分界线的框已在上面剔除，不会出现在录像里。
  for (std::size_t i = 0; i < n; i++) {
    if (!configs_[i].recorder || frames[i].empty()) continue;

    cv::Mat display = frames[i].clone();
    for (const auto & armor : buckets[i]) {
      cv::rectangle(display, armor.box, cv::Scalar(0, 255, 0), 2);
      const std::string label = fmt::format(
        "{} {} {:.2f}", auto_aim::COLORS[armor.color], auto_aim::ARMOR_NAMES[armor.name],
        armor.confidence);
      cv::putText(
        display, label, cv::Point(armor.box.x, armor.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
        cv::Scalar(0, 255, 0), 1);
      for (const auto & point : armor.points) {
        cv::circle(display, point, 2, cv::Scalar(0, 0, 255), -1);
      }
    }
    configs_[i].recorder->record(display, Eigen::Quaterniond::Identity(), timestamps[i]);
  }

  // 每路独立完成：敌方过滤 -> 排序 -> PnP 解算。
  // 顺序与 omni_jobs 的推理线程完全一致，保证 delta_yaw/delta_pitch 描述的目标
  // 与 armors.front() 是同一块装甲板。
  for (std::size_t i = 0; i < n; i++) {
    if (buckets[i].empty()) continue;

    deciders_[i]->armor_filter(buckets[i]);
    buckets[i].sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
      return a.priority < b.priority;
    });
    if (buckets[i].empty()) continue;

    DetectionResult result;
    result.armors = std::move(buckets[i]);
    result.timestamp = timestamps[i];

    // image_size 必须是「该相机原始分辨率」：内参是按原始帧标定的，
    // 传大图或分区尺寸会让 FOV 近似内参与 PnP 一起偏掉。
    const cv::Size source_size =
      source_sizes_[i].width > 0 ? source_sizes_[i] : frames[i].size();
    const auto da = deciders_[i]->delta_angle(result.armors, configs_[i], source_size);
    result.delta_yaw = da[0] / 57.3;
    result.delta_pitch = da[1] / 57.3;

    tools::logger()->debug(
      "[StitchedOmni] omni{} -> 分区 ({}, {})，{} 块装甲板，da_yaw={:.2f}° da_pitch={:.2f}°",
      i + 1, layout_.col_of(static_cast<int>(i)), layout_.row_of(static_cast<int>(i)),
      result.armors.size(), da[0], da[1]);

    results.push_back(std::move(result));
  }
}

}  // namespace omniperception
