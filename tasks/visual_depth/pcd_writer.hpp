#ifndef VISUAL_DEPTH__PCD_WRITER_HPP
#define VISUAL_DEPTH__PCD_WRITER_HPP

// =============================================================================
//  pcd_writer.hpp —— 建图点落盘（方案 §5.4 出口 C / L4）
//
//  出口 C 只做离线文件，不做在线累积节点（方案 §1.2 已定）：
//      视觉侧建图运行 -> 累积 + 质量筛选 -> output/vision_map.pcd
//      -> 人工拷到 pb2025_nav_bringup/pcd/reality/
//      -> 离线脚本与激光 PCD 融合 -> small_gicp_relocalization 加载
//
//  字段布局刻意与 C4 话题一致（PointXYZINormal，8 个 FLOAT32）：
//      x y z intensity normal_x normal_y normal_z curvature
//    normal_x = 1/重投影误差 RMS，normal_y = 观测次数，normal_z = 视差角（度），
//    curvature = 一致性得分。这样离线脚本对文件与话题可以用同一套阈值。
//
//  写文件用 DATA binary：2M 点约 64 MB，ASCII 会膨胀 3 倍且解析更慢。
//  ⚠ 只在 flush() / 析构时落盘，进程被 SIGKILL 时最后一段会丢（Ctrl+C 正常退出不丢）。
// =============================================================================

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <vector>

namespace visual_depth
{

struct PcdPoint
{
  Eigen::Vector3f p = Eigen::Vector3f::Zero();
  float quality = 0.f;       // normal_x：1 / 重投影 RMS
  int observations = 0;      // normal_y
  float parallax_deg = 0.f;  // normal_z
  float consistency = 1.f;   // curvature
};

class PcdWriter
{
public:
  PcdWriter(bool enable, std::string path, size_t max_points = 2000000);
  ~PcdWriter();

  PcdWriter(const PcdWriter &) = delete;
  PcdWriter & operator=(const PcdWriter &) = delete;

  bool enabled() const { return enable_; }
  const std::string & path() const { return path_; }

  // 只入内存；超过 max_points 后新点直接丢弃（保护内存，打一次 warn 由调用方负责）
  void add(const std::vector<PcdPoint> & points);
  size_t size() const { return points_.size(); }
  bool full() const { return points_.size() >= max_points_; }

  // 写回 path_；成功返回 true（目录不存在时自动创建）
  bool flush();

private:
  bool enable_ = false;
  std::string path_;
  size_t max_points_ = 2000000;
  std::vector<PcdPoint> points_;
  bool dumped_ = false;
};

}  // namespace visual_depth

#endif  // VISUAL_DEPTH__PCD_WRITER_HPP
