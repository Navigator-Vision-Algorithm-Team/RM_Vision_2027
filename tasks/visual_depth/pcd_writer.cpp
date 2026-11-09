#include "pcd_writer.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <utility>

#include "tools/logger.hpp"

namespace visual_depth
{

PcdWriter::PcdWriter(bool enable, std::string path, size_t max_points)
: enable_(enable), path_(std::move(path)), max_points_(max_points > 0 ? max_points : 1)
{
  if (enable_) tools::logger()->info("[PcdWriter] 建图点落盘 -> {}", path_);
}

PcdWriter::~PcdWriter()
{
  if (enable_ && !dumped_) flush();
}

void PcdWriter::add(const std::vector<PcdPoint> & points)
{
  if (!enable_ || points.empty()) return;
  const size_t room = max_points_ - points_.size();
  const size_t n = points.size() < room ? points.size() : room;
  points_.insert(points_.end(), points.begin(), points.begin() + static_cast<long>(n));
}

bool PcdWriter::flush()
{
  if (!enable_) return false;

  try {
    const std::filesystem::path out(path_);
    if (out.has_parent_path()) std::filesystem::create_directories(out.parent_path());

    std::ofstream ofs(path_, std::ios::binary | std::ios::trunc);
    if (!ofs) {
      tools::logger()->warn("[PcdWriter] 无法写入 {}", path_);
      return false;
    }

    // PCD v0.7 header：8 个 FLOAT32 字段，与 PointXYZINormal 同口径
    ofs << "# .PCD v0.7 - Point Cloud Data file format\n"
        << "VERSION 0.7\n"
        << "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
        << "SIZE 4 4 4 4 4 4 4 4\n"
        << "TYPE F F F F F F F F\n"
        << "COUNT 1 1 1 1 1 1 1 1\n"
        << "WIDTH " << points_.size() << "\n"
        << "HEIGHT 1\n"
        << "VIEWPOINT 0 0 0 1 0 0 0\n"
        << "POINTS " << points_.size() << "\n"
        << "DATA binary\n";

    // 每点 8 × FLOAT32 = 32 B（文件里不需要 PCL 结构体的 16 B 对齐 padding）
    std::vector<float> buffer;
    buffer.reserve(points_.size() * 8);
    for (const auto & p : points_) {
      buffer.push_back(p.p.x());
      buffer.push_back(p.p.y());
      buffer.push_back(p.p.z());
      buffer.push_back(1.0f);  // intensity 占位，兼容下游
      buffer.push_back(p.quality);
      buffer.push_back(static_cast<float>(p.observations));
      buffer.push_back(p.parallax_deg);
      buffer.push_back(p.consistency);
    }
    if (!buffer.empty()) {
      ofs.write(
        reinterpret_cast<const char *>(buffer.data()),
        static_cast<std::streamsize>(buffer.size() * sizeof(float)));
    }
    ofs.close();

    dumped_ = true;
    tools::logger()->info("[PcdWriter] 已写出 {} 点 -> {}", points_.size(), path_);
    return true;
  } catch (const std::exception & e) {
    tools::logger()->warn("[PcdWriter] 写 {} 失败：{}", path_, e.what());
    return false;
  }
}

}  // namespace visual_depth
