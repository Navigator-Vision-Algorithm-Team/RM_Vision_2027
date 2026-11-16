/*
 * 相机录像与回放的实现：camera.tsv 记录元信息，camera/*.png 存图。
 *
 * 数据流：CameraRecorder::record() → 深拷贝入队 → worker() 逐帧写 png 并追加 tsv 行；
 *         CameraReplayReader::load() 解析 tsv，read_image() 按行内文件名读图。
 *
 * 注意：契约写在 io/sim/recorder.hpp，本文件只写实现要点。
 *       串口字节记录已迁至 info_relay/link/transport/serial_recorder.cpp。
 */
#include "io/sim/recorder.hpp"

#include <fmt/format.h>

#include <chrono>
#include <filesystem>
#include <sstream>

#include "tools/logger.hpp"

namespace fs = std::filesystem;

namespace io::sim
{
namespace
{
// 功能：创建目录，失败只告警
// 输入：
//   dir  目录路径
// 输出：原样返回 dir
// 备注：只创建不删除；权限不足时后续写文件会失败并告警
std::string ensure_dir(const std::string & dir)
{
  try {
    fs::create_directories(dir);
  } catch (const std::exception & e) {
    tools::logger()->warn("[REC] Failed to create dir {}: {}", dir, e.what());
  }
  return dir;
}
}  // namespace

// 实现要点：先建目录与 tsv 再启动写盘线程，避免线程起来后 tsv_ 还是未打开状态。
CameraRecorder::CameraRecorder(std::string session_dir, bool save_images)
: session_dir_(std::move(session_dir)), save_images_(save_images)
{
  ensure_dir(session_dir_);
  if (save_images_) {
    images_dir_ = session_dir_ + "/camera";
    ensure_dir(images_dir_);
  }

  tsv_.open(session_dir_ + "/camera.tsv", std::ios::trunc);
  if (!tsv_.is_open()) {
    tools::logger()->warn("[REC] Cannot open camera.tsv in {}", session_dir_);
  } else {
    tsv_ << "# frame_id\ttimestamp_ns\twidth\theight\tformat\tfile\n";
  }

  worker_ = std::thread([this] { worker(); });
  tools::logger()->info("[CAM-REC] Camera recording to {} (images={})", session_dir_, save_images_);
}

CameraRecorder::~CameraRecorder() { close(); }

// 实现要点：这里必须深拷贝——上游相机可能是零拷贝缓冲池，浅拷贝会让写盘时读到已被覆盖的帧。
void CameraRecorder::record(const cv::Mat & img, TimePoint ts, uint64_t frame_id, int format)
{
  if (closed_ || img.empty()) return;
  Item item;
  item.img = img;  // 深拷贝，避免与上游复用缓冲
  item.t_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(ts - SessionClock::start_time()).count());
  item.frame_id = frame_id;
  item.format = format;
  queue_.push(item);
}

// 实现要点：写图失败时把文件名记为 "none"，让回放侧生成占位图而不是崩在 imread。
void CameraRecorder::write_item(const Item & item)
{
  if (!tsv_.is_open()) return;

  std::string rel = "none";
  if (save_images_) {
    rel = fmt::format("camera/{:06d}.png", item.frame_id);
    const std::string path = session_dir_ + "/" + rel;
    if (!cv::imwrite(path, item.img)) {
      tools::logger()->warn("[CAM-REC] Failed to write {}", path);
      rel = "none";
    }
  }

  tsv_ << item.frame_id << '\t' << item.t_ns << '\t' << item.img.cols << '\t' << item.img.rows
       << '\t' << item.format << '\t' << rel << '\n';
}

// 实现要点：用 100ms 超时轮询而不是阻塞 pop，析构置 quit_ 后最多等 100ms 就能退出；
// 退出前把队列排空，避免丢最后一帧。
void CameraRecorder::worker()
{
  while (!quit_) {
    Item item;
    if (!queue_.try_pop_for(item, std::chrono::milliseconds(100))) continue;
    write_item(item);
  }
  // 排空
  Item item;
  while (queue_.try_pop(item)) write_item(item);
  if (tsv_.is_open()) tsv_.flush();
}

void CameraRecorder::close()
{
  if (closed_) return;
  closed_ = true;
  quit_ = true;
  if (worker_.joinable()) worker_.join();
  if (tsv_.is_open()) {
    tsv_.flush();
    tsv_.close();
  }
  tools::logger()->info("[CAM-REC] Camera recording closed: {}", session_dir_);
}

CameraReplayReader::CameraReplayReader(const std::string & session_dir)
: session_dir_(session_dir)
{
}

// 实现要点：tsv 是制表符分隔，最后一段是相对路径；解析失败的行直接跳过而不是整体报错，
// 这样半截损坏的会话仍能回放可用的部分。
bool CameraReplayReader::load(std::string & error)
{
  const std::string path = session_dir_ + "/camera.tsv";
  std::ifstream in(path);
  if (!in.is_open()) {
    error = "cannot open " + path;
    return ok_ = false;
  }

  std::string line;
  records_.clear();
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    CameraFrameRecord rec;
    std::string file;
    if (!(ss >> rec.frame_id >> rec.t_ns >> rec.width >> rec.height >> rec.format >> file)) continue;
    rec.file = file;
    records_.push_back(std::move(rec));
  }

  ok_ = !records_.empty();
  if (!ok_) error = "no frames in " + path;
  return ok_;
}

cv::Mat CameraReplayReader::read_image(size_t index)
{
  if (index >= records_.size()) return {};
  const auto & rec = records_[index];
  if (rec.file == "none" || rec.file.empty()) {
    // 仅记录元信息（未存图）时返回合成占位图，保持尺寸/格式一致
    return cv::Mat(rec.height, rec.width, CV_8UC3, cv::Scalar(0, 0, 0));
  }
  cv::Mat img = cv::imread(session_dir_ + "/" + rec.file, cv::IMREAD_COLOR);
  return img;
}

uint64_t CameraReplayReader::duration_ns() const
{
  if (records_.empty()) return 0;
  return records_.back().t_ns - records_.front().t_ns;
}

}  // namespace io::sim
