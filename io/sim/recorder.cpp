#include "io/sim/recorder.hpp"

#include <filesystem>
#include <fmt/format.h>
#include <sstream>

#include "tools/logger.hpp"

namespace fs = std::filesystem;

namespace io::sim
{

namespace
{
constexpr char k_serial_magic[8] = {'R', 'M', 'S', 'E', 'R', 'I', 'A', 'L'};

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

// ============================ SerialRecorder ============================
SerialRecorder::SerialRecorder(const std::string & session_dir)
{
  ensure_dir(session_dir);
  path_ = session_dir + "/serial.bin";
  file_.open(path_, std::ios::binary | std::ios::trunc);
  if (!file_.is_open()) {
    tools::logger()->warn("[REC] Cannot open serial record file: {}", path_);
    return;
  }

  const uint64_t start_unix_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count());
  uint32_t count = 0;
  file_.write(k_serial_magic, sizeof(k_serial_magic));
  file_.write(reinterpret_cast<const char *>(&start_unix_ns), sizeof(start_unix_ns));
  file_.write(reinterpret_cast<const char *>(&count), sizeof(count));
  file_.flush();

  tools::logger()->info("[REC] Serial recording to {}", path_);
}

SerialRecorder::~SerialRecorder() { close(); }

void SerialRecorder::record_rx(const uint8_t * data, size_t size) { write_record(0, data, size); }

void SerialRecorder::record_tx(const uint8_t * data, size_t size) { write_record(1, data, size); }

void SerialRecorder::write_record(uint8_t dir, const uint8_t * data, size_t size)
{
  if (!file_.is_open() || !data || size == 0) return;
  std::lock_guard<std::mutex> lock(mutex_);

  const uint64_t t_ns = SessionClock::now_rel_ns();
  const uint32_t len = static_cast<uint32_t>(size);
  file_.write(reinterpret_cast<const char *>(&t_ns), sizeof(t_ns));
  file_.write(reinterpret_cast<const char *>(&dir), sizeof(dir));
  file_.write(reinterpret_cast<const char *>(&len), sizeof(len));
  file_.write(reinterpret_cast<const char *>(data), size);
  count_++;

  // 周期性落盘，避免崩溃时丢数据
  if (count_ % 256 == 0) file_.flush();
}

void SerialRecorder::close()
{
  if (closed_) return;
  closed_ = true;
  if (!file_.is_open()) return;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 回填记录条数
    file_.seekp(static_cast<std::streamoff>(sizeof(k_serial_magic) + sizeof(uint64_t)),
                std::ios::beg);
    file_.write(reinterpret_cast<const char *>(&count_), sizeof(count_));
    file_.flush();
    file_.close();
  }
  tools::logger()->info("[REC] Serial recording closed ({} records): {}", count_, path_);
}

// ============================ SerialReplayReader ============================
SerialReplayReader::SerialReplayReader(const std::string & path) : path_(path) {}

bool SerialReplayReader::load(std::string & error)
{
  std::ifstream in(path_, std::ios::binary);
  if (!in.is_open()) {
    error = "cannot open " + path_;
    return ok_ = false;
  }

  char magic[8] = {};
  uint64_t start_unix_ns = 0;
  uint32_t count = 0;
  in.read(magic, sizeof(magic));
  in.read(reinterpret_cast<char *>(&start_unix_ns), sizeof(start_unix_ns));
  in.read(reinterpret_cast<char *>(&count), sizeof(count));

  for (size_t i = 0; i < sizeof(k_serial_magic); ++i) {
    if (magic[i] != k_serial_magic[i]) {
      error = "bad magic in " + path_;
      return ok_ = false;
    }
  }

  records_.clear();
  rx_bytes_ = 0;
  while (in.good()) {
    SerialRecord rec;
    in.read(reinterpret_cast<char *>(&rec.t_ns), sizeof(rec.t_ns));
    if (!in.good()) break;
    in.read(reinterpret_cast<char *>(&rec.dir), sizeof(rec.dir));
    uint32_t len = 0;
    in.read(reinterpret_cast<char *>(&len), sizeof(len));
    if (!in.good() || len == 0 || len > (1u << 20)) break;
    rec.data.resize(len);
    in.read(reinterpret_cast<char *>(rec.data.data()), len);
    if (!in.good()) break;
    if (rec.dir == 0) rx_bytes_ += len;
    records_.push_back(std::move(rec));
  }

  ok_ = true;
  return true;
}

// ============================ CameraRecorder ============================
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

  tsv_ << item.frame_id << '\t' << item.t_ns << '\t' << item.img.cols << '\t' << item.img.rows << '\t'
       << item.format << '\t' << rel << '\n';
}

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

// ============================ CameraReplayReader ============================
CameraReplayReader::CameraReplayReader(const std::string & session_dir) : session_dir_(session_dir) {}

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
