#ifndef IO__SIM__RECORDER_HPP
#define IO__SIM__RECORDER_HPP

// 记录与回放：
//   - 串口原始字节 + 时间戳（serial.bin）
//   - 相机帧 + 时间戳 + 帧信息（camera.tsv + camera/*.png）
// 记录使用统一会话时钟的相对时间戳，回放时映射回当前 steady_clock，保证时间轴一致。

#include <opencv2/opencv.hpp>

#include <atomic>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "io/sim/common.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io::sim
{

struct SerialRecord
{
  uint64_t t_ns = 0;  // 相对会话起点
  uint8_t dir = 0;    // 0 = rx(下位机→视觉)，1 = tx(视觉→下位机)
  std::vector<uint8_t> data;
};

// ============================ 串口记录 ============================
class SerialRecorder
{
public:
  explicit SerialRecorder(const std::string & session_dir);
  ~SerialRecorder();

  void record_rx(const uint8_t * data, size_t size);
  void record_tx(const uint8_t * data, size_t size);
  void close();

  const std::string & path() const { return path_; }

private:
  void write_record(uint8_t dir, const uint8_t * data, size_t size);

  std::string path_;
  std::ofstream file_;
  std::mutex mutex_;
  uint32_t count_ = 0;
  bool closed_ = false;
};

// ============================ 串口回放读取 ============================
class SerialReplayReader
{
public:
  explicit SerialReplayReader(const std::string & path);

  bool load(std::string & error);
  bool ok() const { return ok_; }
  const std::vector<SerialRecord> & records() const { return records_; }
  size_t rx_bytes() const { return rx_bytes_; }

private:
  std::string path_;
  bool ok_ = false;
  std::vector<SerialRecord> records_;
  size_t rx_bytes_ = 0;
};

// ============================ 相机记录 ============================
class CameraRecorder
{
public:
  CameraRecorder(std::string session_dir, bool save_images);
  ~CameraRecorder();

  void record(const cv::Mat & img, TimePoint ts, uint64_t frame_id, int format);
  void close();

  const std::string & dir() const { return session_dir_; }

private:
  struct Item
  {
    cv::Mat img;
    uint64_t t_ns = 0;
    uint64_t frame_id = 0;
    int format = 0;
  };

  void worker();
  void write_item(const Item & item);

  std::string session_dir_;
  std::string images_dir_;
  bool save_images_;
  std::ofstream tsv_;
  tools::ThreadSafeQueue<Item> queue_{512};
  std::thread worker_;
  std::atomic<bool> quit_{false};
  bool closed_ = false;
};

// ============================ 相机回放读取 ============================
struct CameraFrameRecord
{
  uint64_t frame_id = 0;
  uint64_t t_ns = 0;
  int width = 0;
  int height = 0;
  int format = 0;
  std::string file;
};

class CameraReplayReader
{
public:
  explicit CameraReplayReader(const std::string & session_dir);

  bool load(std::string & error);
  bool ok() const { return ok_; }
  const std::vector<CameraFrameRecord> & records() const { return records_; }
  cv::Mat read_image(size_t index);
  uint64_t duration_ns() const;

private:
  std::string session_dir_;
  bool ok_ = false;
  bool save_images_ = true;
  std::vector<CameraFrameRecord> records_;
};

}  // namespace io::sim

#endif  // IO__SIM__RECORDER_HPP
