#ifndef IO__SIM__RECORDER_HPP
#define IO__SIM__RECORDER_HPP

/*
 * 相机录像与回放：把每帧图像与帧信息按「相对会话起点的时间戳」落盘。
 *
 * 数据流：io::Camera 读到的帧 → CameraRecorder（异步线程）
 *         → <session>/camera.tsv（元信息）+ <session>/camera/*.png（图像）；
 *         回放时 CameraReplayReader 读 tsv 并按下标取图。
 *
 * 线程：CameraRecorder 内部有一条后台写盘线程与容量 512 的队列，record() 不阻塞；
 *       CameraReplayReader 只在加载阶段单线程读取，之后只读。
 *
 * 注意：时间戳用 io::sim::SessionClock 的相对纳秒，与串口记录共用同一条时间轴，
 *       因此相机与串口回放可以按同一时间基准对齐。
 *       串口字节记录（serial.bin）已迁至 info_relay/link/transport/serial_recorder.hpp。
 */

#include <opencv2/opencv.hpp>

#include <atomic>
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "io/sim/common.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io::sim
{

// ============================ 相机记录 ============================
class CameraRecorder
{
public:
  // 功能：打开会话目录下的 camera.tsv 并启动后台写盘线程
  // 输入：
  //   session_dir   会话目录路径，不存在时自动创建
  //   save_images   true 时把每帧写入 camera/*.png；false 时只记元信息
  // 输出：无
  // 备注：tsv 打不开时只告警；后台线程在析构时 join
  CameraRecorder(std::string session_dir, bool save_images);

  // 功能：停止后台线程并关闭 tsv
  // 输入：无
  // 输出：无
  // 备注：会阻塞到队列排空为止；重复调用安全
  ~CameraRecorder();

  // 功能：提交一帧待记录数据
  // 输入：
  //   img       图像，空图直接忽略
  //   ts        该帧采集时刻（steady_clock）
  //   frame_id  帧序号，用于生成文件名
  //   format    图像类型（cv::Mat::type()）
  // 输出：无
  // 备注：内部深拷贝图像后入队，调用方可以立即复用原缓冲
  void record(const cv::Mat & img, TimePoint ts, uint64_t frame_id, int format);

  // 功能：停止后台线程并关闭文件
  // 输入：无
  // 输出：无
  // 备注：重复调用安全；析构会自动调用
  void close();

  // 返回值：会话目录路径
  const std::string & dir() const { return session_dir_; }

private:
  // 一帧待写盘数据
  struct Item
  {
    cv::Mat img;      // 已深拷贝的图像
    uint64_t t_ns = 0;  // 相对会话起点的时刻，ns
    uint64_t frame_id = 0;  // 帧序号
    int format = 0;         // cv::Mat::type()
  };

  // 功能：后台线程主体，循环取帧写盘，退出前排空队列
  // 输入：无
  // 输出：无
  // 备注：用 100ms 超时轮询 quit_，保证析构能及时退出
  void worker();

  // 功能：把一帧写入 png 与 tsv
  // 输入：
  //   item  待写盘数据
  // 输出：无
  // 备注：写图失败时 tsv 里的文件名记为 "none"，回放侧据此生成占位图
  void write_item(const Item & item);

  std::string session_dir_;   // 会话目录
  std::string images_dir_;    // camera/ 子目录，save_images_ 为 false 时为空
  bool save_images_ = true;   // 是否保存图像
  std::ofstream tsv_;         // camera.tsv 输出流
  tools::ThreadSafeQueue<Item> queue_{512};  // 待写盘队列，容量 512
  std::thread worker_;        // 写盘线程
  std::atomic<bool> quit_{false};  // 写盘线程退出标志
  bool closed_ = false;       // 是否已关闭，避免重复 join
};

// ============================ 相机回放读取 ============================
struct CameraFrameRecord
{
  uint64_t frame_id = 0;  // 帧序号
  uint64_t t_ns = 0;      // 相对会话起点的时刻，ns
  int width = 0;          // 图像宽，px
  int height = 0;         // 图像高，px
  int format = 0;         // cv::Mat::type()
  std::string file;       // 相对会话目录的图像路径；"none" 表示未存图
};

class CameraReplayReader
{
public:
  // 功能：构造读取器，只记录目录不读文件
  // 输入：
  //   session_dir  会话目录路径
  // 输出：无
  // 备注：实际读取发生在 load()
  explicit CameraReplayReader(const std::string & session_dir);

  // 功能：解析 camera.tsv 得到帧列表
  // 输入：
  //   error  失败时写入可读原因
  // 输出：true 表示解析到至少一帧
  // 备注：跳过以 # 开头的注释行与无法解析的行
  bool load(std::string & error);

  // 返回值：是否已成功加载
  bool ok() const { return ok_; }

  // 返回值：按时间顺序排列的帧记录
  const std::vector<CameraFrameRecord> & records() const { return records_; }

  // 功能：按索引读取一帧图像
  // 输入：
  //   index  帧索引，越界时返回空 Mat
  // 输出：BGR 图像；该帧未存图时返回同尺寸的全黑图
  // 备注：每次都读盘，不做缓存
  cv::Mat read_image(size_t index);

  // 返回值：首帧到末帧的时长，ns；无记录时为 0
  uint64_t duration_ns() const;

private:
  std::string session_dir_;  // 会话目录
  bool ok_ = false;          // 加载是否成功
  bool save_images_ = true;  // 会话是否包含图像
  std::vector<CameraFrameRecord> records_;  // 帧记录
};

}  // namespace io::sim

#endif  // IO__SIM__RECORDER_HPP
