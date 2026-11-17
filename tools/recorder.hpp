#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

/*
 * 录制器：把相机帧与当帧 IMU 四元数异步落盘，视频走硬件 H.264 编码。
 *
 * 数据流：调用方 record(img, q, t) → 单槽队列 → 保存线程 → FFmpeg 编码
 *         → records/<file_name>/<启动时刻>.mkv（H.264 + Matroska 容器）
 *         + records/<file_name>/<启动时刻>.txt（每行 "t w x y z"）。
 *
 * 线程：record() 由调用方线程调用（主相机、各全向相机各持有一个实例）；编码与写盘在
 *       保存线程内完成，主循环不会被编码或磁盘 IO 阻塞。全进程同一时刻只允许一个保存
 *       线程进入编码（内部全局锁），即多相机串行编码，避免多路同时抢 GPU 的编码单元。
 *
 * 注意：1) 队列容量为 1，编码跟不上时新帧被直接丢弃；视频时间戳取真实采集时刻，
 *          因此码流是可变帧率，丢帧只会让画面跳，不会让回放时长变短；
 *       2) 同一实例非线程安全，只允许一个线程调用 record()；
 *       3) 构造即创建 records/<file_name>/ 目录（相对进程当前工作目录），失败抛异常；
 *       4) 编码器优先 h264_vaapi，不可用时自动回退 libx264；两者都失败则本次不录像，
 *          只打一条 error 日志，不影响主链路。
 */

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "tools/frame_timer.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/video_overlay.hpp"

namespace tools
{
class Recorder
{
public:
  // 功能：确定本次录制的目录、文件名与录像参数，并创建目录
  // 输入：
  //   fps        录制帧率，Hz（帧/秒），必须 > 0；作为限帧阈值与码流标称帧率，
  //              应与调用方的实际出帧率一致（如全向相机 15、主相机限到 30）
  //   file_name  设备名兼子目录名，无扩展名；录制文件落在 records/<file_name>/
  //   quality    编码质量，1 ~ 51，越小越清晰体积越大；对应 VAAPI 的 global_quality
  //              与 libx264 的 crf，录像复盘建议 20 ~ 28
  // 输出：无
  // 备注：只建目录、不打开文件（编码器需要首帧宽高，延迟到 init()）；
  //       目录创建失败抛 std::filesystem::filesystem_error，不做降级
  explicit Recorder(double fps = 30, const std::string & file_name = "main", int quality = 25);

  // 功能：停止保存线程、冲刷编码器缓冲并关闭文件
  // 输入：无
  // 输出：无
  // 备注：会阻塞到保存线程写完当前帧并完成 flush 为止（通常几十毫秒）；
  //       队列中尚未消费的帧被丢弃；进程被强杀时文件仍可播（Matroska 无需尾部索引）
  ~Recorder();

  // 功能：提交一帧待录制数据，按 fps 限帧后放入队列
  // 输入：
  //   img        当前帧图像，BGR（CV_8UC3）或灰度（CV_8UC1），允许为空（空图直接忽略）；
  //              与调用方共享像素缓冲，不 clone
  //   q          该帧的 IMU 四元数（IMU 体坐标系 -> 世界系），只写入 txt
  //   timestamp  该帧采集时刻，steady_clock 时间点；用于限帧与视频/文本时间轴
  //   overlay    要画到画面上的覆盖层，按值随帧入队；默认空表示不画
  // 输出：无
  // 备注：首次调用会初始化编码器并启动保存线程；限帧后仍会覆盖层绘制，画完立即编码；
  //       img 是浅拷贝入队，**返回后调用方不得再修改 img**；队列满时本帧被丢弃，不阻塞；
  //       非线程安全，同一实例只允许单线程调用
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp,
    const VideoOverlay & overlay = VideoOverlay{});

private:
  // 一帧待写盘数据
  struct FrameData
  {
    cv::Mat img;                                      // BGR 图像；浅拷贝，与调用方共享像素缓冲
    Eigen::Quaterniond q;                             // IMU 体坐标系 -> 世界系的四元数
    std::chrono::steady_clock::time_point timestamp;  // 该帧采集时刻
    VideoOverlay overlay;                             // 随帧传递的覆盖层，按值拷贝（定长 POD）
  };

  // 编码器 + 封装器（VAAPI / libx264 + Matroska），定义在 recorder.cpp，外部不可见
  struct Impl;
  std::unique_ptr<Impl> impl_;  // 为 nullptr 表示编码器初始化失败，本实例不再录像

  bool init_;                        // 编码器是否已尝试初始化；只尝试一次，访问无锁
  std::atomic<bool> stop_thread_;    // 保存线程退出标志；仅析构置 true
  double fps_;                       // 录制帧率，Hz；来自构造参数 fps
  int quality_;                      // 编码质量；来自构造参数 quality
  std::string camera_name_;          // 设备名兼子目录名；来自构造参数 file_name
  std::string text_path_;            // 四元数 txt 的完整路径，构造期确定，运行期不变
  std::string video_path_;           // 视频 mkv 的完整路径，构造期确定，运行期不变
  std::ofstream text_writer_;        // 文本输出流；只在保存线程内使用
  std::chrono::steady_clock::time_point start_time_;  // 录制起点，s；时间轴相对它计算
  std::chrono::steady_clock::time_point last_time_;   // 上一帧被接受的时间戳，s；用于限帧
  tools::ThreadSafeQueue<FrameData> queue_;  // 帧队列，容量固定 1；满时丢弃新帧
  std::thread saving_thread_;                // 负责编码与保存的线程，由 init() 启动
  tools::FrameTimer frame_timer_;            // 保存线程内的绘制+编码耗时埋点

  // 功能：用首帧尺寸打开编码器与封装器，并启动保存线程
  // 输入：
  //   img  首帧图像，必须非空；只用于取宽高，不写入其内容
  // 输出：无
  // 备注：由 record() 内部惰性调用；编码器打开失败时只打 error 日志、不启动线程，
  //       且不会再重试（重试一次就要重开 VAAPI 设备，代价高于放弃录像）
  void init(const cv::Mat & img);

  // 功能：保存线程主体，循环取帧、编码、写入视频与四元数文本
  // 输入：无
  // 输出：无
  // 备注：阻塞在 queue_.pop() 上，靠析构推入的空帧唤醒后退出；每写一行文本立即 flush，
  //       保证进程异常退出时已录帧的时间戳不丢
  void save_to_file();
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP
