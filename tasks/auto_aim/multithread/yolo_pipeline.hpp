#ifndef AUTO_AIM__MULTITHREAD__YOLO_PIPELINE_HPP
#define AUTO_AIM__MULTITHREAD__YOLO_PIPELINE_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <list>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/armor.hpp"

namespace auto_aim
{
namespace multithread
{

// 单路相机的一帧推理结果
struct CameraResult
{
  int camera_id = -1;  // -1 = 主相机；0..N-1 = 全向相机序号
  std::list<Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
  cv::Mat img;               // 仅主相机保留原图（画框/录像），全向为空
  double delta_yaw = 0.0;    // 仅全向：目标相对大yaw 的 yaw（rad）
  double delta_pitch = 0.0;  // 仅全向：目标相对大yaw 的 pitch（rad）
  double inference_ms = 0.0;

  bool is_main() const { return camera_id < 0; }
};

// 一路相机的任务描述：相机、YOLO 实例与线程一一绑定，数据不跨相机复用。
struct CameraJob
{
  io::Camera * camera = nullptr;
  int camera_id = -1;        // -1 = 主相机
  bool keep_image = false;   // 结果里是否带上原图（只有主相机需要）
  // 推理之后、入队之前的相机相关处理（全向相机：过滤 / 优先级 / 角度解算 / 录像）。
  // 没有该相机特有的处理时留空。
  std::function<
    void(
      const cv::Mat & img,
      std::chrono::steady_clock::time_point timestamp,
      std::list<Armor> & armors,
      CameraResult & result
      )>
    postprocess;

  bool is_main() const { return camera_id < 0; }
};

// 多相机 YOLO 推理池：
//   * 每个 CameraJob 一个独立 YOLO 实例 + 一个独立线程，相机数据只喂给自己的实例，
//     彻底消除“多生产者抢一个消费者（OpenVINO InferRequest）”的串行瓶颈；
//   * 线程启动时先绑定 CPU 核、再构造 YOLO：OpenVINO 在 compile_model 期间创建的
//     CPU 线程池会继承创建者的亲和性掩码，推理线程因此也固定在同一颗核上；
//   * 结果进入同一个带优先级的队列：主相机结果永远最先被消费（且只保留最新一帧，
//     主循环来不及处理时不追旧帧），全向结果按到达顺序排队。
class YoloPipeline
{
public:
  // config_path 里的可选键 yolo_cpu_cores: [core_main, core_omni1, ...]
  //   缺省时自动分配（0 号核留给系统/主循环，推理从 1 号核依次往后）；
  //   某项为负数表示该路不绑核。
  YoloPipeline(const std::string & config_path, std::vector<CameraJob> jobs, bool debug = false);
  ~YoloPipeline();

  YoloPipeline(const YoloPipeline &) = delete;
  YoloPipeline & operator=(const YoloPipeline &) = delete;

  // 比赛开始前关闭推理：相机仍持续读数、主相机结果照常入队（armors 为空），
  // 全向不再产生新结果，避免空转占满 CPU。
  void set_inference_enabled(bool enabled);

  // 阻塞等待一帧主相机结果；推理池已停止时返回 false。
  bool wait_main(CameraResult & result);

  // 非阻塞取走当前积压的所有全向结果（按到达顺序）。
  void drain_omni(std::vector<CameraResult> & results);

private:
  void run(std::size_t index);
  void mark_ready(std::exception_ptr error);
  void push(CameraResult && result);

  std::string config_path_;
  bool debug_;
  std::vector<CameraJob> jobs_;
  std::vector<int> cores_;

  std::vector<std::thread> threads_;
  std::atomic<bool> quit_{false};
  std::atomic<bool> inference_enabled_{true};

  // 初始化同步：所有线程完成“绑核 + YOLO 编译”后构造函数才返回
  std::mutex ready_mutex_;
  std::condition_variable ready_cv_;
  std::size_t ready_count_ = 0;
  std::exception_ptr init_error_;

  // 带优先级的结果队列：主相机结果单独占一个“最新帧”槽位，永远优先于全向
  std::mutex result_mutex_;
  std::condition_variable result_cv_;
  CameraResult main_result_;
  bool has_main_ = false;
  std::deque<CameraResult> omni_results_;
  std::size_t max_omni_results_ = 16;
};

}  // namespace multithread
}  // namespace auto_aim

#endif  // AUTO_AIM__MULTITHREAD__YOLO_PIPELINE_HPP
