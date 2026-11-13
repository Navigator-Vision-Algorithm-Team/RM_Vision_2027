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
#include "tools/cpu_core_manager.hpp"

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

// wait_main() 的返回值：调用方需要在 Timeout 分支里继续检查自己的退出条件
// （Ctrl+C / rclcpp shutdown / 相机掉线），不能无限期阻塞在队列上。
enum class WaitStatus
{
  Ok,       // 取到一帧主相机结果
  Timeout,  // 超时，暂无主相机结果（相机可能没出图）
  Stopped   // 推理池已停止，不会再产生结果
};

// 一路相机的任务描述：相机、YOLO 实例与线程一一绑定，数据不跨相机复用。
struct CameraJob
{
  io::Camera * camera = nullptr;
  int camera_id = -1;        // -1 = 主相机
  bool keep_image = false;   // 结果里是否带上原图（只有主相机需要）
  // 是否把当前线程绑定到独占 CPU 核。全向相机默认 true；主相机可关闭，
  // 让 OpenVINO 按自身策略使用更多核。
  bool pin_cpu = true;
  // 绑核时优先申请的物理核类型（仅 pin_cpu=true 时生效）：
  //   Unknown      -> 默认顺序，性能核优先
  //   Efficiency   -> 只用能效核，把性能核让给不绑核的主相机
  //   Performance  -> 只用性能核
  // 该类型的核不够时回退到其它核，并在启动日志里逐个告警，不会静默降级。
  tools::CpuCoreType core_preference = tools::CpuCoreType::Unknown;
  // CPU 推理是否限制为单线程。全向相机默认 true（每路独占一核），
  // 主相机可关闭，允许 OpenVINO 自主并行。
  bool single_thread_cpu = true;
  // 推理门控：返回 true 才执行本帧 YOLO，返回 false 直接跳过推理
  // （全向相机的常驻颜色层就挂在这里）。为空表示每帧都推理，即旧行为。
  //
  // 跳帧只是不调用 detect()：YOLO 实例与模型仍常驻内存，没有加载/卸载开销。
  // 门控失败时 result.armors 为空，postprocess 照常调用（录像 / 颜色同步不中断）。
  std::function<bool(const cv::Mat & img)> inference_gate;
  // 推理之后、入队之前的相机相关处理（全向相机：过滤 / 优先级 / 角度解算 / 录像）。
  // img 是 const：处理函数只能读取，需要画框请自行 clone。
  // 原因：伪相机（camera_name=fake）走的是零拷贝缓冲池，且 render_overlay() 每帧
  // 只重画左上角一小块做"复原"，直接在 img 上画框会把框永久留在池缓冲里，
  // 后续帧复用同一块缓冲时出现残影叠加。生产路径（hikrobot/mindvision 每帧
  // 独立缓冲）虽然可以直接画，但契约必须对所有数据源一致。
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
//   * 自动分配时按物理核心分配、性能核优先，每个物理核心只选一个逻辑 CPU；
//     仅对 pin_cpu=true 的任务绑核，且线程启动时先绑定、再构造 YOLO，使 OpenVINO
//     在线程初始化时继承该亲和性；
//   * 结果进入同一个带优先级的队列：主相机结果永远最先被消费（且只保留最新一帧，
//     主循环来不及处理时不追旧帧），全向结果按到达顺序排队。
class YoloPipeline
{
public:
  // 按 jobs 自动分配 CPU：仅 pin_cpu=true 的任务占用物理核心，优先性能核。
  // 不需要单独配置 CPU 数量或逻辑 CPU ID。
  YoloPipeline(const std::string & config_path, std::vector<CameraJob> jobs, bool debug = false);
  ~YoloPipeline();

  YoloPipeline(const YoloPipeline &) = delete;
  YoloPipeline & operator=(const YoloPipeline &) = delete;

  // 比赛开始前关闭推理：相机仍持续读数、主相机结果照常入队（armors 为空），
  // 全向不再产生新结果，避免空转占满 CPU。
  void set_inference_enabled(bool enabled);

  // 等待一帧主相机结果，最多等 timeout。
  // 必须带超时：相机掉线时线程可能永远等不到新帧，若这里无限阻塞，
  // 主循环就感知不到 Ctrl+C / rclcpp shutdown，也无法在析构时退出。
  WaitStatus wait_main(
    CameraResult & result, std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

  // 非阻塞取走当前积压的所有全向结果（按到达顺序）。
  void drain_omni(std::vector<CameraResult> & results);

private:
  void run(std::size_t index);
  void mark_ready(std::exception_ptr error);
  void push(CameraResult && result);

  std::string config_path_;
  bool debug_;
  std::vector<CameraJob> jobs_;
  tools::CpuCoreManager cpu_manager_;
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
