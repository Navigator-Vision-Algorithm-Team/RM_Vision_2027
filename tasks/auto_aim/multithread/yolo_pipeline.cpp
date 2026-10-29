#include "yolo_pipeline.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/cpu_affinity.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace auto_aim
{
namespace multithread
{
namespace
{

// 相机没出图时的空转间隔（避免把一颗核跑满）
constexpr auto kIdlePoll = std::chrono::milliseconds(5);

// 每路推理的统计日志周期
constexpr double kStatsPeriodS = 5.0;

// 解析每路相机要绑定的核：
//   · 配置了 yolo_cpu_cores 就按列表循环取（负数 = 该路不绑核）；
//   · 没配置则自动分配：0 号核留给系统与主循环，推理线程从 1 号核往后排；
//   · 越界的核号告警后不绑核，避免在核数不同的机器上启动失败。
std::vector<int> resolve_cores(std::size_t count, const std::vector<int> & configured)
{
  const int hw = tools::hardware_concurrency();
  std::vector<int> cores(count, -1);

  if (configured.empty() && hw > 1 && count > static_cast<std::size_t>(hw - 1)) {
    tools::logger()->warn(
      "[YoloPipeline] 相机路数 {} 超过可分配的推理核数 {}，部分推理线程会共享核心", count, hw - 1);
  }

  for (std::size_t i = 0; i < count; i++) {
    int core = -1;
    if (!configured.empty()) {
      core = configured[i % configured.size()];
    } else if (hw > 1) {
      core = 1 + static_cast<int>(i % static_cast<std::size_t>(hw - 1));
    }

    if (core >= hw) {
      tools::logger()->warn(
        "[YoloPipeline] 配置的 CPU {} 超出本机核数 {}，该路不绑定", core, hw);
      core = -1;
    }
    cores[i] = core;
  }

  return cores;
}

}  // namespace

YoloPipeline::YoloPipeline(
  const std::string & config_path, std::vector<CameraJob> jobs, bool debug)
: config_path_(config_path), debug_(debug), jobs_(std::move(jobs))
{
  if (jobs_.empty()) throw std::runtime_error("[YoloPipeline] 至少需要一路相机任务");

  std::size_t main_count = 0;
  for (const auto & job : jobs_) {
    if (!job.camera) throw std::runtime_error("[YoloPipeline] 相机指针为空");
    if (job.is_main()) main_count++;
  }
  if (main_count != 1) {
    throw std::runtime_error(
      "[YoloPipeline] 必须有且只有一路主相机（camera_id = -1），当前 " +
      std::to_string(main_count) + " 路");
  }

  std::vector<int> configured;
  const auto yaml = tools::load(config_path);
  if (yaml["yolo_cpu_cores"]) {
    for (const auto & node : yaml["yolo_cpu_cores"]) configured.push_back(node.as<int>());
  }
  cores_ = resolve_cores(jobs_.size(), configured);
  max_omni_results_ = std::max<std::size_t>(8, jobs_.size() * 4);

  tools::logger()->info(
    "[YoloPipeline] 启动 {} 路相机推理（主相机 1 路 + 全向 {} 路），每路一个 YOLO 实例",
    jobs_.size(), jobs_.size() - 1);

  for (std::size_t i = 0; i < jobs_.size(); i++) {
    threads_.emplace_back(&YoloPipeline::run, this, i);
  }

  // 等所有线程完成“绑核 + YOLO 编译”；任何一路初始化失败就在启动阶段暴露
  {
    std::unique_lock<std::mutex> lock(ready_mutex_);
    ready_cv_.wait(lock, [this] { return ready_count_ == threads_.size(); });
  }

  if (init_error_) {
    quit_ = true;
    result_cv_.notify_all();
    for (auto & thread : threads_) {
      if (thread.joinable()) thread.join();
    }
    std::rethrow_exception(init_error_);
  }
}

YoloPipeline::~YoloPipeline()
{
  quit_ = true;
  result_cv_.notify_all();

  // 兜底：worker 可能正阻塞在 camera->read() 里等一帧。HikRobot 与虚拟数据源
  // 的读图循环会检查 tools::exiting()，置位后它们能立即返回空帧，worker 随即
  // 看到 quit_ 退出，避免 join 卡死。
  tools::request_exit();

  for (auto & thread : threads_) {
    if (thread.joinable()) thread.join();
  }
  tools::logger()->info("[YoloPipeline] 已停止。");
}

void YoloPipeline::set_inference_enabled(bool enabled)
{
  inference_enabled_.store(enabled, std::memory_order_relaxed);
}

WaitStatus YoloPipeline::wait_main(CameraResult & result, std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(result_mutex_);

  // 只等有限时间：相机掉线时主相机线程会一直卡在读图里，如果这里无限等，
  // 主循环既看不到 Ctrl+C / SIGTERM，也无法走到析构去置 quit_。
  const bool ready = result_cv_.wait_for(lock, timeout, [this] { return has_main_ || quit_; });
  if (!ready) return WaitStatus::Timeout;
  if (!has_main_) return WaitStatus::Stopped;

  result = std::move(main_result_);
  has_main_ = false;
  return WaitStatus::Ok;
}

void YoloPipeline::drain_omni(std::vector<CameraResult> & results)
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  results.reserve(results.size() + omni_results_.size());
  while (!omni_results_.empty()) {
    results.push_back(std::move(omni_results_.front()));
    omni_results_.pop_front();
  }
}

void YoloPipeline::mark_ready(std::exception_ptr error)
{
  std::lock_guard<std::mutex> lock(ready_mutex_);
  if (error && !init_error_) init_error_ = std::move(error);
  ready_count_++;
  ready_cv_.notify_all();
}

void YoloPipeline::push(CameraResult && result)
{
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    if (result.is_main()) {
      // 主相机只保留最新一帧：主循环慢于相机时丢旧帧，而不是批量补算
      main_result_ = std::move(result);
      has_main_ = true;
    } else {
      omni_results_.push_back(std::move(result));
      while (omni_results_.size() > max_omni_results_) omni_results_.pop_front();
    }
  }
  result_cv_.notify_all();
}

void YoloPipeline::run(std::size_t index)
{
  const CameraJob & job = jobs_[index];
  const std::string name = job.camera->device_name();
  const int target_cpu = cores_[index];

  bool ready_notified = false;
  auto notify_ready = [&](std::exception_ptr error) {
    if (ready_notified) return;
    ready_notified = true;
    mark_ready(std::move(error));
  };

  try {
    const bool bound = tools::bind_current_thread_to_cpu(target_cpu);

    // 关键顺序：先绑核、后编译模型。OpenVINO 的 CPU 插件在 compile_model 期间创建
    // 线程池，新线程继承创建线程的亲和性掩码，推理线程因此也只会跑在这颗核上。
    YOLO yolo(config_path_, debug_, true);

    tools::logger()->info(
      "[YoloPipeline] '{}' 推理线程启动：目标核 {}，编译后实际核 {}（{}）", name, target_cpu,
      tools::current_thread_cpu(), bound ? "已绑定" : "未绑定");

    notify_ready(nullptr);

    std::size_t frames = 0;
    double last_inference_ms = 0.0;
    auto last_stats = std::chrono::steady_clock::now();

    while (!quit_) {
      cv::Mat img;
      std::chrono::steady_clock::time_point timestamp;
      job.camera->read(img, timestamp);
      if (img.empty()) {
        if (quit_) break;
        std::this_thread::sleep_for(kIdlePoll);
        continue;
      }

      CameraResult result;
      result.camera_id = job.camera_id;
      result.timestamp = timestamp;

      if (inference_enabled_.load(std::memory_order_relaxed)) {
        const auto begin = std::chrono::steady_clock::now();
        result.armors = yolo.detect(img);
        last_inference_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - begin)
                              .count();
        result.inference_ms = last_inference_ms;
      }

      if (job.postprocess) job.postprocess(img, timestamp, result.armors, result);

      // 主相机每帧都入队（驱动主循环、录像）；全向只在检测到目标时入队
      const bool should_push = result.is_main() || !result.armors.empty();
      if (should_push) {
        if (job.keep_image) result.img = img;
        push(std::move(result));
      }

      frames++;
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - last_stats).count();
      if (elapsed >= kStatsPeriodS) {
        tools::logger()->info(
          "[YoloPipeline] '{}' cpu={} {:.1f}fps infer={:.1f}ms", name,
          tools::current_thread_cpu(), static_cast<double>(frames) / elapsed,
          last_inference_ms);
        frames = 0;
        last_stats = now;
      }
    }
  } catch (const std::exception & e) {
    tools::logger()->error("[YoloPipeline] '{}' 推理线程异常：{}", name, e.what());
    notify_ready(std::current_exception());
  } catch (...) {
    tools::logger()->error("[YoloPipeline] '{}' 推理线程未知异常", name);
    notify_ready(std::current_exception());
  }

  // 主相机线程退出后主循环永远等不到数据，直接停机；全向线程退出只损失这一路
  if (job.is_main()) {
    quit_ = true;
    result_cv_.notify_all();
  }
}

}  // namespace multithread
}  // namespace auto_aim
