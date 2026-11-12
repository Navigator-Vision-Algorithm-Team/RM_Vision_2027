#include "yolo_pipeline.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>

#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/cpu_affinity.hpp"
#include "tools/cpu_core_manager.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

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

std::vector<int> allocate_cores(
  const std::vector<CameraJob> & jobs, tools::CpuCoreManager & manager)
{
  std::vector<int> cores(jobs.size(), -1);
  for (std::size_t i = 0; i < jobs.size(); i++) {
    if (!jobs[i].pin_cpu) continue;
    // 按任务的核类型偏好分配：全向相机申请能效核，性能核留给不绑核的主相机。
    const auto cpu = manager.allocate_cpu(jobs[i].core_preference);
    if (cpu) cores[i] = *cpu;
  }
  return cores;
}

const char * core_type_name(tools::CpuCoreType type)
{
  switch (type) {
    case tools::CpuCoreType::Performance:
      return "性能核";
    case tools::CpuCoreType::Efficiency:
      return "能效核";
    case tools::CpuCoreType::Unknown:
      return "未分类";
  }
  return "未分类";
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

  cores_ = allocate_cores(jobs_, cpu_manager_);
  const auto & physical_cores = cpu_manager_.cores();
  const auto performance_count = std::count_if(
    physical_cores.begin(), physical_cores.end(), [](const tools::CpuCoreInfo & core) {
      return core.type == tools::CpuCoreType::Performance;
    });
  const auto efficiency_count = std::count_if(
    physical_cores.begin(), physical_cores.end(), [](const tools::CpuCoreInfo & core) {
      return core.type == tools::CpuCoreType::Efficiency;
    });
  tools::logger()->info(
    "[YoloPipeline] CPU 拓扑：物理核 {} 个（性能核 {}，能效核 {}），识别来源：{}",
    physical_cores.size(), performance_count, efficiency_count,
    cpu_manager_.classification_source());
  const auto pin_required = std::count_if(jobs_.begin(), jobs_.end(), [](const CameraJob & job) {
    return job.pin_cpu;
  });
  const auto assigned_count = std::count_if(cores_.begin(), cores_.end(), [](int cpu) {
    return cpu >= 0;
  });
  if (assigned_count < pin_required) {
    tools::logger()->warn(
      "[YoloPipeline] 可分配的物理核不足（请求绑核 {} 路，可独占分配 {} 路），超出部分不绑核",
      pin_required, assigned_count);
  }
  if (physical_cores.empty()) {
    tools::logger()->warn("[YoloPipeline] 无法读取可用 CPU 拓扑，推理线程将不绑定 CPU");
  }

  // 逐路确认绑核结果：偏好类型申请不到时必须显式告警，否则会静默把全向
  // 塞回性能核、继续和主相机抢大核。
  for (std::size_t i = 0; i < jobs_.size(); i++) {
    const std::string name = jobs_[i].camera->device_name();
    if (!jobs_[i].pin_cpu) {
      tools::logger()->info(
        "[YoloPipeline] '{}' 不绑核（OpenVINO 自主调度）", name);
      continue;
    }
    if (cores_[i] < 0) {
      tools::logger()->warn("[YoloPipeline] '{}' 没有可用核，将不绑核", name);
      continue;
    }
    const auto actual = cpu_manager_.core_type_of(cores_[i]);
    if (
      jobs_[i].core_preference != tools::CpuCoreType::Unknown &&
      actual != jobs_[i].core_preference)
    {
      tools::logger()->warn(
        "[YoloPipeline] '{}' 申请的{}不足，回退到 cpu={}（{}）", name,
        core_type_name(jobs_[i].core_preference), cores_[i], core_type_name(actual));
    } else {
      tools::logger()->info(
        "[YoloPipeline] '{}' 绑核 cpu={}（{}）", name, cores_[i], core_type_name(actual));
    }
  }
  max_omni_results_ = std::max<std::size_t>(8, jobs_.size() * 4);

  const auto traditional_count = std::count_if(
    jobs_.begin(), jobs_.end(), [](const CameraJob & job) { return job.use_traditional; });
  tools::logger()->info(
    "[YoloPipeline] 启动 {} 路相机推理（主相机 1 路 + 全向 {} 路），"
    "其中 {} 路使用传统检测、{} 路使用 YOLO",
    jobs_.size(), jobs_.size() - 1, traditional_count, jobs_.size() - traditional_count);

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
      // 同一路相机只保留最新一帧：主循环慢于推理时，旧结果已经没有使用价值
      // （下游只取队首的一条），继续堆积只会让 drain_omni 每帧多搬几份无用的
      // list<Armor>。不同路之间仍保持"先到先排"的轮转顺序。
      const int id = result.camera_id;
      auto slot = std::find_if(
        omni_results_.begin(), omni_results_.end(),
        [id](const CameraResult & item) { return item.camera_id == id; });
      if (slot != omni_results_.end()) {
        *slot = std::move(result);
      } else {
        omni_results_.push_back(std::move(result));
        while (omni_results_.size() > max_omni_results_) omni_results_.pop_front();
      }
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
    const bool should_pin = job.pin_cpu && target_cpu >= 0;
    const bool bound = should_pin ? cpu_manager_.bind_current_thread_to_cpu(target_cpu) : false;
    if (should_pin && !bound) {
      tools::logger()->warn(
        "[YoloPipeline] '{}' 无法绑定到逻辑 CPU {}，将由调度器自由调度", name, target_cpu);
    }

    // 关键顺序：先绑核、后构造检测器。对 pin_cpu=true 的任务，OpenVINO 的 CPU 插件
    // 在 compile_model 期间创建线程池，新线程继承创建线程的亲和性掩码。
    //
    // 检测后端二选一：use_traditional=true 的路（全向相机）完全不创建 YOLO 实例，
    // 也就不加载 .xml 模型、不建 YOLO 的推理线程池；传统检测器只带一个
    // 32×32 的编号分类器，构造开销和常驻内存都远小于 YOLO。
    std::unique_ptr<YOLO> yolo;
    std::unique_ptr<Detector> traditional;
    if (job.use_traditional) {
      traditional = std::make_unique<Detector>(config_path_, false, job.traditional_key_prefix);
    } else {
      yolo = std::make_unique<YOLO>(config_path_, debug_, job.single_thread_cpu);
    }

    const char * backend = job.use_traditional ? "traditional" : "yolo";
    if (job.pin_cpu) {
      tools::logger()->info(
        "[YoloPipeline] '{}' 推理线程启动：backend={}，目标核 {}，编译后实际核 {}"
        "（{}，single_thread_cpu={}）",
        name, backend, target_cpu, tools::current_thread_cpu(), bound ? "已绑定" : "未绑定",
        job.single_thread_cpu ? "true" : "false");
    } else {
      tools::logger()->info(
        "[YoloPipeline] '{}' 推理线程启动：backend={}，不绑核（single_thread_cpu={}）", name,
        backend, job.single_thread_cpu ? "true" : "false");
    }

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
        result.armors = traditional ? traditional->detect(img) : yolo->detect(img);
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
