#include "ov_device.hpp"

#include <exception>

#include "tools/logger.hpp"

namespace auto_aim
{
namespace
{
// CPU 专属属性：限制每个实例的推理线程/stream 数，避免多实例时线程数成倍膨胀。
// GPU / NPU 不认识这些属性，只在确定编译到 CPU 时才加。
ov::AnyMap cpu_properties(
  ov::hint::PerformanceMode mode, int num_threads, bool cpu)
{
  ov::AnyMap props;
  props.emplace(ov::hint::performance_mode(mode));
  if (cpu && num_threads > 0) {
    props.emplace(ov::inference_num_threads(num_threads));
    props.emplace(ov::streams::num(num_threads));
    // 亲和性由调用方（YoloPipeline 绑核）自己管：OpenVINO 默认的 CPU pinning 会
    // 按自己的策略重设线程亲和性，把已经绑好的线程又挪走，这里明确关掉。
    props.emplace(ov::hint::enable_cpu_pinning(false));
  }
  return props;
}
}  // namespace

std::string device_from_config(const std::string & configured)
{
  if (configured.empty() || configured == "auto") return "auto";
  return configured;
}

ov::CompiledModel compile_ov_model(
  ov::Core & core, const std::shared_ptr<ov::Model> & model, const std::string & device,
  ov::hint::PerformanceMode mode, int num_threads)
{
  if (device == "auto") {
    try {
      return core.compile_model(model, "GPU", cpu_properties(mode, num_threads, false));
    } catch (const std::exception & e) {
      // ARM 板没有 GPU 插件，走到这里属于正常情况，用 info 级别避免日志噪音
      tools::logger()->info("[OpenVINO] GPU 不可用，改用 CPU 推理：{}", e.what());
      return core.compile_model(model, "CPU", cpu_properties(mode, num_threads, true));
    }
  }

  try {
    return core.compile_model(
      model, device, cpu_properties(mode, num_threads, device == "CPU"));
  } catch (const std::exception & e) {
    if (device == "CPU") throw;
    tools::logger()->warn("[OpenVINO] device={} 不可用（{}），回退 CPU", device, e.what());
    return core.compile_model(model, "CPU", cpu_properties(mode, num_threads, true));
  }
}

}  // namespace auto_aim
