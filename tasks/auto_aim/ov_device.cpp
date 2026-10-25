#include "ov_device.hpp"

#include <exception>

#include "tools/logger.hpp"

namespace auto_aim
{
std::string device_from_config(const std::string & configured)
{
  if (configured.empty() || configured == "auto") return "auto";
  return configured;
}

ov::CompiledModel compile_ov_model(
  ov::Core & core, const std::shared_ptr<ov::Model> & model, const std::string & device,
  ov::hint::PerformanceMode mode)
{
  const auto property = ov::hint::performance_mode(mode);

  if (device == "auto") {
    try {
      return core.compile_model(model, "GPU", property);
    } catch (const std::exception & e) {
      // ARM 板没有 GPU 插件，走到这里属于正常情况，用 info 级别避免日志噪音
      tools::logger()->info("[OpenVINO] GPU 不可用，改用 CPU 推理：{}", e.what());
      return core.compile_model(model, "CPU", property);
    }
  }

  try {
    return core.compile_model(model, device, property);
  } catch (const std::exception & e) {
    if (device == "CPU") throw;
    tools::logger()->warn("[OpenVINO] device={} 不可用（{}），回退 CPU", device, e.what());
    return core.compile_model(model, "CPU", property);
  }
}

}  // namespace auto_aim
