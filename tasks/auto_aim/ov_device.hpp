#ifndef AUTO_AIM__OV_DEVICE_HPP
#define AUTO_AIM__OV_DEVICE_HPP

#include <memory>
#include <string>

#include <openvino/openvino.hpp>

namespace auto_aim
{
// 解析配置里的 device 字段：
//   缺省 / "auto" -> "auto"，运行时自动挑选（见 compile_ov_model）
//   其它取值      -> 原样返回（CPU / GPU / GPU.0 / NPU ...）
//
// 存在的意义：x86 开发机通常有核显可用 GPU，而 ARM 板只有 CPU。若把设备名写死在
// 配置文件里，两条硬件线就必须各维护一套 yaml。改为 auto 后，同一份配置两端通用，
// 硬件差异只体现在运行时的设备探测结果上。
std::string device_from_config(const std::string & configured);

// 编译 OpenVINO 模型，并兜住"设备不可用"这种纯硬件差异：
//   device == "auto"：优先 GPU，失败回退 CPU
//   显式 device     ：失败时告警并回退 CPU（CPU 本身失败则原样抛出）
//
// num_threads > 0 时只对 CPU 生效：显式限制每个实例的推理线程数与 stream 数。
// 多相机场景下每路 YOLO 独占一个 CPU 核，若放任每个实例各开一套线程池，
// 线程数会按实例数成倍膨胀并互相抢核，因此这里强制单线程。
ov::CompiledModel compile_ov_model(
  ov::Core & core, const std::shared_ptr<ov::Model> & model, const std::string & device,
  ov::hint::PerformanceMode mode, int num_threads = 0);

}  // namespace auto_aim

#endif  // AUTO_AIM__OV_DEVICE_HPP
