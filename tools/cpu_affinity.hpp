#ifndef TOOLS__CPU_AFFINITY_HPP
#define TOOLS__CPU_AFFINITY_HPP

// CPU 亲和性工具（仅 Linux 生效，其它平台退化为空操作）。
//
// 用途：把每一路 YOLO 推理线程钉在固定核上。OpenVINO 的 CPU 插件默认会按可用核
// 数创建线程池，多个实例同时推理时线程会在核间迁移，带来缓存失效与上下文切换
// 开销；固定核之后，每路推理的算力边界是可预期的。

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

#include <thread>

namespace tools
{

// 把“当前线程”绑定到指定 CPU 核。cpu < 0 或平台不支持时返回 false（不绑定）。
inline bool bind_current_thread_to_cpu(int cpu)
{
#if defined(__linux__)
  if (cpu < 0) return false;
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
#else
  (void)cpu;
  return false;
#endif
}

// 当前线程实际所在（或最近运行过）的核，取不到返回 -1。
inline int current_thread_cpu()
{
#if defined(__linux__)
  const int cpu = sched_getcpu();
  return cpu < 0 ? -1 : cpu;
#else
  return -1;
#endif
}

inline int hardware_concurrency()
{
  return static_cast<int>(std::thread::hardware_concurrency());
}

}  // namespace tools

#endif  // TOOLS__CPU_AFFINITY_HPP
