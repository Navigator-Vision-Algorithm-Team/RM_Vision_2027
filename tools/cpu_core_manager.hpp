#ifndef TOOLS__CPU_CORE_MANAGER_HPP
#define TOOLS__CPU_CORE_MANAGER_HPP

#include <cstddef>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace tools
{

enum class CpuCoreType
{
  Performance,
  Efficiency,
  Unknown
};

struct CpuCoreInfo
{
  int package_id = -1;
  int core_id = -1;
  CpuCoreType type = CpuCoreType::Unknown;
  std::vector<int> available_logical_cpus;
};

// Discovers physical cores among CPUs available to this process. Each allocation
// selects one logical CPU from one physical core, so SMT siblings are never
// allocated as separate cores.
class CpuCoreManager
{
public:
  explicit CpuCoreManager(const std::string & sysfs_cpu_root = "/sys/devices/system/cpu");

  // preferred 指定时优先在该类型的物理核里分配；该类型已耗尽（例如 x86 全为
  // 性能核、却要求 Efficiency）才回退到默认顺序，因此调用方需要判返回值的
  // 实际类型（见 core_type_of）决定是否告警。
  std::optional<int> allocate_cpu(CpuCoreType preferred = CpuCoreType::Unknown);
  bool is_cpu_allowed(int cpu) const;
  bool bind_current_thread_to_cpu(int cpu) const;

  // 逻辑 CPU 所属物理核的类型，找不到返回 Unknown。
  CpuCoreType core_type_of(int cpu) const;

  const std::vector<CpuCoreInfo> & cores() const { return cores_; }
  const std::string & classification_source() const { return classification_source_; }

private:
  std::vector<CpuCoreInfo> cores_;
  std::vector<std::size_t> allocation_order_;
  std::vector<bool> allocated_;
  std::set<int> allowed_cpus_;
  std::string classification_source_;
  std::mutex allocation_mutex_;
};

}  // namespace tools

#endif  // TOOLS__CPU_CORE_MANAGER_HPP
