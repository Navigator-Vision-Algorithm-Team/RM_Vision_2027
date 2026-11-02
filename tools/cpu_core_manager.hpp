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

  std::optional<int> allocate_cpu();
  bool is_cpu_allowed(int cpu) const;
  bool bind_current_thread_to_cpu(int cpu) const;

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
