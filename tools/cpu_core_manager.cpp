#include "cpu_core_manager.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <fstream>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <sched.h>

#include "tools/cpu_affinity.hpp"

namespace tools
{
namespace
{

bool read_integer(const std::string & path, int & value)
{
  std::ifstream file(path);
  return static_cast<bool>(file >> value);
}

bool read_unsigned(const std::string & path, std::uint64_t & value)
{
  std::ifstream file(path);
  return static_cast<bool>(file >> value);
}

bool parse_cpu_list(const std::string & text, std::vector<int> & cpus)
{
  std::istringstream input(text);
  std::string item;
  while (std::getline(input, item, ',')) {
    const auto dash = item.find('-');
    try {
      const int first = std::stoi(item.substr(0, dash));
      const int last = dash == std::string::npos ? first : std::stoi(item.substr(dash + 1));
      if (first < 0 || last < first) return false;
      for (int cpu = first; cpu <= last; cpu++) cpus.push_back(cpu);
    } catch (const std::exception &) {
      return false;
    }
  }
  return !cpus.empty();
}

CpuCoreType decode_core_type(int type)
{
  if (type == 1) return CpuCoreType::Performance;
  if (type == 2) return CpuCoreType::Efficiency;
  return CpuCoreType::Unknown;
}

template<typename ReadSignal>
bool classify_unknown_cores(
  std::vector<CpuCoreInfo> & cores, ReadSignal read_signal, const std::string & root,
  const std::string & signal_path)
{
  std::vector<std::pair<std::size_t, std::uint64_t>> signals;
  for (std::size_t i = 0; i < cores.size(); i++) {
    if (cores[i].type != CpuCoreType::Unknown || cores[i].available_logical_cpus.empty()) continue;
    std::uint64_t value = 0;
    const int cpu = cores[i].available_logical_cpus.front();
    if (read_signal(
          root + "/cpu" + std::to_string(cpu) + signal_path, value))
    {
      signals.emplace_back(i, value);
    }
  }
  if (signals.size() < 2) return false;

  const auto [minimum, maximum] = std::minmax_element(
    signals.begin(), signals.end(),
    [](const auto & left, const auto & right) { return left.second < right.second; });
  if (maximum->second == 0 || minimum->second * 10 >= maximum->second * 9) return false;

  for (const auto & [index, value] : signals) {
    cores[index].type =
      value * 10 >= maximum->second * 9 ? CpuCoreType::Performance : CpuCoreType::Efficiency;
  }
  return true;
}

}  // namespace

CpuCoreManager::CpuCoreManager(const std::string & sysfs_cpu_root)
{
  cpu_set_t allowed;
  CPU_ZERO(&allowed);
  if (sched_getaffinity(0, sizeof(allowed), &allowed) != 0) {
    throw std::system_error(errno, std::generic_category(), "sched_getaffinity");
  }

  std::map<std::string, std::size_t> core_indices;
  for (int cpu = 0; cpu < CPU_SETSIZE; cpu++) {
    if (!CPU_ISSET(cpu, &allowed)) continue;
    allowed_cpus_.insert(cpu);

    const std::string cpu_path = sysfs_cpu_root + "/cpu" + std::to_string(cpu);
    int package_id = -1;
    int core_id = -1;
    int core_type_value = 0;
    const bool has_core_ids =
      read_integer(cpu_path + "/topology/physical_package_id", package_id) &&
      read_integer(cpu_path + "/topology/core_id", core_id);

    std::string key;
    if (has_core_ids) {
      key = "package:" + std::to_string(package_id) + ":core:" + std::to_string(core_id);
    } else {
      std::ifstream siblings_file(cpu_path + "/topology/thread_siblings_list");
      std::string siblings_text;
      std::vector<int> siblings;
      if (!std::getline(siblings_file, siblings_text) || !parse_cpu_list(siblings_text, siblings)) {
        continue;
      }
      key = "siblings:";
      for (const int sibling : siblings) key += std::to_string(sibling) + ",";
    }

    auto [it, inserted] = core_indices.emplace(key, cores_.size());
    if (inserted) {
      CpuCoreInfo core;
      core.package_id = package_id;
      core.core_id = core_id;
      cores_.push_back(std::move(core));
    }

    auto & core = cores_[it->second];
    core.available_logical_cpus.push_back(cpu);

    if (read_integer(cpu_path + "/topology/core_type", core_type_value)) {
      const auto type = decode_core_type(core_type_value);
      if (type != CpuCoreType::Unknown) core.type = type;
    }

  }

  for (auto & core : cores_) {
    std::sort(core.available_logical_cpus.begin(), core.available_logical_cpus.end());
  }

  const bool has_kernel_core_type = std::any_of(
    cores_.begin(), cores_.end(),
    [](const CpuCoreInfo & core) { return core.type != CpuCoreType::Unknown; });
  if (has_kernel_core_type) classification_source_ = "Linux core_type";

  const auto capacity_reader = [](const std::string & path, std::uint64_t & value) {
    return read_unsigned(path, value);
  };
  if (classify_unknown_cores(cores_, capacity_reader, sysfs_cpu_root, "/cpu_capacity"))
  {
    if (classification_source_.empty()) classification_source_ = "CPU capacity";
    else classification_source_ += " + CPU capacity";
  }

  const auto frequency_reader = [](const std::string & path, std::uint64_t & value) {
    return read_unsigned(path, value);
  };
  if (classify_unknown_cores(
        cores_, frequency_reader, sysfs_cpu_root, "/cpufreq/cpuinfo_max_freq"))
  {
    if (classification_source_.empty()) classification_source_ = "maximum frequency";
    else classification_source_ += " + maximum frequency";
  }

  if (classification_source_.empty()) classification_source_ = "unavailable";

  allocation_order_.resize(cores_.size());
  std::iota(allocation_order_.begin(), allocation_order_.end(), 0);
  const auto priority = [](CpuCoreType type) {
    switch (type) {
      case CpuCoreType::Performance:
        return 0;
      case CpuCoreType::Unknown:
        return 1;
      case CpuCoreType::Efficiency:
        return 2;
    }
    return 1;
  };
  std::stable_sort(
    allocation_order_.begin(), allocation_order_.end(), [&](std::size_t left, std::size_t right) {
      const auto left_priority = priority(cores_[left].type);
      const auto right_priority = priority(cores_[right].type);
      if (left_priority != right_priority) return left_priority < right_priority;
      return cores_[left].available_logical_cpus.front() <
             cores_[right].available_logical_cpus.front();
    });
  allocated_.resize(cores_.size(), false);
}

std::optional<int> CpuCoreManager::allocate_cpu()
{
  std::lock_guard<std::mutex> lock(allocation_mutex_);
  for (const auto index : allocation_order_) {
    if (allocated_[index]) continue;
    allocated_[index] = true;
    if (cores_[index].available_logical_cpus.empty()) continue;
    return cores_[index].available_logical_cpus.front();
  }
  return std::nullopt;
}

bool CpuCoreManager::is_cpu_allowed(int cpu) const
{
  return allowed_cpus_.contains(cpu);
}

bool CpuCoreManager::bind_current_thread_to_cpu(int cpu) const
{
  return is_cpu_allowed(cpu) && tools::bind_current_thread_to_cpu(cpu);
}

}  // namespace tools
