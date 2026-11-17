#ifndef TOOLS__FRAME_TIMER_HPP
#define TOOLS__FRAME_TIMER_HPP

/*
 * 逐帧耗时埋点：统计"被实际执行的绘制帧"的耗时分布，供性能评估使用。
 *
 * 数据流：start() 起点 → stop() 记录一个样本 → 每 300 帧 report() 输出 p50 / p99
 *         并清空样本，避免长跑时无界增长。
 *
 * 依赖与边界：只依赖 tools/logger.hpp 与标准库；头文件内实现，不新增构建目标。
 *
 * 线程：仅供单线程使用（本项目两个入口的绘制都在主循环线程），内部无锁。
 *
 * 使用注意：被门控跳过的帧不要调用 start/stop，否则会把"没画"误统计成"画得很快"。
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "tools/logger.hpp"

namespace tools
{

class FrameTimer
{
public:
  // 功能：记录本帧的起始时刻
  // 输入：无
  // 输出：无
  // 备注：必须与 stop() 成对调用；重复 start 会覆盖上一次起点
  void start() { start_ = Clock::now(); }

  // 功能：结束本帧计时，保存一个样本
  // 输入：无
  // 输出：无
  // 备注：达到报告周期时自动调用 report()；样本缓冲随后清空
  void stop()
  {
    const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    samples_.push_back(ms);
    ++total_;
    if (total_ % kReportEvery == 0) report();
  }

  // 功能：输出当前缓冲内样本的 p50 / p99，并清空缓冲
  // 输入：无
  // 输出：无，以 debug 级别写日志
  // 备注：只在 stop() 内自动调用；排序只在报告周期发生，不在热路径上
  void report()
  {
    if (samples_.empty()) return;

    std::vector<double> sorted = samples_;  // 每 300 帧一次，开销可忽略
    std::sort(sorted.begin(), sorted.end());

    const size_t p50 = sorted.size() / 2;
    const size_t p99 = std::min(sorted.size() - 1, sorted.size() * 99 / 100);
    tools::logger()->debug(
      "[frame-timer] 绘制 {} 帧：p50 {:.3f} ms，p99 {:.3f} ms，max {:.3f} ms（累计 {} 帧）",
      samples_.size(), sorted[p50], sorted[p99], sorted.back(), total_);
    samples_.clear();
  }

private:
  using Clock = std::chrono::steady_clock;  // 单调时钟，不受系统时间调整影响

  static constexpr uint64_t kReportEvery = 300;  // 每多少帧输出一次统计

  Clock::time_point start_{};          // 本帧起点，由 start() 写入
  std::vector<double> samples_;        // 未上报的耗时样本，ms；报告周期后清空
  uint64_t total_ = 0;                 // 累计记录的帧数，用于计算报告周期
};

}  // namespace tools

#endif  // TOOLS__FRAME_TIMER_HPP
