#include "logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

namespace tools
{
namespace
{
/*
 * 创建 logs/ 目录。只读文件系统或权限不足时返回 false，
 * 由调用方退化为仅控制台输出，避免日志初始化成为启动失败点。
 */
bool ensure_log_dir()
{
  std::error_code ec;
  std::filesystem::create_directories("logs", ec);
  return !ec;
}
}  // namespace

std::shared_ptr<spdlog::logger> logger_ = nullptr;

void set_logger()
{
  std::vector<spdlog::sink_ptr> sinks;

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::debug);
  sinks.push_back(console_sink);

  if (ensure_log_dir()) {
    try {
      auto file_name =
        fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());
      auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
      file_sink->set_level(spdlog::level::debug);
      sinks.push_back(file_sink);
    } catch (const spdlog::spdlog_ex &) {
      // 文件 sink 打不开时只保留控制台输出
    }
  }

  logger_ = std::make_shared<spdlog::logger>("", sinks.begin(), sinks.end());
  logger_->set_level(spdlog::level::debug);
  logger_->flush_on(spdlog::level::info);
}

std::shared_ptr<spdlog::logger> logger()
{
  if (!logger_) set_logger();
  return logger_;
}

}  // namespace tools
