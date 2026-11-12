#ifndef TOOLS__LOGGER_HPP
#define TOOLS__LOGGER_HPP

#include <spdlog/spdlog.h>

namespace tools
{
/*
 * 全局日志器，首次调用时惰性创建，之后返回同一个实例（spdlog logger 本身线程安全）。
 *
 * 输出两路：彩色 stdout + logs/<启动时刻>.log，级别均为 debug，info 及以上立即 flush。
 * logs/ 目录在初始化时自动创建；创建失败（只读文件系统、权限不足）时退化为仅控制台输出，
 * 不会因为写不了日志而让程序启动失败。
 */
std::shared_ptr<spdlog::logger> logger();

}  // namespace tools

#endif  // TOOLS__LOGGER_HPP
