#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools
{
bool exiting();

// 请求退出：置位全局退出标志。
// 用于让仍在阻塞等待（如相机 read）的线程尽快返回，避免退出时 join 卡死。
void request_exit();

class Exiter
{
public:
  Exiter();

  bool exit() const;
};

}  // namespace tools

#endif  // TOOLS__EXITER_HPP