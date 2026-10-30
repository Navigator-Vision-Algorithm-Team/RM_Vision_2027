#ifndef IO__SIM__TRANSPORT_HPP
#define IO__SIM__TRANSPORT_HPP

// 字节传输抽象：业务/解析代码只依赖 IByteTransport，
// 真实串口与虚拟数据源（内置生成 / 回放）行为一致（阻塞、超时、部分读取语义相同）。

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "io/sim/common.hpp"
#include "serial/serial.h"

namespace io::sim
{

class IByteTransport
{
public:
  using Ptr = std::unique_ptr<IByteTransport>;
  virtual ~IByteTransport() = default;

  virtual void open() = 0;   // 失败抛异常
  virtual void close() = 0;
  // 最多读取 size 字节，最多等待 timeout_ms；返回实际读取字节数（可能 < size）
  virtual size_t read(uint8_t * buffer, size_t size, int timeout_ms) = 0;
  virtual size_t write(const uint8_t * data, size_t size) = 0;
  virtual std::string describe() const = 0;
};

// ============================ 真实串口 ============================
class RealSerialTransport : public IByteTransport
{
public:
  RealSerialTransport(std::string port, uint32_t baud, int timeout_ms = 50);
  ~RealSerialTransport() override;

  void open() override;
  void close() override;
  size_t read(uint8_t * buffer, size_t size, int timeout_ms) override;
  size_t write(const uint8_t * data, size_t size) override;
  std::string describe() const override;

private:
  std::string port_;
  uint32_t baud_;
  int timeout_ms_;
  serial::Serial serial_;
  bool opened_ = false;
};

// ============================ 字节生产器 ============================
// 虚拟下位机 / 回放器的统一接口：把字节推给 sink（由 VirtualSerialTransport 提供）
class IByteProducer
{
public:
  using Ptr = std::unique_ptr<IByteProducer>;
  using Sink = std::function<void(const uint8_t * data, size_t size)>;
  virtual ~IByteProducer() = default;

  virtual void start(const Sink & sink) = 0;  // 启动生产线程
  virtual void stop() = 0;                    // 停止并回收线程
  virtual std::string name() const = 0;
};

// ============================ 虚拟串口 ============================
// 接收生产器推入的字节，提供与真实串口一致的 read/write 语义；
// 本端 write 的字节被缓存，用于记录/校验视觉下发内容。
class VirtualSerialTransport : public IByteTransport
{
public:
  explicit VirtualSerialTransport(IByteProducer::Ptr producer);
  ~VirtualSerialTransport() override;

  void open() override;
  void close() override;
  size_t read(uint8_t * buffer, size_t size, int timeout_ms) override;
  size_t write(const uint8_t * data, size_t size) override;
  std::string describe() const override;

  // 取出并清空本端已写出的字节
  std::vector<uint8_t> take_written();

private:
  void push(const uint8_t * data, size_t size);

  IByteProducer::Ptr producer_;
  std::deque<uint8_t> rx_;
  std::deque<uint8_t> tx_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool opened_ = false;
  bool quitting_ = false;
};

// 按配置创建传输层：
//   real   → RealSerialTransport(com_port)
//   fake   → external ? RealSerialTransport(com_port, 指向 Python pty)
//                     : VirtualSerialTransport(内置虚拟下位机)
//   replay → VirtualSerialTransport(原始字节回放器)
IByteTransport::Ptr make_transport(const SimConfig & config);

}  // namespace io::sim

#endif  // IO__SIM__TRANSPORT_HPP
