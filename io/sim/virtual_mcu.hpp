#ifndef IO__SIM__VIRTUAL_MCU_HPP
#define IO__SIM__VIRTUAL_MCU_HPP

// 虚拟下位机：按真实协议（0xA5 帧头 + CRC8/CRC16 + 0x502/0x503/0x505/0x0001）
// 与真实频率打包四元数等数据，可注入丢包/延迟/跳变/校验错误；
// 以及原始字节回放器，用于复现 bug。

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include "io/sim/common.hpp"
#include "io/sim/recorder.hpp"
#include "io/sim/transport.hpp"

namespace io::sim
{

class VirtualMcu : public IByteProducer
{
public:
  VirtualMcu(const SerialSourceConfig & cfg, const FaultConfig & fault, int seed, uint8_t robot_id = 0);
  ~VirtualMcu() override;

  void start(const Sink & sink) override;
  void stop() override;
  std::string name() const override { return "VirtualMcu"; }

private:
  void run(const Sink & sink);
  void emit_imu(const Sink & sink);
  void emit_chassis(const Sink & sink);
  void emit_status(const Sink & sink);
  void emit_bass(const Sink & sink, bool reset);
  void maybe_log();

  SerialSourceConfig cfg_;
  FaultConfig fault_;
  FaultInjector injector_;
  uint8_t robot_id_;
  double t_now_ = 0.0;

  std::thread thread_;
  std::atomic<bool> quit_{false};

  long long imu_index_ = 0;
  long long emitted_ = 0;
  long long dropped_ = 0;
  long long injected_ = 0;
  long long corrupt_ = 0;
  long long delayed_ = 0;
};

class SerialReplayer : public IByteProducer
{
public:
  SerialReplayer(std::string file, double speed = 1.0, bool loop = false);
  ~SerialReplayer() override;

  void start(const Sink & sink) override;
  void stop() override;
  std::string name() const override { return "SerialReplay"; }

  bool ready() const { return ready_; }

private:
  void run(const Sink & sink);

  std::string file_;
  double speed_;
  bool loop_;
  bool ready_ = false;
  SerialReplayReader reader_;
  std::thread thread_;
  std::atomic<bool> quit_{false};
};

}  // namespace io::sim

#endif  // IO__SIM__VIRTUAL_MCU_HPP
