#include "io/sim/transport.hpp"

#include <algorithm>

#include "tools/logger.hpp"

namespace io::sim
{

// ============================ RealSerialTransport ============================
RealSerialTransport::RealSerialTransport(std::string port, uint32_t baud, int timeout_ms)
: port_(std::move(port)), baud_(baud), timeout_ms_(timeout_ms)
{
}

RealSerialTransport::~RealSerialTransport() { close(); }

void RealSerialTransport::open()
{
  try {
    serial_.setPort(port_);
    serial_.setBaudrate(baud_);
    serial_.setBytesize(serial::eightbits);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout_ms_));
    serial_.setTimeout(timeout);
    serial_.open();
    opened_ = true;
  } catch (const std::exception & e) {
    tools::logger()->error("[REAL] Failed to open serial {}: {}", port_, e.what());
    throw;
  }
  tools::logger()->info("[REAL] Serial {} opened, baud={}", port_, baud_);
}

void RealSerialTransport::close()
{
  try {
    if (serial_.isOpen()) serial_.close();
  } catch (...) {
  }
  opened_ = false;
}

size_t RealSerialTransport::read(uint8_t * buffer, size_t size, int timeout_ms)
{
  if (timeout_ms != timeout_ms_) {
    try {
      serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout_ms));
      serial_.setTimeout(timeout);
      timeout_ms_ = timeout_ms;
    } catch (...) {
    }
  }
  try {
    return serial_.read(buffer, size);
  } catch (const std::exception & e) {
    return 0;
  }
}

size_t RealSerialTransport::write(const uint8_t * data, size_t size)
{
  try {
    return serial_.write(data, size);
  } catch (const std::exception & e) {
    tools::logger()->warn("[REAL] Failed to write serial: {}", e.what());
    return 0;
  }
}

std::string RealSerialTransport::describe() const
{
  return "real:" + port_ + "@" + std::to_string(baud_);
}

// ============================ VirtualSerialTransport ============================
VirtualSerialTransport::VirtualSerialTransport(IByteProducer::Ptr producer)
: producer_(std::move(producer))
{
}

VirtualSerialTransport::~VirtualSerialTransport() { close(); }

void VirtualSerialTransport::open()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rx_.clear();
    tx_.clear();
    quitting_ = false;
    opened_ = true;
  }

  if (producer_) {
    producer_->start([this](const uint8_t * d, size_t n) { push(d, n); });
    tools::logger()->info("[VIRTUAL] Producer '{}' started", producer_->name());
  }
}

void VirtualSerialTransport::close()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!opened_) return;
    quitting_ = true;
    opened_ = false;
  }
  cv_.notify_all();
  if (producer_) producer_->stop();
}

void VirtualSerialTransport::push(const uint8_t * data, size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (quitting_ || !data || size == 0) return;
  rx_.insert(rx_.end(), data, data + size);
  cv_.notify_all();
}

size_t VirtualSerialTransport::read(uint8_t * buffer, size_t size, int timeout_ms)
{
  if (!buffer || size == 0) return 0;

  std::unique_lock<std::mutex> lock(mutex_);
  if (rx_.empty()) {
    cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return !rx_.empty() || quitting_; });
  }
  if (rx_.empty()) return 0;

  const size_t n = std::min(size, rx_.size());
  for (size_t i = 0; i < n; ++i) {
    buffer[i] = rx_.front();
    rx_.pop_front();
  }
  return n;
}

size_t VirtualSerialTransport::write(const uint8_t * data, size_t size)
{
  if (!data || size == 0) return 0;
  std::lock_guard<std::mutex> lock(mutex_);
  tx_.insert(tx_.end(), data, data + size);
  return size;
}

std::string VirtualSerialTransport::describe() const
{
  return "virtual:" + (producer_ ? producer_->name() : std::string("none"));
}

std::vector<uint8_t> VirtualSerialTransport::take_written()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> out(tx_.begin(), tx_.end());
  tx_.clear();
  return out;
}

}  // namespace io::sim
