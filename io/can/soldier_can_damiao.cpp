#include "soldier_can_board.hpp"

#include <stdexcept>
#include <unordered_map>

#include <dmcan.h>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{

class SoldierCanBoard::DamiaoCan
{
public:
  DamiaoCan(SoldierCanBoard & board, std::uint8_t channel) : board_(board), channel_(channel)
  {
    dmcan_context * context = nullptr;
    dmcan_context_create(&context);
    context_.reset(context);
    if (!context_) throw std::runtime_error("cannot create Damiao SDK context");

    if (dmcan_find_devices(context_.get()) != 1) {
      throw std::runtime_error("expected exactly one Damiao USB-CAN device; check USB and permissions");
    }
    if (!dmcan_device_get(context_.get(), &device_, 0) || !device_ ||
        !dmcan_device_open(device_)) {
      throw std::runtime_error("cannot open Damiao USB-CAN device");
    }
    if (!dmcan_device_enable_channel(device_, channel_)) {
      throw std::runtime_error("cannot enable the configured Damiao CAN channel");
    }

    dmcan_channel_can_info_t bitrate{};
    if (!dmcan_device_get_channel_baudrate(device_, channel_, &bitrate)) {
      throw std::runtime_error("cannot read Damiao CAN channel settings");
    }
    bitrate.channel = channel_;
    bitrate.canfd = false;
    bitrate.can_baudrate = soldier_can::config::kBitrate;
    bitrate.can_sp = 0.75f;
    if (!dmcan_device_set_channel_baudrate(device_, channel_, bitrate)) {
      throw std::runtime_error("cannot configure Damiao channel for CAN 2.0 at 1 Mbps");
    }

    dmcan_channel_can_info_t confirmed{};
    if (!dmcan_device_get_channel_baudrate(device_, channel_, &confirmed) || confirmed.canfd ||
        confirmed.can_baudrate != soldier_can::config::kBitrate) {
      throw std::runtime_error("Damiao channel did not confirm CAN 2.0 at 1 Mbps");
    }

    tools::logger()->info("[soldier] Damiao USB-CAN channel {}: CAN 2.0, 1 Mbps", channel_);
    {
      auto & callbacks = registry();
      const std::lock_guard lock(callbacks.mutex);
      callbacks.boards.emplace(device_, this);
    }
    dmcan_device_hook_recv_callback(device_, &on_receive);
  }

  ~DamiaoCan()
  {
    // 等待正在分发的回调结束，再关闭 SDK 上下文。关闭时不持有回调锁。
    auto & callbacks = registry();
    {
      const std::lock_guard lock(callbacks.mutex);
      callbacks.boards.erase(device_);
    }
    context_.reset();
  }

  void write(std::uint32_t id, const soldier_can::Payload & payload)
  {
    auto bytes = payload;
    // SDK 参数依次为 FD、扩展帧、远程帧、BRS，步兵协议四项均关闭。
    if (!dmcan_device_send_can(device_, channel_, id, false, false, false, false,
                              soldier_can::config::kPayloadSize, bytes.data())) {
      throw std::runtime_error("Damiao CAN 2.0 write failed");
    }
  }

private:
  struct CallbackRegistry
  {
    std::mutex mutex;
    std::unordered_map<dmcan_device_handle *, DamiaoCan *> boards;
  };

  static CallbackRegistry & registry()
  {
    static CallbackRegistry callbacks;
    return callbacks;
  }

  static void on_receive(dmcan_device_handle * device, usb_rx_frame_t * frame) noexcept
  {
    if (!frame) return;
    const auto & head = frame->head;
    if (head.canfd || head.ext || head.rtr || head.brs ||
        head.dlc != soldier_can::config::kPayloadSize) return;

    try {
      auto & callbacks = registry();
      const std::lock_guard lock(callbacks.mutex);
      const auto entry = callbacks.boards.find(device);
      if (entry == callbacks.boards.end() || head.channel != entry->second->channel_) return;
      entry->second->board_.receive(
        head.can_id, std::span<const std::uint8_t>(frame->payload, head.dlc));
    } catch (...) {
      // 接收回调由 SDK 的线程调用，异常不能穿过 C 接口。
    }
  }

  SoldierCanBoard & board_;
  const std::uint8_t channel_;
  std::unique_ptr<dmcan_context, decltype(&dmcan_context_destroy)> context_{
    nullptr, &dmcan_context_destroy};
  dmcan_device_handle * device_ = nullptr;
};

SoldierCanBoard::SoldierCanBoard(const std::string & config_path)
: SoldierCanBoard(Writer{})
{
  const auto yaml = tools::load(config_path);
  const auto channel = yaml["can_channel"] ? yaml["can_channel"].as<int>() : 0;
  if (channel != 0 && channel != 1) {
    throw std::invalid_argument("can_channel must be 0 or 1 for Damiao USB-CAN");
  }
  can_ = std::make_shared<DamiaoCan>(*this, static_cast<std::uint8_t>(channel));
  writer_ = [this](std::uint32_t identifier, const soldier_can::Payload & payload) {
    can_->write(identifier, payload);
  };
}

}  // namespace io
