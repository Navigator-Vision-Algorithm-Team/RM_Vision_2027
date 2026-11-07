#include "soldier_cboard.hpp"

#include <algorithm>

#include "socketcan.hpp"
#include "tools/yaml.hpp"

namespace io
{

SoldierCBoard::SoldierCBoard(const std::string & config_path)
: SoldierCBoard(Writer{})
{
  can_ = std::make_shared<SocketCAN>(
    read_yaml(config_path), std::bind(&SoldierCBoard::callback, this, std::placeholders::_1),
    [this] { reset(); });
  writer_ = [this](std::uint32_t identifier, const minipc::Payload & payload) {
    can_frame frame{};
    frame.can_id = identifier;
    frame.can_dlc = 8;
    std::copy(payload.begin(), payload.end(), frame.data);
    can_->write(&frame, true);
  };
}

void SoldierCBoard::callback(const can_frame & frame)
{
  if (frame.can_dlc != 8) return;
  receive(frame.can_id, std::span<const std::uint8_t>(frame.data, frame.can_dlc));
}

std::string SoldierCBoard::read_yaml(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  return tools::read<std::string>(yaml, "can_interface");
}

}  // namespace io
