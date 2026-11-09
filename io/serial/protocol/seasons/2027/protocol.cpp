#include "io/serial/protocol/seasons/2027/protocol.hpp"

#include <cstring>

namespace referee_comm
{
namespace v2027
{

template <typename T>
bool Protocol::Assign(const Frame & frame, T & target, size_t expected)
{
  if (expected == 0 || expected > sizeof(T) || frame.length != expected) return false;
  std::memcpy(&target, frame.data, expected);
  return true;
}

bool Protocol::OnCommand(const Frame & frame)
{
  if (frame.data == nullptr) return false;

  switch (frame.cmd_id) {
    case GAME_STATUS:
      return Assign(frame, game_status, sizeof(game_status_t));
    case GAME_RESULT:
      return Assign(frame, game_result, sizeof(game_result_t));
    case GAME_ROBOT_HP:
      return Assign(frame, game_robot_hp, sizeof(game_robot_HP_t));
    case ROBOT_STATUS:
      return Assign(frame, robot_status, sizeof(robot_status_t));
    case POWER_HEAT_DATA:
      return Assign(frame, power_heat_data, sizeof(power_heat_data_t));
    case HURT_DATA:
      return Assign(frame, hurt_data, sizeof(hurt_data_t));
    case SHOOT_DATA:
      return Assign(frame, shoot_data, sizeof(shoot_data_t));
    case PROJECTILE_ALLOWANCE:
      return Assign(frame, projectile_allowance, sizeof(projectile_allowance_t));
    case SENTRY_INFO:
      return Assign(frame, sentry_info, sizeof(sentry_info_t));

    default:
      /* 未知或已废弃命令：静默忽略，不影响字节流同步 */
      return false;
  }
}

bool Protocol::IsInBattle() const
{
  // 0x0001 的 game_progress 为 4 表示比赛中。
  return game_status.game_progress == 4;
}

bool Protocol::IsDead() const
{
  return robot_status.current_HP == 0;
}

uint16_t Protocol::BufferEnergy() const
{
  return power_heat_data.buffer_energy;
}

uint16_t Protocol::BarrelHeat17mm() const
{
  return power_heat_data.shooter_17mm_barrel_heat;
}

}  // namespace v2027
}  // namespace referee_comm
