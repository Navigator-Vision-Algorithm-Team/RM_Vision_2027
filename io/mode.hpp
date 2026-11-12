#ifndef IO__MODE_HPP
#define IO__MODE_HPP

#include <string>
#include <vector>

namespace io
{
/*
 * 视觉工作模式。
 *
 * 配置里用字符串指定，MODES 提供字符串 <-> 枚举的查表依据。
 * 取值依次为：空闲（不接管云台）、自瞄、小能量机关、大能量机关、前哨站。
 */
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};

/* Mode 的字符串表，下标与枚举值一一对应，改动时必须同步维护 */
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

/* 发射机构选择，仅对双发射机构的兵种有意义 */
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};

/* ShootMode 的字符串表，下标与枚举值一一对应 */
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

}  // namespace io

#endif  // IO__MODE_HPP
