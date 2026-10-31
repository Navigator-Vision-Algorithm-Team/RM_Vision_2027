#include "tools/tuning.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace tools
{
namespace
{
Tuning g_tuning;

template <typename T>
void set_if(const YAML::Node & n, const char * key, T & dst)
{
  if (n[key]) dst = n[key].as<T>();
}

void load_p0(const YAML::Node & n, std::array<double, 11> & p0)
{
  if (!n) return;
  if (!n.IsSequence() || n.size() != p0.size()) {
    logger()->warn("[Tuning] P0 必须是 {} 个数字的数组，已忽略", p0.size());
    return;
  }
  for (size_t i = 0; i < p0.size(); ++i) p0[i] = n[i].as<double>();
}
}  // namespace

const Tuning & tuning() { return g_tuning; }

void load_tuning(const std::string & config_path)
{
  // 先全部用默认值，yaml 里缺的键不会改变行为
  Tuning t;

  try {
    const auto yaml = YAML::LoadFile(config_path);
    const auto n = yaml["tuning"];
    if (n) {
      // 过程噪声
      set_if(n, "v1", t.v1);
      set_if(n, "v2", t.v2);
      set_if(n, "v1_outpost", t.v1_outpost);
      set_if(n, "v2_outpost", t.v2_outpost);
      set_if(n, "outpost_w_trigger", t.outpost_w_trigger);
      set_if(n, "outpost_w_limit", t.outpost_w_limit);

      // 观测噪声
      set_if(n, "R_yaw", t.R_yaw);
      set_if(n, "R_pitch", t.R_pitch);
      set_if(n, "R_dist_log_gain", t.R_dist_log_gain);
      set_if(n, "R_dist_base", t.R_dist_base);
      set_if(n, "R_armor_log_gain", t.R_armor_log_gain);
      set_if(n, "R_armor_base", t.R_armor_base);

      // 初始化
      load_p0(n["P0"], t.P0);
      set_if(n, "radius", t.radius);
      set_if(n, "armor_num", t.armor_num);

      // 判据
      set_if(n, "r_min", t.r_min);
      set_if(n, "r_max", t.r_max);
      set_if(n, "converged_update_count", t.converged_update_count);
      set_if(n, "outpost_converged_update_count", t.outpost_converged_update_count);
      set_if(n, "max_dt", t.max_dt);

      // 图像尺寸
      set_if(n, "image_width", t.image_width);
      set_if(n, "image_height", t.image_height);

      // NIS / NEES
      set_if(n, "nis_window", t.nis_window);
      set_if(n, "nis_threshold", t.nis_threshold);
      set_if(n, "nees_threshold", t.nees_threshold);
    } else {
      logger()->info("[Tuning] {} 没有 tuning: 段，使用内置默认值", config_path);
    }
  } catch (const std::exception & e) {
    logger()->warn("[Tuning] 读取 {} 的 tuning 段失败，使用默认值: {}", config_path, e.what());
  }

  g_tuning = t;

  logger()->info(
    "[Tuning] v1={:.1f} v2={:.1f} | R_yaw={:.4g} R_pitch={:.4g} "
    "| P0[1]={:.0f} P0[7]={:.0f} P0[8]={:.3g} | radius={:.3f} armor_num={} "
    "| r_range=({:.3f},{:.3f}) max_dt={:.3f} nis_win={}",
    t.v1, t.v2, t.R_yaw, t.R_pitch, t.P0[1], t.P0[7], t.P0[8], t.radius, t.armor_num, t.r_min,
    t.r_max, t.max_dt, t.nis_window);
}

}  // namespace tools
