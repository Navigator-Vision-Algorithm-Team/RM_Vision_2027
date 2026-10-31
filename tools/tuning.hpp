#ifndef TOOLS__TUNING_HPP
#define TOOLS__TUNING_HPP

#include <array>
#include <string>

namespace tools
{

// 卡尔曼滤波 / 跟踪器的可调参数。
//
// 全部从配置文件的 tuning: 段读取；文件里没写的键用结构体里的默认值，
// 而这些默认值与「搬到 yaml 之前」的硬编码值完全一致，所以不配置时行为不变。
//
// 用法：在 Tracker 构造时调用一次 load_tuning(config_path)，
// 之后任何地方用 tuning() 读取即可（返回的是进程内唯一的一份，只读）。
struct Tuning
{
  // ---- 过程噪声（target.cpp 的 Q）----
  double v1 = 100.0;             // 普通目标：加速度方差
  double v2 = 400.0;             // 普通目标：角加速度方差
  double v1_outpost = 10.0;      // 前哨站：加速度方差
  double v2_outpost = 0.1;       // 前哨站：角加速度方差
  double outpost_w_trigger = 2.0;  // 前哨站转速超过该值开始限幅
  double outpost_w_limit = 2.51;   // 限幅后的转速上限

  // ---- 观测噪声（target.cpp 的 R_dig）----
  double R_yaw = 4e-3;           // yaw 观测方差 (rad^2)
  double R_pitch = 4e-3;         // pitch 观测方差 (rad^2)
  double R_dist_log_gain = 1.0;  // 距离项：log(|Δangle|+1) 的系数
  double R_dist_base = 1.0;      // 距离项：常数偏置
  double R_armor_log_gain = 0.005;  // 装甲板朝向项：log(dist+1) 的系数（原本是 /200）
  double R_armor_base = 9e-2;       // 装甲板朝向项：常数偏置

  // ---- 初始化（tracker.cpp 的 P0 / radius / armor_num）----
  std::array<double, 11> P0{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1};
  double radius = 0.2;
  int armor_num = 4;

  // ---- 判据 / 状态机 ----
  double r_min = 0.05;                     // diverged(): r 下限
  double r_max = 0.5;                      // diverged(): r 上限
  int converged_update_count = 3;          // 普通目标判定收敛所需更新次数
  int outpost_converged_update_count = 10; // 前哨站判定收敛所需更新次数
  double max_dt = 0.1;                     // 帧间隔超过该值直接判丢 (s)

  // ---- 图像尺寸（选板时用于计算画面中心）----
  int image_width = 1440;
  int image_height = 1080;

  // ---- NIS / NEES（扩展卡尔曼滤波内部）----
  int nis_window = 100;
  double nis_threshold = 0.711;
  double nees_threshold = 0.711;
};

// 进程内唯一的一份参数（未调用 load_tuning 时返回全默认值）
const Tuning & tuning();

// 读取 config_path 指向的 yaml 的 tuning: 段；文件或字段缺失时保留默认值。
// 解析异常只打警告，不中断程序。
void load_tuning(const std::string & config_path);

}  // namespace tools

#endif  // TOOLS__TUNING_HPP
