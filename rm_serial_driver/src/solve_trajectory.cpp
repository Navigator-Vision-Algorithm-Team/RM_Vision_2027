/*
@brief: 弹道解算 - improved with sp_vision_25 aim point selection logic
@author: CodeAlan (original), adapted from TongjiSuperPower sp_vision_25
*/
// 近点只考虑水平方向的空气阻力

#include "rm_serial_driver/solve_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

struct tar_pos tar_position[4];  //最多只有四块装甲板
float t = 0.6f;                  // 飞行时间
struct SolveTrajectoryParams st;

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
  float z;
  //t为给定v与angle时的飞行时间
  t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
  if (t < 0) {
    t = 0;
    return 0;
  }
  //z为给定v与angle时的高度
  z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return z;
}

/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float completeAirResistanceModel(float s, float v, float angle) { return 0; }

/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
  float z_temp, z_actual, dz;
  float angle_pitch;
  int i = 0;
  z_temp = z;
  // iteration
  for (i = 0; i < 20; i++) {
    angle_pitch = atan2(z_temp, s);  // rad
    z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
    if (z_actual == 0) {
      angle_pitch = 0;
      break;
    }
    dz = 0.3 * (z - z_actual);
    z_temp = z_temp + dz;
    if (fabsf(dz) < 0.00001) {
      break;
    }
  }
  return angle_pitch;
}

/*
@brief 迭代求解飞行时间（用于打前兼顾发弹延时）
@param pitch: 传入传出的 pitch 角
@param s: 水平距离
@param z: 垂直距离
@param v: 弹速
*/
void iterativeFlyTimeCompensation(float * pitch, float * yaw, float * aim_x, float * aim_y, float * aim_z)
{
  // 迭代求解飞行时间 (最多10次)
  constexpr int max_iter = 10;
  float prev_fly_time = 0;
  float timeDelay = st.bias_time / 1000.0;

  for (int iter = 0; iter < max_iter; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    float total_delay = timeDelay + prev_fly_time;
    st.tar_yaw += st.v_yaw * total_delay;

    // 计算所有装甲板位置并选择
    float temp_aim_z = tar_position[0].z + st.vzw * total_delay;
    float temp_aim_x = tar_position[0].x + st.vxw * total_delay;
    float temp_aim_y = tar_position[0].y + st.vyw * total_delay;

    float temp_pitch = pitchTrajectoryCompensation(
      sqrt(temp_aim_x * temp_aim_x + temp_aim_y * temp_aim_y) - st.s_bias,
      temp_aim_z + st.z_bias, st.current_v);

    if (temp_pitch == 0) break;

    float fly_time = (float)((exp(st.k * sqrt(temp_aim_x * temp_aim_x + temp_aim_y * temp_aim_y)) - 1) /
                             (st.k * st.current_v * cos(temp_pitch)));

    if (fabsf(fly_time - prev_fly_time) < 0.001f) break;
    prev_fly_time = fly_time;
  }
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
使用 coming/leaving angle 逻辑（来自 sp_vision_25）用于陀螺目标
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
uint8_t autoSolveTrajectory(float * pitch, float * yaw, float * aim_x, float * aim_y, float * aim_z)
{
  // 线性预测
  float timeDelay = st.bias_time / 1000.0 + t;
  st.tar_yaw += st.v_yaw * timeDelay;

  //计算所有装甲板的位置
  int use_1 = 1;
  int i = 0;
  int idx = 0;  // 选择的装甲板
  //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
  if (st.armor_num == ARMOR_NUM_BALANCE) {
    for (i = 0; i < 2; i++) {
      float tmp_yaw = st.tar_yaw + i * PI;
      float r = st.r1;
      tar_position[i].x = st.xw - r * cos(tmp_yaw);
      tar_position[i].y = st.yw - r * sin(tmp_yaw);
      tar_position[i].z = st.zw;
      tar_position[i].yaw = tmp_yaw;
    }

    float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
    //因为是平衡步兵 只需判断两块装甲板即可
    float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
    if (temp_yaw_diff < yaw_diff_min) {
      yaw_diff_min = temp_yaw_diff;
      idx = 1;
    }

  } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
    for (i = 0; i < 3; i++) {
      float tmp_yaw = st.tar_yaw + i * 2.0 * PI / 3.0;  // 2/3PI
      float r = (st.r1 + st.r2) / 2;                    //理论上r1=r2 这里取个平均值
      tar_position[i].x = st.xw - r * cos(tmp_yaw);
      tar_position[i].y = st.yw - r * sin(tmp_yaw);
      tar_position[i].z = st.zw;
      tar_position[i].yaw = tmp_yaw;
    }

    // 前哨站选择最优装甲板：使用距离+瞄准角度混合决策
    float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
    idx = 0;
    for (i = 1; i < 3; i++) {
      float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
      if (temp_yaw_diff < yaw_diff_min) {
        yaw_diff_min = temp_yaw_diff;
        idx = i;
      }
    }

  } else {
    // 普通4装甲板
    for (i = 0; i < 4; i++) {
      float tmp_yaw = st.tar_yaw + i * PI / 2.0;
      float r = use_1 ? st.r1 : st.r2;
      tar_position[i].x = st.xw - r * cos(tmp_yaw);
      tar_position[i].y = st.yw - r * sin(tmp_yaw);
      tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
      tar_position[i].yaw = tmp_yaw;
      use_1 = !use_1;
    }

    // coming/leaving angle 逻辑（小陀螺目标）
    // 计算整车旋转中心到云台的 yaw
    float center_yaw = atan2f(st.yw, st.xw);

    // 如果目标在旋转（转速 > 2 rad/s），使用 coming/leaving angle 逻辑
    constexpr float spin_threshold = 2.0f;
    constexpr float coming_angle = 55.0f * PI / 180.0f;   // 55 degrees
    constexpr float leaving_angle = 20.0f * PI / 180.0f;  // 20 degrees

    if (fabsf(st.v_yaw) > spin_threshold) {
      // 小陀螺模式：优先选择 coming 阶段的装甲板
      float ca = coming_angle;
      float la = leaving_angle;

      std::vector<int> candidate_indices;
      for (i = 0; i < 4; i++) {
        float delta_angle = tar_position[i].yaw - center_yaw;
        // Normalize to [-pi, pi]
        while (delta_angle > PI) delta_angle -= 2 * PI;
        while (delta_angle <= -PI) delta_angle += 2 * PI;

        if (fabsf(delta_angle) > ca) continue;
        if (st.v_yaw > 0 && delta_angle < la) {
          candidate_indices.push_back(i);
        } else if (st.v_yaw < 0 && delta_angle > -la) {
          candidate_indices.push_back(i);
        }
      }

      if (!candidate_indices.empty()) {
        // 选择距离最近的可射击装甲板
        float min_dist = INFINITY;
        for (int ci : candidate_indices) {
          float dist = tar_position[ci].x * tar_position[ci].x +
                       tar_position[ci].y * tar_position[ci].y;
          if (dist < min_dist) {
            min_dist = dist;
            idx = ci;
          }
        }
        // 找到了 coming 阶段的装甲板，直接使用
        *aim_z = tar_position[idx].z + st.vzw * timeDelay;
        *aim_x = tar_position[idx].x + st.vxw * timeDelay;
        *aim_y = tar_position[idx].y + st.vyw * timeDelay;
        float temp_pitch = -pitchTrajectoryCompensation(
          sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
          *aim_z + st.z_bias, st.current_v);
        if (temp_pitch) *pitch = temp_pitch;
        if (*aim_x || *aim_y) *yaw = (float)(atan2(*aim_y, *aim_x));
        return 1;
      }
    }

    // 非小陀螺/未找到coming装甲板：使用加权距离+角度决策
    float min = INFINITY;
    float yaw_to_shooter = atan2f(st.yw, st.xw);
    for (int i = 0; i < 4; i++) {
      if (fabsf(tar_position[i].yaw - yaw_to_shooter) > 0.1 * PI) {
        continue;
      }

      const float dist =
        tar_position[i].x * tar_position[i].x + tar_position[i].y * tar_position[i].y;
      const float yaw_diff = fabsf(*yaw - tar_position[i].yaw);
      if (dist * 0.5 + yaw_diff * 0.5 < min) {
        min = dist * 0.5 + yaw_diff * 0.5;
        idx = i;
      }
    }
    if (min == INFINITY) {
      return 0;
    }
  }

  *aim_z = tar_position[idx].z + st.vzw * timeDelay;
  *aim_x = tar_position[idx].x + st.vxw * timeDelay;
  *aim_y = tar_position[idx].y + st.vyw * timeDelay;
  //这里符号给错了
  float temp_pitch = -pitchTrajectoryCompensation(
    sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias, *aim_z + st.z_bias, st.current_v);
  if (temp_pitch) *pitch = temp_pitch;
  if (*aim_x || *aim_y) *yaw = (float)(atan2(*aim_y, *aim_x));
  return 1;
}

// 从坐标轴正向看向原点，逆时针方向为正
