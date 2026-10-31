#ifndef AUTO_AIM__DEBUG_STREAM_HPP
#define AUTO_AIM__DEBUG_STREAM_HPP

#include <Eigen/Dense>
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"

namespace auto_aim
{

// 一帧调试状态：只填「业务侧才知道」的量。
// FPS / 推理耗时由 DebugStream 自己统计，图像由 DebugStream 自己持有，调用方不必准备。
// 角度单位统一为弧度，且与下位机协议一致（世界系下 pitch 向上为负）。
struct DebugStatus
{
  Eigen::Vector3d gimbal_ypr = Eigen::Vector3d::Zero();  // 当前云台姿态 yaw/pitch/roll
  Eigen::Vector3d target_ypr = Eigen::Vector3d::Zero();  // 本帧下发给云台的目标 yaw/pitch
  bool control = false;                                  // 本帧是否真的下发了控制指令
  bool shoot = false;                                    // 本帧是否允许开火
  std::string state;                                     // 业务状态文本（tracker 状态等）
  std::string note;                                      // 额外一行文本，可留空

  // 【新增】EKF 遥测：只有带 tracker 的入口会填；valid=false 时不发布 ekf 话题
  struct Ekf
  {
    bool valid = false;
    double residual_yaw = 0.0;       // 观测残差（rad）
    double residual_pitch = 0.0;     // 观测残差（rad）
    double residual_distance = 0.0;  // 观测残差（m）
    double residual_angle = 0.0;     // 观测残差（rad）
    double nis = 0.0;                // 归一化新息平方
    double nis_failure_rate = 0.0;   // 最近窗口内 NIS 超阈值比例 0~1
    double w = 0.0;                  // 自转角速度估计（rad/s）
    double r = 0.0;                  // 第一组装甲板半径估计（m）
    double l = 0.0;                  // 半径差 r2-r1（m）
    double h = 0.0;                  // 高度差 z2-z1（m）
  } ekf;

  // 【新增】EKF 预测的投影像素：由调用方用 Solver::world2pixel 算好，DebugStream 只负责画
  bool has_aim = false;                             // aim_pixel 是否有效
  cv::Point2f aim_pixel;                            // 预测命中点（含发弹延迟 + 飞行时间）
  std::vector<cv::Point2f> predicted_armor_pixels;  // 当前时刻 4 块装甲板的预测位置
};

// ROS2 -> Foxglove 实时调试数据流。
//
// 设计约束（保证「只插入、不改变业务」）：
//   * 只读业务数据，不参与控制；任何异常都在内部吞掉并自动降级为空实现，
//     绝不打断自瞄主链路；
//   * 图像编码 + 发布放在独立线程，主线程每帧只多一次 clone（且受 image_hz 节流）；
//   * 编译时没有 ROS2 环境则整体退化为空实现，enabled() 恒为 false。
//
// 话题（默认，<ns>/<tag> 前缀）：
//   image         sensor_msgs/CompressedImage  原图 + YOLO 框 + 敌人编号 + FPS/位姿叠加
//                                              + PnP 一级校验（重投影点 / 残差 / 板面三轴，见 debug_stream.cpp）
//                                              + EKF 预测叠加（P0..P3 预测装甲板 / AIM 预测命中点）
//   image_raw     sensor_msgs/CompressedImage  相机原图（可选）
//   yolo_input    sensor_msgs/CompressedImage  YOLO 预处理后的输入图（可选）
//   status        std_msgs/String              JSON：FPS / 推理耗时 / 云台位姿 / 目标位姿
//   gimbal_pose   geometry_msgs/PoseStamped    当前云台位姿（世界系）
//   target_pose   geometry_msgs/PoseStamped    下发给云台的目标位姿（世界系）
//   ekf           std_msgs/Float64MultiArray   EKF 遥测（仅跟踪成功时发布），data 顺序：
//                 [0]residual_yaw [1]residual_pitch [2]residual_distance [3]residual_angle
//                 [4]nis [5]nis_failure_rate [6]w [7]r [8]l [9]h
class DebugStream
{
public:
  // config_path: 与业务相同的 yaml，读取其中的 debug_ros2 段（缺省则关闭）；
  // tag: 进程标识（如 "sentry" / "soldier"），同时用于节点名与话题前缀。
  DebugStream(const std::string & config_path, const std::string & tag);
  ~DebugStream();

  DebugStream(const DebugStream &) = delete;
  DebugStream & operator=(const DebugStream &) = delete;

  bool enabled() const;

  // 帧首调用（相机出图后、任何绘制之前）：按 image_hz 节流，被节流时不拷贝图像。
  void frame(const cv::Mat & raw_img);

  // 帧尾调用，必须与 frame() 成对：提交本帧检测结果与状态。
  void publish(const std::list<Armor> & armors, const DebugStatus & status);

  // YOLO 推理前后调用，用于统计单帧推理耗时与处理帧率（FPS）。
  void begin_yolo();
  void end_yolo();

  // 推理已由别的线程完成时直接上报耗时（多相机推理池 YoloPipeline），
  // 效果与 begin_yolo()/end_yolo() 成对调用一致。
  void yolo_sample(double infer_ms);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DEBUG_STREAM_HPP
