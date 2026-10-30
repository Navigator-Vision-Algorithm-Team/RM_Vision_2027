#ifndef AUTO_AIM__DEBUG_STREAM_HPP
#define AUTO_AIM__DEBUG_STREAM_HPP

#include <Eigen/Dense>
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

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

  // ---- 以下为 IMM/EKF 的预测信息，由业务侧按需填充 ----
  // has_ekf=false 时面板与 JSON 不输出这一块（例如 soldier 纯 PnP 链路）
  bool has_ekf = false;
  Eigen::Vector3d target_center = Eigen::Vector3d::Zero();  // 目标旋转中心（世界系，m）
  Eigen::Vector2d target_vel = Eigen::Vector2d::Zero();     // 目标速度（世界系，m/s）
  double target_w = 0.0;                                    // 角速度（rad/s）
  double target_r = 0.0;                                    // 估计半径（m）
  Eigen::Vector3d model_prob = Eigen::Vector3d::Zero();     // IMM 三个模型的概率
  double nis_ratio = 0.0;                                   // NIS/dof，理想 1
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
//   image_raw     sensor_msgs/CompressedImage  相机原图（可选）
//   yolo_input    sensor_msgs/CompressedImage  YOLO 预处理后的输入图（可选）
//   status        std_msgs/String              JSON：FPS / 推理耗时 / 云台位姿 / 目标位姿
//   gimbal_pose   geometry_msgs/PoseStamped    当前云台位姿（世界系）
//   target_pose   geometry_msgs/PoseStamped    下发给云台的目标位姿（世界系）
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
