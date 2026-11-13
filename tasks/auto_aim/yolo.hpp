#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include <mutex>

#include "armor.hpp"

namespace auto_aim
{
class YOLOBase
{
public:
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;

  // 模型的真实输入尺寸（宽, 高）。letterbox 的目标尺寸必须以它为准：该尺寸由导出的
  // IR 静态 shape 决定，改配置改不动它，只能重新导出模型。全向拼接用它校验
  // 「大图 vs 输入」，避免在拼接侧再写死一份 640 而与模型对不上。
  virtual cv::Size input_size() const = 0;
};

class YOLO
{
public:
  // single_thread_cpu: 该实例独占一个线程/一颗核时置 true —— CPU 推理只开 1 个
  // 线程、1 个 stream。多相机场景下每路一个实例，不限制的话线程数会成倍膨胀。
  YOLO(const std::string & config_path, bool debug = true, bool single_thread_cpu = false);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

  // 底层模型的输入尺寸，转发给各 YOLO 实现（见 YOLOBase::input_size）。
  cv::Size input_size() const;

private:
  std::unique_ptr<YOLOBase> yolo_;
  // 多相机推理池（multithread::YoloPipeline）里每个实例只被自己的线程调用，
  // 不存在并发；但保留这把锁是为了兼容“多路共用一个实例”的旧用法。
  //
  // 【并发契约】底层各 YOLO 实现会复用一个成员 ov::InferRequest（以及一块输入
  // 缓冲）来避免每帧重建，这要求同一实例同一时刻只有一个线程在 detect()。
  //   · yolo_lock: true（默认）-> 本锁保证这一点；
  //   · yolo_lock: false       -> 调用方承诺「一个实例只在一个线程里用」。
  //     YoloPipeline 每路相机各建一个实例，满足该承诺；请勿在多路共享实例的
  //     同时关锁。关锁前请确认配置里没有其它代码复用同一个 YOLO 对象。
  //
  // 注意：这里必须用 unique_ptr 而不是直接放 std::mutex ——
  // std::mutex 不可拷贝也不可移动，直接作为成员会让 YOLO 变成不可移动类型。
  std::unique_ptr<std::mutex> mutex_ = std::make_unique<std::mutex>();
  bool lock_enabled_ = true;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP