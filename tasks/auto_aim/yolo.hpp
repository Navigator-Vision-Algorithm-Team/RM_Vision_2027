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

private:
  std::unique_ptr<YOLOBase> yolo_;
  // 多相机推理池（multithread::YoloPipeline）里每个实例只被自己的线程调用，
  // 不存在并发；但保留这把锁是为了兼容“多路共用一个实例”的旧用法，
  // 因为底层 OpenVINO 的 InferRequest 不允许并发调用。
  // 单相机场景可用配置 yolo_lock: false 关掉锁，省掉一次加解锁。
  //
  // 注意：这里必须用 unique_ptr 而不是直接放 std::mutex ——
  // std::mutex 不可拷贝也不可移动，直接作为成员会让 YOLO 变成不可移动类型。
  std::unique_ptr<std::mutex> mutex_ = std::make_unique<std::mutex>();
  bool lock_enabled_ = true;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP