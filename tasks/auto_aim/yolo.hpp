#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

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
  // 【并发契约】底层各实现复用同一个 ov::InferRequest 与一块输入缓冲，因此
  // **一个实例只能被一个线程使用**。当前所有构造点都满足该约束：
  //   · YoloPipeline 每路相机在各自线程内构造独立实例；
  //   · 测试入口单线程使用。
  // 若将来需要多线程共享同一实例，必须自行加锁或改为每线程一份；
  // 早先那层内建互斥只在单实例内串行，对真正的跨实例并发毫无作用，已移除。
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP