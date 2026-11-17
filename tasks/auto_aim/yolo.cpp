#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

// 各后端由 CMake 依据配置里的 yolo_name 决定是否编译；
// 未启用的头文件与分支在预处理阶段整段消失，工厂函数不存在多余的运行期判断
#ifdef SP_WITH_YOLO11
#include "yolos/yolo11.hpp"
#endif
#ifdef SP_WITH_YOLOV5
#include "yolos/yolov5.hpp"
#endif
#ifdef SP_WITH_YOLOV8
#include "yolos/yolov8.hpp"
#endif

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug, bool single_thread_cpu)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

#ifdef SP_WITH_YOLOV8
  if (yolo_name == "yolov8") {
    yolo_ = std::make_unique<YOLOV8>(config_path, debug, single_thread_cpu);
  } else
#endif
#ifdef SP_WITH_YOLO11
    if (yolo_name == "yolo11") {
    yolo_ = std::make_unique<YOLO11>(config_path, debug, single_thread_cpu);
  } else
#endif
#ifdef SP_WITH_YOLOV5
    if (yolo_name == "yolov5") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug, single_thread_cpu);
  } else
#endif
  {
    // 走到这里说明运行期配置与编译期裁剪不一致（构建用的不是这份配置），直接报错
    throw std::runtime_error(
      "yolo_name 未在本次构建中编译：" + yolo_name +
      "。请用 -DSP_CONFIG= 指向该配置后重新构建。");
  }
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  // 不加锁：并发约定是「一个实例一个线程」，见 yolo.hpp 的说明
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim