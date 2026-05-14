#ifndef ARMOR_DETECTOR__YOLO_DETECTOR_HPP_
#define ARMOR_DETECTOR__YOLO_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#ifdef HAS_OPENVINO
#include <openvino/openvino.hpp>
#endif
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{

class YOLODetector
{
public:
  YOLODetector(
    const std::string & model_path, const std::string & device = "AUTO",
    float score_threshold = 0.7, float nms_threshold = 0.3, int input_size = 416);

  std::vector<Armor> detect(const cv::Mat & bgr_img);

  void setROI(const cv::Rect & roi);

private:
  bool use_roi_;
  cv::Rect roi_;
  cv::Point2f offset_;
  int input_size_;
  float score_threshold_;
  float nms_threshold_;
  int class_num_ = 2;

#ifdef HAS_OPENVINO
  ov::Core core_;
  ov::CompiledModel compiled_model_;
#endif

  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
  std::vector<Armor> parse(
    double scale, cv::Mat & output, const cv::Mat & bgr_img);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__YOLO_DETECTOR_HPP_
