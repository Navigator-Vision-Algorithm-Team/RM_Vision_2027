#ifndef AUTO_AIM__DETECTOR_HPP
#define AUTO_AIM__DETECTOR_HPP

#include <cstddef>
#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "classifier.hpp"

namespace auto_aim
{

class Detector
{
public:
  // key_prefix：参数前缀。非空时优先读取 "<key_prefix><键名>"（例如 "omni_threshold"），
  // 该键不存在则回退到通用键名。全向相机用它单独调参，不影响主相机。
  Detector(
    const std::string & config_path, bool debug = true,
    const std::string & key_prefix = std::string());

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count = -1);

  bool detect(Armor & armor, const cv::Mat & bgr_img);

  friend class YOLOV8;

private:
  Classifier classifier_;

  std::string key_prefix_;

  // 全图检测的资源上限：轮廓数量在没有限制时会被反光/噪点推到几百个，后续两两
  // 配对是 O(n²)，每个候选还要跑一次分类器网络。全向 4 路并发时必须设上限。
  std::size_t max_lightbars_ = 32;          // 单帧保留的最长灯条数量
  std::size_t max_armor_candidates_ = 12;   // 送入分类器的最多候选数
  bool save_patterns_ = true;               // 低置信/异常图案是否写盘（全向默认关）

  double threshold_;
  double max_angle_error_;
  double min_lightbar_ratio_, max_lightbar_ratio_;
  double min_lightbar_length_;
  double min_armor_ratio_, max_armor_ratio_;
  double max_side_ratio_;
  double min_confidence_;
  double max_rectangular_error_;

  bool debug_;
  std::string save_path_;

  // 利用PCA回归角点，参考自https://github.com/CSU-FYT-Vision/FYT2024_vision
  void lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const;

  bool check_geometry(const Lightbar & lightbar) const;
  bool check_geometry(const Armor & armor) const;
  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  Color get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const;
  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;
  ArmorType get_type(const Armor & armor);
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  void save(const Armor & armor) const;
  void show_result(
    const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
    const std::list<Armor> & armors, int frame_count) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__DETECTOR_HPP