#ifndef ARMOR_DETECTOR__CLASSIFIER_HPP_
#define ARMOR_DETECTOR__CLASSIFIER_HPP_

#include <opencv2/opencv.hpp>
#include <string>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{

class ArmorClassifier
{
public:
  ArmorClassifier(
    const std::string & model_path, const std::string & label_path, double threshold,
    const std::vector<std::string> & ignore_classes);

  void classify(Armor & armor);

  double threshold;

private:
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__CLASSIFIER_HPP_
