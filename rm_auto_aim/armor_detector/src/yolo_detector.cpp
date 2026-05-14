#include "armor_detector/yolo_detector.hpp"

#include <algorithm>
#include <vector>

namespace rm_auto_aim
{

YOLODetector::YOLODetector(
  const std::string & model_path, const std::string & device, float score_threshold,
  float nms_threshold, int input_size)
: use_roi_(false),
  input_size_(input_size),
  score_threshold_(score_threshold),
  nms_threshold_(nms_threshold)
{
  auto model = core_.read_model(model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, input_size_, input_size_, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0f);

  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

void YOLODetector::setROI(const cv::Rect & roi)
{
  roi_ = roi;
  offset_ = cv::Point2f(roi.x, roi.y);
  use_roi_ = true;
}

std::vector<Armor> YOLODetector::detect(const cv::Mat & raw_img)
{
  if (raw_img.empty()) return {};

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) roi_.width = raw_img.cols;
    if (roi_.height == -1) roi_.height = raw_img.rows;
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  auto x_scale = static_cast<double>(input_size_) / bgr_img.rows;
  auto y_scale = static_cast<double>(input_size_) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // Preprocess: letterbox resize
  auto input = cv::Mat(input_size_, input_size_, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  ov::Tensor input_tensor(ov::element::u8, {1, input_size_, input_size_, 3}, input.data);

  // Infer
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // Postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  return parse(scale, output, raw_img);
}

void YOLODetector::sort_keypoints(std::vector<cv::Point2f> & keypoints)
{
  if (keypoints.size() != 4) return;

  std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
  std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

  std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });
  std::sort(bottom_points.begin(), bottom_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });

  keypoints[0] = top_points[0];      // top-left
  keypoints[1] = top_points[1];      // top-right
  keypoints[2] = bottom_points[1];   // bottom-right
  keypoints[3] = bottom_points[0];   // bottom-left
}

std::vector<Armor> YOLODetector::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img)
{
  cv::Mat transposed;
  cv::transpose(output, transposed);

  std::vector<int> ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_keypoints;

  for (int r = 0; r < transposed.rows; r++) {
    auto scores = transposed.row(r).colRange(4, 4 + class_num_);

    double score;
    cv::Point max_point;
    cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

    if (score < score_threshold_) continue;

    auto xywh = transposed.row(r).colRange(0, 4);
    auto x = xywh.at<float>(0);
    auto y = xywh.at<float>(1);
    auto w = xywh.at<float>(2);
    auto h = xywh.at<float>(3);
    auto left = static_cast<int>((x - 0.5 * w) / scale);
    auto top = static_cast<int>((y - 0.5 * h) / scale);
    auto width = static_cast<int>(w / scale);
    auto height = static_cast<int>(h / scale);

    // Extract 8 keypoint values (4 points x 2 coordinates)
    std::vector<cv::Point2f> keypoints;
    int kp_start = 4 + class_num_;
    for (int i = 0; i < 4 && (kp_start + i * 2 + 1) < transposed.cols; i++) {
      float kx = transposed.row(r).at<float>(kp_start + i * 2) / scale;
      float ky = transposed.row(r).at<float>(kp_start + i * 2 + 1) / scale;
      keypoints.emplace_back(kx, ky);
    }

    if (keypoints.size() != 4) continue;

    ids.emplace_back(max_point.x);
    confidences.emplace_back(static_cast<float>(score));
    boxes.emplace_back(left, top, width, height);
    armors_keypoints.emplace_back(keypoints);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::vector<Armor> armors;
  for (const auto & i : indices) {
    auto & keypoints = armors_keypoints[i];
    sort_keypoints(keypoints);

    if (use_roi_) {
      for (auto & kp : keypoints) {
        kp.x += offset_.x;
        kp.y += offset_.y;
      }
    }

    Armor armor;
    armor.points = keypoints;
    armor.center = (keypoints[0] + keypoints[1] + keypoints[2] + keypoints[3]) / 4.0;
    armor.confidence = confidences[i];

    float armor_width = cv::norm(keypoints[0] - keypoints[1]);
    armor.type = armor_width > 40 ? ArmorType::LARGE : ArmorType::SMALL;

    armors.emplace_back(armor);
  }

  return armors;
}

}  // namespace rm_auto_aim
