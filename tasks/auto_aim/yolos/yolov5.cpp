#include "yolov5.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>

#include "tasks/auto_aim/ov_device.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
YOLOV5::YOLOV5(const std::string & config_path, bool debug, bool single_thread_cpu)
: debug_(debug), single_thread_cpu_(single_thread_cpu), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolov5_model_path"].as<std::string>();
  device_ = device_from_config(yaml["device"] ? yaml["device"].as<std::string>() : "auto");
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);
  auto model = core_.read_model(model_path_);

  // 输入尺寸以 IR 的静态 shape 为准（NCHW）。以前这里和 detect() 的 letterbox
  // 各写死一份 640：模型换成 1280×960（全向 2×2 拼接用）时，两处就会和模型实际
  // shape 对不上，OpenVINO 在 ppp.build() 抛一个很难读的异常。
  {
    const auto shape = model->input().get_partial_shape();
    const bool valid = !shape.rank().is_dynamic() && shape.rank().get_length() == 4 &&
                       !shape.is_dynamic() && shape[1].get_length() == 3;
    if (!valid) {
      throw std::runtime_error(fmt::format(
        "[yolov5] 模型输入必须是静态 NCHW 1x3xHxW（rank=4、通道=3），实际不是：{}。"
        "请用固定 imgsz 重新导出（例如全向 2×2 拼接用 imgsz=960,1280）",
        model_path_));
    }
    input_h_ = static_cast<int>(shape[2].get_length());
    input_w_ = static_cast<int>(shape[3].get_length());
    tools::logger()->info(
      "[yolov5] 模型输入 {}×{}（来自 {}），letterbox 目标同步为该尺寸", input_w_, input_h_,
      model_path_);
  }

  // PrePostProcessor 的 tensor shape 是 NHWC（业务侧直接喂 BGR 的 cv::Mat），
  // 与模型自身的 NCHW 通过下面两行 layout 声明自动转换。
  const ov::Shape nhwc_shape{
    1, static_cast<size_t>(input_h_), static_cast<size_t>(input_w_), 3};

  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape(nhwc_shape)
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  model = ppp.build();
  compiled_model_ = compile_ov_model(
    core_, model, device_, ov::hint::PerformanceMode::LATENCY, single_thread_cpu_ ? 1 : 0);
  infer_request_ = compiled_model_.create_infer_request();
}

std::list<Armor> YOLOV5::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) {  // -1 表示该维度不裁切
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) {  // -1 表示该维度不裁切
      roi_.height = raw_img.rows;
    }

    // ROI 可能越界：配置里的 roi 是按主相机分辨率写的，而同一份配置也可能被
    // 拼接大图（全向融合）复用。越界裁剪会抛异常，这里先收缩到图像内，并把
    // offset 同步成实际裁剪原点，否则 parse() 还原坐标会整体偏移。
    roi_ &= cv::Rect(0, 0, raw_img.cols, raw_img.rows);
    offset_ = roi_.tl();
    if (roi_.width <= 0 || roi_.height <= 0) {
      tools::logger()->warn(
        "[yolov5] ROI 与图像 {}x{} 无交集，本帧跳过", raw_img.cols, raw_img.rows);
      return std::list<Armor>();
    }

    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  // letterbox 目标 = 模型真实输入尺寸（input_w_ × input_h_），不再写死 640
  auto x_scale = static_cast<double>(input_h_) / bgr_img.rows;
  auto y_scale = static_cast<double>(input_w_) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // preproces：复用输入缓冲，只覆盖 ROI，其余保持黑色（letterbox 填充）
  if (input_.rows != input_h_ || input_.cols != input_w_ || input_.type() != CV_8UC3) {
    input_ = cv::Mat(input_h_, input_w_, CV_8UC3);
  }
  input_.setTo(cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input_(roi), {w, h});
  ov::Tensor input_tensor(
    ov::element::u8,
    {1, static_cast<size_t>(input_h_), static_cast<size_t>(input_w_), 3}, input_.data);

  // infer
  infer_request_.set_input_tensor(input_tensor);
  infer_request_.infer();

  // postprocess
  auto output_tensor = infer_request_.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  return parse(scale, output, raw_img, frame_count);
}

/* 
  这个地方将会通过解析来从yolo的输出中得到数据。
  yolo的输出是【1， 25200， 22】
  这意味着yolo会输出25200个候选，而其中一个候选会有22个浮点数来表明一些数据。
  1. 0 - 7: 目标框的坐标信息 (x1, y1, x2, y2)
  2. 8: 置信度
  3. 9 - 12: 颜色
  4. 13 - 21: 类别
*/
std::list<Armor> YOLOV5::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // for each row: xywh + classess
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);

    if (score < score_threshold_) continue;

    std::vector<cv::Point2f> armor_key_points;

    //颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     //color
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  //num
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    // 至此到这个地方，我们的_class_id 都有着0 to 7，也就是八个类别，但是我们是需要3个，1， 3， 哨兵，所以我们需要在一个地方过滤，而这个地方是刚拿到yolo的结果，并且处理的地方，我们在这个地方做一次过滤是最合理的。
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;

    // 过滤掉不想要的类别
    if (_class_id != 1 && _class_id != 3 && _class_id != 0) {
      continue;
    }

    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (int i = 1; i < armor_key_points.size(); i++) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi_) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    // 使用传统方法二次矫正角点
    if (use_traditional_) detector_.detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

bool YOLOV5::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于神经网络的迭代
  // if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool YOLOV5::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  // 保存异常的图案，用于神经网络的迭代
  // if (!name_ok) save(armor);

  return name_ok;
}

cv::Point2f YOLOV5::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {
    std::clamp(center.x / w, 0.0f, 1.0f),
    std::clamp(center.y / h, 0.0f, 1.0f)};
}

void YOLOV5::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);
  // cv::imshow("detection", detection);  // headless-safe: use recorder instead
}

void YOLOV5::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, ARMOR_NAMES[armor.name], file_name);
  cv::imwrite(img_path, tmp_img_);
}

double YOLOV5::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::list<Armor> YOLOV5::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim