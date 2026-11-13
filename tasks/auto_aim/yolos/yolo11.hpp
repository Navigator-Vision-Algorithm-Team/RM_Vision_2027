#ifndef AUTO_AIM__YOLO11_HPP
#define AUTO_AIM__YOLO11_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{
class YOLO11 : public YOLOBase
{
public:
  YOLO11(const std::string & config_path, bool debug, bool single_thread_cpu = false);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

  // 640×640 写死在 detect() 的 letterbox 与 PrePostProcessor 里，与本文件的字面量保持一致。
  cv::Size input_size() const override { return {640, 640}; }

private:
  std::string device_, model_path_;
  std::string save_path_, debug_path_;
  bool debug_, use_roi_;
  bool single_thread_cpu_ = false;

  const int class_num_ = 38;
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  double min_confidence_, binary_threshold_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;
  // 复用推理请求与输入缓冲：create_infer_request() 每次都要分配内部张量/队列，
  // 640x640x3 的输入缓冲每次重新分配同样多余。
  // 【前提】同一实例同一时刻只有一个线程调用 detect()：
  //   YoloPipeline 每路相机一个实例；若走 YOLO 包装类，yolo_lock（默认 true）
  //   会串行化。详见 tasks/auto_aim/yolo.hpp 的并发契约。
  // 若将来需要多线程共用同一实例，请改回每帧 create_infer_request()。
  ov::InferRequest infer_request_;
  cv::Mat input_;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;

  Detector detector_;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
};

}  // namespace auto_aim

#endif  //AUTO_AIM__YOLO11_HPP