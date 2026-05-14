#ifndef RM_OMNIPERCEPTION__OMNI_PERCEPTRON_NODE_HPP_
#define RM_OMNIPERCEPTION__OMNI_PERCEPTRON_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector.hpp"
#include "rm_omniperception/decider.hpp"
#include "rm_omniperception/detection_result.hpp"
#include "rm_omniperception/thread_safe_queue.hpp"

namespace rm_omniperception
{

class OmniPerceptronNode : public rclcpp::Node
{
public:
  explicit OmniPerceptronNode(const rclcpp::NodeOptions & options);

  ~OmniPerceptronNode() override;

private:
  // Decider for priority/filter/fallback logic
  Decider decider_;

  // Thread-safe queue shared between inference threads and main thread
  tools::ThreadSafeQueue<DetectionResult> detection_queue_{10};

  // Inference threads (one per omni-directional camera)
  std::vector<std::thread> infer_threads_;
  bool stop_flag_ = false;
  mutable std::mutex stop_mutex_;
  std::condition_variable stop_cv_;

  // Per-camera subscriptions (unique_ptr to allow std::mutex member)
  struct CameraSlot
  {
    std::string label;
    cv::Mat latest_frame;
    std::mutex frame_mutex;
    bool has_frame = false;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
  };
  std::vector<std::unique_ptr<CameraSlot>> cameras_;

  // Default detector instances per thread (traditional contour-based, lightweight)
  // For YOLO, replace with YOLODetector when OpenVINO is available
  std::vector<std::unique_ptr<rm_auto_aim::Detector>> detectors_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    omni_target_pub_;  // omni-camera target direction (yaw, pitch, distance)

  // Subscriptions for nav integration
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
    invincible_sub_;  // invincible enemy IDs from referee
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
    aim_target_sub_;  // priority target IDs from navigation

  void setupCameras();
  void setupDetectors();

  // Image callbacks for each camera slot
  void imageCallback(size_t slot_idx, const sensor_msgs::msg::Image::SharedPtr msg);
  void infoCallback(size_t slot_idx, const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Invincible / aim target callbacks
  void invincibleCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void aimTargetCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  // Per-camera inference loop (runs in separate thread)
  void inferLoop(size_t slot_idx);

  // Main processing: drain queue, filter, sort, decide
  void processLoop();

  // Timer for main processing loop
  rclcpp::TimerBase::SharedPtr process_timer_;
};

}  // namespace rm_omniperception

#endif  // RM_OMNIPERCEPTION__OMNI_PERCEPTRON_NODE_HPP_
