#include "rm_omniperception/omni_perceptron_node.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include <chrono>
#include <functional>
#include <string>

namespace rm_omniperception
{

OmniPerceptronNode::OmniPerceptronNode(const rclcpp::NodeOptions & options)
: Node("omni_perceptron", options)
{
  RCLCPP_INFO(get_logger(), "Starting OmniPerceptronNode (all-directional perception)");

  // Declare parameters
  int num_cameras = declare_parameter("num_omni_cameras", 4);
  declare_parameter("enemy_color", 0);  // 0=RED, 1=BLUE
  declare_parameter("priority_mode", 1);  // 1=mode1, 2=mode2
  declare_parameter("fov_h", 1.047);     // horizontal FOV (rad, default 60deg)
  declare_parameter("fov_v", 0.785);     // vertical FOV (rad, default 45deg)
  declare_parameter("process_rate", 50.0); // Hz for main processing loop

  // Configure decider
  decider_.setEnemyColor(get_parameter("enemy_color").as_int());
  decider_.setFOV(get_parameter("fov_h").as_double(), get_parameter("fov_v").as_double());
  decider_.setPriorityMode(
    get_parameter("priority_mode").as_int() == 2
      ? PriorityMode::MODE_TWO : PriorityMode::MODE_ONE);

  // Setup camera slots (unique_ptr to avoid mutex copy/move issues)
  const std::vector<std::string> default_labels = {
    "left_0", "right_0", "left_1", "right_1"
  };

  for (int i = 0; i < num_cameras && i < 4; ++i) {
    auto cam = std::make_unique<CameraSlot>();
    cam->label = declare_parameter(
      "camera_label_" + std::to_string(i), default_labels[i]);
    std::string topic_base = declare_parameter(
      "camera_topic_" + std::to_string(i),
      "/omni_camera_" + std::to_string(i));

    // Camera info subscription
    cam->info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
      topic_base + "/camera_info", rclcpp::SensorDataQoS(),
      [this, i](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        infoCallback(i, msg);
      });

    // Image subscription
    cam->image_sub = create_subscription<sensor_msgs::msg::Image>(
      topic_base + "/image_raw", rclcpp::SensorDataQoS(),
      [this, i](sensor_msgs::msg::Image::SharedPtr msg) {
        imageCallback(i, msg);
      });

    RCLCPP_INFO(get_logger(), "Camera %d (%s) on topic: %s",
      i, cam->label.c_str(), topic_base.c_str());

    cameras_.push_back(std::move(cam));
  }

  // Subscriptions for nav/referee integration
  invincible_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/omni/invincible_ids", 10,
    std::bind(&OmniPerceptronNode::invincibleCallback, this, std::placeholders::_1));

  aim_target_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/omni/aim_target_ids", 10,
    std::bind(&OmniPerceptronNode::aimTargetCallback, this, std::placeholders::_1));

  // Publisher for omni-directional target
  omni_target_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/omni/target_direction", 10);

  // Process timer (main thread drains queue + decides)
  double process_period = 1.0 / get_parameter("process_rate").as_double();
  process_timer_ = create_wall_timer(
    std::chrono::duration<double>(process_period),
    std::bind(&OmniPerceptronNode::processLoop, this));

  // Spawn inference threads
  setupDetectors();
  for (int i = 0; i < static_cast<int>(cameras_.size()); ++i) {
    infer_threads_.emplace_back(&OmniPerceptronNode::inferLoop, this, i);
  }

  RCLCPP_INFO(get_logger(), "OmniPerceptron initialized with %zu cameras and %zu infer threads",
    cameras_.size(), infer_threads_.size());
}

OmniPerceptronNode::~OmniPerceptronNode()
{
  // Signal stop
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_flag_ = true;
  }
  stop_cv_.notify_all();
  detection_queue_.stop();

  // Join inference threads
  for (auto & t : infer_threads_) {
    if (t.joinable()) t.join();
  }
}

void OmniPerceptronNode::setupDetectors()
{
  detectors_.resize(cameras_.size());
  for (size_t i = 0; i < cameras_.size(); ++i) {
    // Create lightweight traditional detector per thread
    // (YOLO not available without OpenVINO)
    rm_auto_aim::Detector::LightParams l_params{0.1, 0.4, 40.0};
    rm_auto_aim::Detector::ArmorParams a_params{0.7, 0.8, 3.2, 3.2, 5.5, 35.0};
    detectors_[i] = std::make_unique<rm_auto_aim::Detector>(
      80, get_parameter("enemy_color").as_int(), l_params, a_params);
  }
}

void OmniPerceptronNode::imageCallback(
  size_t slot_idx, const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (slot_idx >= cameras_.size()) return;
  auto & cam = *cameras_[slot_idx];

  try {
    auto cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
    std::lock_guard<std::mutex> lock(cam.frame_mutex);
    cv_ptr->image.copyTo(cam.latest_frame);
    cam.has_frame = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
      "cv_bridge error for camera %zu: %s", slot_idx, e.what());
  }
}

void OmniPerceptronNode::infoCallback(
  size_t slot_idx, const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (slot_idx >= cameras_.size()) return;
  auto & cam = *cameras_[slot_idx];

  cam.camera_matrix = cv::Mat(3, 3, CV_64F,
    const_cast<double *>(msg->k.data())).clone();
  cam.dist_coeffs = cv::Mat(1, 5, CV_64F,
    const_cast<double *>(msg->d.data())).clone();

  decider_.setImageSize(msg->width, msg->height);

  // Unsubscribe after first camera_info received
  if (cam.info_sub) {
    cam.info_sub.reset();
  }
}

void OmniPerceptronNode::invincibleCallback(
  const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  decider_.set_invincible_armors(msg->data);
}

void OmniPerceptronNode::aimTargetCallback(
  const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  // Stored for future use by tracker integration
  (void)msg;
}

void OmniPerceptronNode::inferLoop(size_t slot_idx)
{
  RCLCPP_INFO(get_logger(), "Infer thread %zu started", slot_idx);

  auto & cam = *cameras_[slot_idx];
  auto & detector = detectors_[slot_idx];

  while (rclcpp::ok()) {
    // Check stop flag
    {
      std::lock_guard<std::mutex> lock(stop_mutex_);
      if (stop_flag_) break;
    }

    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(cam.frame_mutex);
      if (!cam.has_frame) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      cam.latest_frame.copyTo(frame);
    }

    if (frame.empty()) continue;

    // Run detection
    auto armors = detector->detect(frame);

    if (!armors.empty()) {
      // Compute angle offsets
      std::list<rm_auto_aim::Armor> armor_list(armors.begin(), armors.end());
      Eigen::Vector2d offsets = decider_.delta_angle(armor_list, cam.label);

      DetectionResult result(armor_list, std::chrono::steady_clock::now(),
                             offsets.x(), offsets.y());
      detection_queue_.push(std::move(result));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  RCLCPP_INFO(get_logger(), "Infer thread %zu stopped", slot_idx);
}

void OmniPerceptronNode::processLoop()
{
  // Drain all detection results from inference threads
  std::vector<DetectionResult> all_results;
  detection_queue_.drain(all_results);

  if (all_results.empty()) return;

  // Filter and sort
  decider_.sort(all_results);

  if (all_results.empty()) return;

  // Publish best target direction
  const auto & best = all_results.front();
  if (!best.armors.empty()) {
    geometry_msgs::msg::Vector3Stamped out;
    out.header.stamp = now();
    out.header.frame_id = "gimbal_link";
    out.vector.x = best.delta_yaw;
    out.vector.y = best.delta_pitch;
    out.vector.z = best.armors.front().confidence;
    omni_target_pub_->publish(out);
  }
}

}  // namespace rm_omniperception

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_omniperception::OmniPerceptronNode)
