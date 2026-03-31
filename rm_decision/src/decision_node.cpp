#include "rm_decision/decision_node.hpp"
#include <rclcpp_action/create_client.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace rm_decision
{
  // ====== 目标点坐标 — 按实际场地修改 ======
  const TargetPoint CENTER_POINT = {4.5, -4.0, 0.0, 0.0, 0.0, 0.0, 1.0}; // 进攻点
  const TargetPoint HOME_POINT = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};   // 出生点/补血点

  DecisionNode::DecisionNode(const rclcpp::NodeOptions &options)
      : Node("decision_node", options)
  {
    this->declare_parameter("hp_threshold", 200.0);
    this->declare_parameter("hp_recovery_ratio", 0.90);
    this->declare_parameter("nav_action_name", "navigate_to_pose");
    this->declare_parameter("nav_action_fallback", "/navigate_to_pose");
    hp_threshold_ = this->get_parameter("hp_threshold").as_double();
    hp_recovery_ratio_ = this->get_parameter("hp_recovery_ratio").as_double();
    nav_action_name_ = this->get_parameter("nav_action_name").as_string();
    nav_action_fallback_name_ = this->get_parameter("nav_action_fallback").as_string();

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action_name_);
    if (!nav_action_fallback_name_.empty() && nav_action_fallback_name_ != nav_action_name_)
    {
      nav_client_fallback_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action_fallback_name_);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅: game_status → remap 到 /serial/game_status
    game_status_sub_ = this->create_subscription<auto_aim_interfaces::msg::GameStatus>(
        "game_status", 10,
        std::bind(&DecisionNode::gameStatusCallback, this, std::placeholders::_1));

    // 订阅: robot_status → remap 到 /serial/game_robot_status
    robot_status_sub_ = this->create_subscription<auto_aim_interfaces::msg::GameRobotStatus>(
        "robot_status", 10,
        std::bind(&DecisionNode::robotStatusCallback, this, std::placeholders::_1));

    strategy_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DecisionNode::strategyTimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "Decision node started | hp_threshold=%.0f | recovery=%.0f%% | nav_action=%s | nav_fallback=%s",
                hp_threshold_, hp_recovery_ratio_ * 100.0,
                nav_action_name_.c_str(),
                nav_action_fallback_name_.empty() ? "<none>" : nav_action_fallback_name_.c_str());
  }

  void DecisionNode::gameStatusCallback(const auto_aim_interfaces::msg::GameStatus::SharedPtr msg)
  {
    current_time_ = msg->stage_remain_time;
    game_progress_ = msg->game_progress;
  }

  void DecisionNode::robotStatusCallback(const auto_aim_interfaces::msg::GameRobotStatus::SharedPtr msg)
  {
    current_hp_ = msg->remain_hp;
    max_hp_ = msg->max_hp;
  }

  void DecisionNode::strategyTimerCallback()
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[Decision] progress=%u | time=%us | HP=%u/%u | healing=%s",
                         game_progress_, current_time_, current_hp_, max_hp_,
                         is_healing_ ? "Y" : "N");

    // 只在比赛中行动
    if (game_progress_ != 4)
      return;

    // 低血量 → 补血
    if (current_hp_ > 0 && current_hp_ < static_cast<uint16_t>(hp_threshold_) && !is_healing_)
    {
      RCLCPP_WARN(this->get_logger(), "HP low (%u < %.0f) -> retreat", current_hp_, hp_threshold_);
      is_healing_ = true;
      last_goal_location_.clear();
    }

    // 血量恢复 → 进攻
    if (is_healing_)
    {
      uint16_t recovery_hp = static_cast<uint16_t>(max_hp_ * hp_recovery_ratio_);
      if (current_hp_ >= recovery_hp)
      {
        RCLCPP_INFO(this->get_logger(), "HP recovered (%u >= %u) -> attack", current_hp_, recovery_hp);
        is_healing_ = false;
        last_goal_location_.clear();
      }
    }

    // 执行
    if (is_healing_)
    {
      if (getDistanceToPoint(HOME_POINT) > 1.0)
        sendGoal("home", HOME_POINT);
    }
    else
    {
      sendGoal("center", CENTER_POINT);
    }
  }

  double DecisionNode::getDistanceToPoint(const TargetPoint &point)
  {
    try
    {
      auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      double dx = point.x - t.transform.translation.x;
      double dy = point.y - t.transform.translation.y;
      return std::sqrt(dx * dx + dy * dy);
    }
    catch (const tf2::TransformException &)
    {
      return 999.0;
    }
  }

  void DecisionNode::sendGoal(const std::string &location_name, const TargetPoint &point)
  {
    if (location_name == last_goal_location_)
      return; // 防抖

    auto active_client = nav_client_;
    const char *active_action_name = nav_action_name_.c_str();

    if (!active_client->action_server_is_ready())
    {
      if (nav_client_fallback_ && nav_client_fallback_->action_server_is_ready())
      {
        active_client = nav_client_fallback_;
        active_action_name = nav_action_fallback_name_.c_str();
      }
      else
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "Nav2 not ready, can't go to %s (tried: %s%s%s)",
                             location_name.c_str(),
                             nav_action_name_.c_str(),
                             nav_action_fallback_name_.empty() ? "" : ", ",
                             nav_action_fallback_name_.empty() ? "" : nav_action_fallback_name_.c_str());
        return;
      }
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = point.x;
    goal_msg.pose.pose.position.y = point.y;
    goal_msg.pose.pose.position.z = point.z;
    goal_msg.pose.pose.orientation.x = point.qx;
    goal_msg.pose.pose.orientation.y = point.qy;
    goal_msg.pose.pose.orientation.z = point.qz;
    goal_msg.pose.pose.orientation.w = point.qw;

    active_client->async_send_goal(goal_msg);
    RCLCPP_WARN(this->get_logger(), ">> Nav goal: %s (%.2f, %.2f) via %s",
                location_name.c_str(), point.x, point.y, active_action_name);
    last_goal_location_ = location_name;
  }

  void DecisionNode::cancelCurrentGoal()
  {
    if (nav_client_->action_server_is_ready())
      nav_client_->async_cancel_all_goals();
    if (nav_client_fallback_ && nav_client_fallback_->action_server_is_ready())
      nav_client_fallback_->async_cancel_all_goals();
    last_goal_location_.clear();
  }

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_decision::DecisionNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
