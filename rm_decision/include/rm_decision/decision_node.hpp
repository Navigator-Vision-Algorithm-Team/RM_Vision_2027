#ifndef RM_DECISION_NODE_HPP_
#define RM_DECISION_NODE_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "auto_aim_interfaces/msg/game_status.hpp"
#include "auto_aim_interfaces/msg/game_robot_status.hpp"

namespace rm_decision
{
    struct TargetPoint
    {
        double x, y, z;        // 位置
        double qx, qy, qz, qw; // 朝向（四元数）
    };

    class DecisionNode : public rclcpp::Node
    {
    public:
        explicit DecisionNode(const rclcpp::NodeOptions &options);

    private:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_fallback_;
        std::string nav_action_name_;
        std::string nav_action_fallback_name_;

        // 用于获取机器人当前位置
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // ── 订阅者 ──
        rclcpp::Subscription<auto_aim_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
        rclcpp::Subscription<auto_aim_interfaces::msg::GameRobotStatus>::SharedPtr robot_status_sub_;

        // ── 定时器 ──
        rclcpp::TimerBase::SharedPtr strategy_timer_;

        // ── 回调函数 ──
        void gameStatusCallback(const auto_aim_interfaces::msg::GameStatus::SharedPtr msg);
        void robotStatusCallback(const auto_aim_interfaces::msg::GameRobotStatus::SharedPtr msg);
        void strategyTimerCallback();

        // ── 辅助函数 ──
        void sendGoal(const std::string &location_name, const TargetPoint &point);
        void cancelCurrentGoal();
        double getDistanceToPoint(const TargetPoint &point);

        // ── 状态变量 ──
        uint16_t current_time_{0};       // 比赛剩余时间 (秒)
        uint8_t game_progress_{0};       // 比赛阶段 (4=比赛中)
        uint16_t current_hp_{0};         // 当前血量
        uint16_t max_hp_{600};           // 最大血量 (默认哨兵600)
        bool is_healing_{false};         // 补血模式标志
        bool nav_server_online_{false};  // Nav2 Action Server 是否可用
        std::string last_goal_location_; // 上一次发送的目标地点名称
        double hp_threshold_;            // 血量阈值 (低于此值回家补血)
        double hp_recovery_ratio_;       // 血量恢复比例 (高于此比例切回进攻)
    };

}

#endif
