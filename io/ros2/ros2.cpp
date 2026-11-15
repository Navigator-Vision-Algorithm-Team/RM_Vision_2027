/*
 * ROS 2 门面实现：把节点与 spin 线程的创建/销毁全部收拢在构造与析构里。
 *
 * 数据流：
 *   发布侧：上层（sentry 主循环、SerialBoard 接收线程）→ Publish2Nav（单节点 + 单 spin 线程）
 *          → 自瞄目标位姿 / 云台关节 / 裁判状态三类话题；
 *   订阅侧：导航侧 Twist → Subscribe2Nav → nav_sender 回调（收到即下发串口，与主循环解耦）
 *          + 内部队列（供主循环用 subscribe_get_data() 拉取）。
 *
 * 线程：本类自建 publish_spin_thread_ 与 subscribe_spin_thread_ 两条线程并在析构时 join；
 *       gimbal_joint_sender() 返回的回调不在本类线程里执行，而是在 SerialBoard 的
 *       串口接收线程里被调用。
 *
 * 注意：功能 / 输入 / 输出等契约写在 io/ros2/ros2.hpp，本文件只写实现要点。
 */
#include "ros2.hpp"
namespace io
{
// 实现要点：rclcpp::init 必须在最前，后续所有节点对象都依赖它。
// 发布侧无条件创建：云台关节反馈与裁判状态转发都挂在发布节点上，即使不订阅导航也要有它。
// 两条线程各自 spin 各自的节点，互不阻塞——串口 500Hz 的关节回调不会拖住宿主循环的订阅侧。
ROS2::ROS2(std::function<void(const NavCommand &)> nav_sender, bool subscribe_nav)
{
  rclcpp::init(0, nullptr);

  publish2nav_ = std::make_shared<Publish2Nav>();

  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });

  if (subscribe_nav) {
    subscribe2nav_ = std::make_shared<Subscribe2Nav>(std::move(nav_sender));

    subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });
  }
}

// 实现要点：先 shutdown 再 join。顺序反过来会永久阻塞——spin 循环只有 shutdown 才会退出，
// 此时 join 等的是一条永远不会结束的线程。
ROS2::~ROS2()
{
  rclcpp::shutdown();
  if (publish_spin_thread_ && publish_spin_thread_->joinable()) publish_spin_thread_->join();
  if (subscribe_spin_thread_ && subscribe_spin_thread_->joinable()) subscribe_spin_thread_->join();
}

// 实现要点：直接转给已有发布节点，不额外起节点或线程（裁判帧只有 1~5 Hz）。
// publish2nav_ 由构造函数保证非空，判空只为防止对象被移动/误用后解引用空指针。
void ROS2::set_referee_source(Publish2Nav::RefereeSource source)
{
  // 复用已有的发布节点与 spin 线程：裁判状态频率低，没必要再起一个节点
  if (!publish2nav_) return;
  publish2nav_->set_referee_source(std::move(source));
}

// 实现要点：这里只做转调，位姿含义与坐标变换都在调用方（Solver/Aimer）完成。
void ROS2::publish(const Eigen::Vector4d & target_pos) { publish2nav_->send_data(target_pos); }

// 实现要点：按值捕获发布节点的 shared_ptr 而不是捕获 this——
// 回调常常被串口接收线程持有到 ROS2 析构之后，捕获 shared_ptr 可保证节点本身仍存活；
// 再用 rclcpp::ok() 兜住 shutdown 之后的最后一次调用，否则 publish 会抛异常。
std::function<void(double, double)> ROS2::gimbal_joint_sender()
{
  auto node = publish2nav_;
  return [node](double yaw_rad, double pitch_rad) {
    // 本对象析构时 rclcpp 已 shutdown，串口接收线程可能还在收尾，
    // 此时直接跳过（否则 publish 会抛异常）。
    if (!rclcpp::ok()) return;
    node->publish_gimbal_joint_state(yaw_rad, pitch_rad);
  };
}

// 实现要点：subscribe_nav=false 时没有订阅侧，返回默认构造的 NavCommand（全零 = 原地不动），
// 让上层不必再判空。
NavCommand ROS2::subscribe_get_data()
{
  if (!subscribe2nav_) return NavCommand();  // subscribe_nav=false 时没有订阅侧
  return subscribe2nav_->subscribe_move_info();
}

}  // namespace io
