#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

/*
 * ROS 2 门面：用一个对象收拢本进程全部 ROS 2 节点与 spin 线程，向上层提供四个接口。
 *
 * 数据流：详见 io/ros2/ros2.cpp 的文件头；本头文件只声明契约。
 *
 * 线程：构造时创建发布 / 订阅两条 spin 线程，析构时 join；
 *       gimbal_joint_sender() 返回的回调由 SerialBoard 的串口接收线程调用。
 *
 * 注意：1) 构造即 rclcpp::init、析构即 rclcpp::shutdown，全进程只允许存在一个实例；
 *       2) 必须比 tools::Exiter 先构造：rclcpp 会接管 SIGINT 处理，后安装的 handler
 *          才能让 Ctrl+C 正常置位退出标志（见 src/sentry_multithread.cpp）；
 *       3) 只有找到 ROS 2 依赖的目标才会编译本模块，其余目标不含。
 */

#include <functional>

#include "publish2nav.hpp"
#include "subscribe2nav.hpp"
#include "command.hpp"

namespace io
{
class ROS2
{
public:
  // 功能：初始化 ROS 2，创建发布节点（可选订阅节点）并各启动一条 spin 线程
  // 输入：
  //   nav_sender     收到导航速度后的发送回调（通常直接下发串口）；允许为空，
  //                  为空时只把数据放进队列，等主循环用 subscribe_get_data() 拉取
  //   subscribe_nav  true 创建订阅侧并订阅导航速度；false 只创建发布侧
  // 输出：无
  // 备注：内部调用 rclcpp::init，会接管进程的 SIGINT/SIGTERM 处理；
  //       构造后必须确保析构被调用，否则 spin 线程不会退出
  explicit ROS2(
    std::function<void(const NavCommand &)> nav_sender = nullptr, bool subscribe_nav = true);

  // 功能：关闭 ROS 2 并回收两条 spin 线程
  // 输入：无
  // 输出：无
  // 备注：先 shutdown 后 join；shutdown 之后仍可能有回调在其他线程里执行，
  //       回调内部以 rclcpp::ok() 自我短路
  ~ROS2();

  // 功能：发布自瞄目标位置（历史接口，std_msgs/String）
  // 输入：
  //   target_pos  目标位置四元组；各分量的语义由发布节点的序列化格式决定
  // 输出：无
  // 备注：非阻塞；发布节点由构造函数保证存在，故不做判空
  void publish(const Eigen::Vector4d & target_pos);

  // 功能：返回一个「发布云台关节状态」的回调，供串口层每收到一帧 0x0503 时调用
  // 输入：无
  // 输出：std::function，参数 yaw_rad / pitch_rad 单位 rad；回调在 SerialBoard 的
  //       串口接收线程里执行；pitch 无自由度时由调用方传 0
  // 备注：回调按值持有发布节点的 shared_ptr，因此 ROS2 对象先析构也不会悬空；
  //       rclcpp 已 shutdown 时回调直接返回，不做发布
  std::function<void(double yaw_rad, double pitch_rad)> gimbal_joint_sender();

  // 功能：启动裁判状态转发，由已有的发布节点按 5 Hz 拉取快照并发布三个标量话题
  // 输入：
  //   source  裁判状态数据源，返回 RefereeState 快照；通常包装 SerialBoard::referee_state，
  //           因此 SerialBoard 必须比本对象活得久（先构造、后析构）
  // 输出：无
  // 备注：复用的是发布节点与它的 spin 线程，不额外起节点；
  //       转发的是 referee/game_progress、referee/current_hp、referee/maximum_hp 三个话题，
  //       话题名带 red_standard_robot1 命名空间，换命名空间时此处与导航侧必须同步改；
  //       只允许设置一次，重复调用被忽略并告警；source 为空时不发布任何裁判话题
  void set_referee_source(Publish2Nav::RefereeSource source);

  // 功能：取走订阅侧收到的最新导航速度
  // 输入：无
  // 输出：NavCommand，vx / vy 单位 m/s、wz 单位 rad/s；未创建订阅侧或暂无数据时返回全零
  // 备注：非阻塞；数据取出后队列为空，调用频率应与主循环频率一致
  NavCommand subscribe_get_data();

  // 功能：新建一个独立节点并在后台 spin，返回该话题的发布者
  // 输入：
  //   node_name   节点名，需在 ROS 图中唯一
  //   topic_name  话题名
  //   queue_size  发布队列长度，溢出时丢最旧的样本
  // 输出：该话题的 Publisher 共享指针，调用方负责持有
  // 备注：为每个发布者新起一个 detached 线程且永不回收，只适合少量长期存在的发布者；
  //       主链路（目标位姿 / 关节 / 裁判）走 Publish2Nav，不经过这里
  template <typename T>
  std::shared_ptr<rclcpp::Publisher<T>> create_publisher(
    const std::string & node_name, const std::string & topic_name, size_t queue_size)
  {
    auto node = std::make_shared<rclcpp::Node>(node_name);

    auto publisher = node->create_publisher<T>(topic_name, queue_size);

    // 运行一个单独的线程来 spin 这个节点，确保消息可以被正确发布
    std::thread([node]() { rclcpp::spin(node); }).detach();

    return publisher;
  }

private:
  std::shared_ptr<Publish2Nav> publish2nav_;      // 发布节点；构造即创建，非空
  std::shared_ptr<Subscribe2Nav> subscribe2nav_;  // 订阅节点；subscribe_nav=false 时为空

  std::unique_ptr<std::thread> publish_spin_thread_;    // 发布节点 spin 线程，析构时 join
  std::unique_ptr<std::thread> subscribe_spin_thread_;  // 订阅节点 spin 线程，析构时 join
};

}  // namespace io
#endif  // IO__ROS2_HPP
