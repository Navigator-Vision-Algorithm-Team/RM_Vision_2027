// =============================================================================
//  裁判状态转发自测：串口帧 -> SerialBoard 解析 -> ROS 话题 -> 导航决策节点输入
//
//  链路：
//    内置虚拟下位机(fake) 发 0x0001(比赛状态) / 0x0201(机器人状态)
//      -> SerialBoard::referee_state()      解析 + 有效位
//      -> io::Publish2Nav                   5 Hz 发布
//         /red_standard_robot1/referee/{game_progress,current_hp,maximum_hp}
//      -> 本程序自带的订阅节点              校验字段值与导航侧期望一致
//
//  用法：./referee_forward_test [config] [timeout_s]
//        config 默认 configs/virtual_referee.yaml，timeout_s 默认 5
//  退出码：0 = 通过，1 = 失败
// =============================================================================

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "io/ros2/publish2nav.hpp"
#include "io/serial/serial_board.hpp"
#include "tools/yaml.hpp"

namespace
{

// 话题名必须与 io/ros2/publish2nav.cpp、导航决策节点（相对名 referee/xxx
// + 命名空间 red_standard_robot1）三者一致；不一致时本测试直接失败。
constexpr char kGameProgressTopic[] = "/red_standard_robot1/referee/game_progress";
constexpr char kCurrentHpTopic[] = "/red_standard_robot1/referee/current_hp";
constexpr char kMaximumHpTopic[] = "/red_standard_robot1/referee/maximum_hp";

/* 订阅侧：只记录三个话题最近一次收到的值与收包次数 */
class Checker : public rclcpp::Node
{
public:
  Checker() : rclcpp::Node("referee_forward_test_checker")
  {
    game_progress_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      kGameProgressTopic, 10,
      [this](std_msgs::msg::UInt8::ConstSharedPtr msg) {
        game_progress_.store(msg->data);
        ++game_progress_count_;
      });

    current_hp_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      kCurrentHpTopic, 10,
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        current_hp_.store(msg->data);
        ++current_hp_count_;
      });

    maximum_hp_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      kMaximumHpTopic, 10,
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        maximum_hp_.store(msg->data);
        ++maximum_hp_count_;
      });
  }

  uint8_t game_progress() const { return game_progress_.load(); }
  int32_t current_hp() const { return current_hp_.load(); }
  int32_t maximum_hp() const { return maximum_hp_.load(); }

  bool all_received() const
  {
    return game_progress_count_ > 0 && current_hp_count_ > 0 && maximum_hp_count_ > 0;
  }

  int counts_total() const
  {
    return game_progress_count_.load() + current_hp_count_.load() + maximum_hp_count_.load();
  }

private:
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr game_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_hp_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr maximum_hp_sub_;

  std::atomic<uint8_t> game_progress_{0};
  std::atomic<int32_t> current_hp_{0};
  std::atomic<int32_t> maximum_hp_{0};

  std::atomic<int> game_progress_count_{0};
  std::atomic<int> current_hp_count_{0};
  std::atomic<int> maximum_hp_count_{0};
};

int fail(const char * what)
{
  std::printf("[FAIL] %s\n", what);
  return 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string config = argc > 1 ? argv[1] : "configs/virtual_referee.yaml";
  const double timeout_s = argc > 2 ? std::atof(argv[2]) : 5.0;

  // 期望值从同一份配置推导，避免测试里硬编码数字
  bool test_nav = false;           // true = referee_state() 被固定值覆盖
  uint16_t expect_max_hp = 400;    // 期望对外发布的 maximum_hp
  uint16_t expect_hp = 400;        // 期望对外发布的 current_hp
  uint16_t raw_max_hp = 400;       // 虚拟下位机真实上报的 max_hp
  uint16_t raw_hp = 400;           // 虚拟下位机真实上报的 remain_hp
  try {
    const auto yaml = tools::load(config);
    raw_max_hp = yaml["fake_max_hp"].as<uint16_t>(400);
    raw_hp = yaml["fake_hp"].as<uint16_t>(raw_max_hp);
    test_nav = yaml["test_nav"].as<bool>(false);
    if (test_nav) {
      // 开关打开：对外发布的是 test_nav_* 固定值，与下位机上报无关
      expect_max_hp = yaml["test_nav_max_hp"].as<uint16_t>(400);
      expect_hp = yaml["test_nav_hp"].as<uint16_t>(expect_max_hp);
    } else {
      expect_max_hp = raw_max_hp;
      expect_hp = raw_hp;
    }
  } catch (const std::exception & e) {
    std::printf("[FAIL] 读取配置 %s 失败: %s\n", config.c_str(), e.what());
    return 1;
  }
  const int32_t expected_hp = expect_hp > expect_max_hp ? expect_max_hp : expect_hp;

  std::printf("== referee forward test ==\n");
  std::printf("config      : %s\n", config.c_str());
  std::printf(
    "mode        : %s\n",
    test_nav ? "test_nav=true（强制开赛 + 默认血量，忽略下位机）" : "test_nav=false（透传下位机帧）");
  std::printf("expect      : game_progress=4 current_hp=%d maximum_hp=%d\n", expected_hp,
              static_cast<int>(expect_max_hp));
  if (test_nav) {
    std::printf("mcu reports : current_hp=%u maximum_hp=%u (应被覆盖)\n", raw_hp, raw_max_hp);
  }

  rclcpp::init(argc, argv);

  // 1) 串口侧：虚拟下位机 -> SerialBoard 解析
  //    即使 test_nav 会覆盖对外结果，也要等真实帧到达，才能验证「覆盖前确实收到了真实值」
  io::SerialBoard board(config);
  const auto parse_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  io::RefereeState state;
  io::RobotStatus raw{};
  while (std::chrono::steady_clock::now() < parse_deadline) {
    state = board.referee_state();
    raw = board.robot_status();
    if (state.game_status_valid && state.robot_status_valid && raw.max_hp != 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::printf("[raw   ] 0x0201 hp=%u/%u\n", raw.remain_hp, raw.max_hp);
  std::printf("[parse ] game=%u(%d) hp=%d/%d\n", state.game_progress,
              static_cast<int>(state.game_status_valid), state.current_hp, state.maximum_hp);

  // 下位机上报必须被正确解析（test_nav 只覆盖对外结果，不掩盖解析失败）
  if (raw.max_hp != raw_max_hp || raw.remain_hp != raw_hp) {
    return fail("SerialBoard 未按配置解析出 0x0201 的血量");
  }
  if (!state.game_status_valid) return fail("SerialBoard 未解析到 0x0001（game_status_valid=false）");
  if (!state.robot_status_valid) return fail("SerialBoard 未解析到 0x0201（robot_status_valid=false）");
  if (state.game_progress != 4) return fail("game_progress 应为 4（虚拟下位机固定发比赛中）");
  if (state.current_hp != expected_hp || state.maximum_hp != static_cast<int32_t>(expect_max_hp)) {
    return fail("referee_state() 对外结果与配置不一致");
  }
  if (test_nav && (state.current_hp == static_cast<int32_t>(raw.remain_hp)) &&
      (raw.remain_hp != expected_hp)) {
    return fail("test_nav=true 时对外血量没有覆盖下位机上报");
  }

  // 2) ROS 侧：Publish2Nav 发布 + 本程序订阅校验
  auto publisher = std::make_shared<io::Publish2Nav>();
  publisher->set_referee_source([&board]() { return board.referee_state(); });
  auto checker = std::make_shared<Checker>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(checker);

  const auto ros_deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < ros_deadline && !checker->all_received()) {
    executor.spin_once(std::chrono::milliseconds(50));
  }
  std::printf("[ros   ] game=%u hp=%d max=%d total_msgs=%d\n", checker->game_progress(),
              checker->current_hp(), checker->maximum_hp(), checker->counts_total());

  if (!checker->all_received()) return fail("三个 referee 话题未全部收到");
  if (checker->game_progress() != 4) return fail("game_progress 话题值错误");
  if (checker->current_hp() != expected_hp) return fail("current_hp 话题值错误");
  if (checker->maximum_hp() != static_cast<int32_t>(expect_max_hp)) {
    return fail("maximum_hp 话题值错误");
  }

  // 3) 再跑 0.5 s，确认是周期发布而不是只发一次
  const int first_count = checker->counts_total();
  const auto burst_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < burst_deadline) {
    executor.spin_once(std::chrono::milliseconds(50));
  }
  const int second_count = checker->counts_total();
  std::printf("[rate  ] msgs %d -> %d in 500 ms\n", first_count, second_count);
  if (second_count <= first_count) return fail("话题没有持续发布（订阅方无法持续获得状态）");

  executor.cancel();
  std::printf("[PASS] 串口裁判帧 -> ROS 话题链路正常\n");

  rclcpp::shutdown();
  return 0;
}
