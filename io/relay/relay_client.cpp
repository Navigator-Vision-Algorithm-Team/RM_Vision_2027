/*
 * 视觉侧客户端实现：一条接收线程同时负责"收上行"与"发下行"。
 *
 * 数据流：iceoryx2 订阅 imu/robot/game → 加锁更新快照与样本队列；
 *         send() 只把待发指令放进 pending 槽，由接收线程在下一拍发布，
 *         因此本对象不持有任何跨线程的 iceoryx2 句柄，实现上也不需要 pimpl。
 *
 * 注意：契约见 relay_client.hpp，本文件只写实现要点。
 */
#include "io/relay/relay_client.hpp"

#include <cmath>
#include <utility>

#include "io/relay/relay_msgs.hpp"
#include "iox2/iceoryx2.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
// 姿态样本队列上限：500Hz 下约 0.1s，足够覆盖图像处理延迟且不会无限增长
constexpr size_t kSampleCapacity = 64;

// 功能：把 steady_clock 时间点转成纳秒
// 输入：
//   tp  时间点
// 输出：自 epoch 起的纳秒数
// 备注：与 relay 侧使用同一内核时钟
int64_t to_ns(std::chrono::steady_clock::time_point tp)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}

// 功能：借共享内存样本发布一帧
// 输入：
//   pub      发布者
//   payload  载荷
// 输出：无
// 备注：队列满时静默丢弃，避免阻塞主循环
template <typename Publisher, typename Payload>
void publish(Publisher & pub, const Payload & payload)
{
  auto sample = pub.loan_uninit();
  if (!sample.has_value()) return;
  (void)iox2::send(std::move(sample.value().write_payload(payload)));
}
}  // namespace

RelayClient::RelayClient(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  bullet_speed = yaml["bullet_speed"] ? yaml["bullet_speed"].as<double>() : 28.0;
  if (yaml["relay_timeout_ms"]) timeout_ms_ = yaml["relay_timeout_ms"].as<int>();
  prefix_ = yaml["relay_prefix"] ? yaml["relay_prefix"].as<std::string>()
                                 : info_relay::msg::kDefaultPrefix;

  thread_ = std::thread(&RelayClient::receive_loop, this);
}

RelayClient::~RelayClient()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
}

// 实现要点：所有 iceoryx2 对象都是本函数内的局部变量，随线程生命周期存在；
// 任一步初始化失败都只打日志并让线程空转，不影响视觉主链路启动。
void RelayClient::receive_loop()
{
  using namespace iox2;

  // 主题前缀与 relay 必须一致，构造期已从配置读入
  const std::string prefix = prefix_;

  auto node_exp = NodeBuilder().create<ServiceType::Ipc>();
  if (!node_exp.has_value()) {
    tools::logger()->error("[RelayClient] 无法创建 iceoryx2 节点，链路不可用");
    return;
  }
  auto & node = node_exp.value();

  auto imu_name = ServiceName::create((prefix + "/" + info_relay::msg::kTopicImu).c_str());
  auto rb_name = ServiceName::create((prefix + "/" + info_relay::msg::kTopicRobot).c_str());
  auto gm_name = ServiceName::create((prefix + "/" + info_relay::msg::kTopicGame).c_str());
  auto aim_name = ServiceName::create((prefix + "/" + info_relay::msg::kTopicCmdAim).c_str());
  auto nav_name = ServiceName::create((prefix + "/" + info_relay::msg::kTopicCmdNav).c_str());
  auto hl_name = ServiceName::create((prefix + "/" + info_relay::msg::kTopicHealth).c_str());
  if (
    !imu_name.has_value() || !rb_name.has_value() || !gm_name.has_value() ||
    !aim_name.has_value() || !nav_name.has_value() || !hl_name.has_value()) {
    tools::logger()->error("[RelayClient] 主题名非法，链路不可用");
    return;
  }

  auto imu_service =
    node.service_builder(imu_name.value()).publish_subscribe<info_relay::msg::Imu>().open_or_create();
  auto rb_service = node.service_builder(rb_name.value())
                      .publish_subscribe<info_relay::msg::Robot>()
                      .open_or_create();
  auto gm_service = node.service_builder(gm_name.value())
                      .publish_subscribe<info_relay::msg::Game>()
                      .open_or_create();
  auto aim_service = node.service_builder(aim_name.value())
                       .publish_subscribe<info_relay::msg::AimCmd>()
                       .open_or_create();
  auto nav_service = node.service_builder(nav_name.value())
                       .publish_subscribe<info_relay::msg::NavCmd>()
                       .open_or_create();
  auto hl_service = node.service_builder(hl_name.value())
                      .publish_subscribe<info_relay::msg::Health>()
                      .open_or_create();
  if (
    !imu_service.has_value() || !rb_service.has_value() || !gm_service.has_value() ||
    !aim_service.has_value() || !nav_service.has_value() || !hl_service.has_value()) {
    tools::logger()->error("[RelayClient] 创建主题失败，链路不可用");
    return;
  }

  auto imu_sub = imu_service.value().subscriber_builder().create();
  auto rb_sub = rb_service.value().subscriber_builder().create();
  auto gm_sub = gm_service.value().subscriber_builder().create();
  auto aim_pub = aim_service.value().publisher_builder().create();
  auto nav_pub = nav_service.value().publisher_builder().create();
  auto hl_sub = hl_service.value().subscriber_builder().create();
  if (
    !imu_sub.has_value() || !rb_sub.has_value() || !gm_sub.has_value() ||
    !aim_pub.has_value() || !nav_pub.has_value() || !hl_sub.has_value()) {
    tools::logger()->error("[RelayClient] 创建订阅/发布者失败，链路不可用");
    return;
  }

  tools::logger()->info("[RelayClient] 已连接主题前缀 {}", prefix);

  while (!quit_) {
    // ---- 上行：IMU 逐帧收，状态类只保留最新 ----
    auto imu = imu_sub.value().receive();
    while (imu.has_value() && imu.value().has_value()) {
      const auto & m = imu.value().value().payload();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ImuSample sample;
        sample.t_ns = static_cast<int64_t>(m.t_ns);
        sample.state.pitch = m.pitch_rad;
        sample.state.roll = m.roll_rad;
        sample.state.upper_yaw = m.yaw_rad;
        // 关节角还原：lower = yaw - upper_joint
        sample.state.lower_yaw = m.yaw_rad - m.upper_yaw_rad;
        imu_samples_.push_back(sample);
        if (imu_samples_.size() > kSampleCapacity) imu_samples_.pop_front();

        // 首个样本标志链路建立：现场排查时它比任何错误码都直接
        if (last_imu_ns_ == 0) tools::logger()->info("[RelayClient] 收到首个 IMU 样本，链路建立");
        last_imu_ns_ = sample.t_ns;
        // uptime 回落 = MCU 重启，与 relay 侧判定口径一致
        if (last_uptime_ms_ != 0 && m.uptime_ms < last_uptime_ms_) reset_pending_ = true;
        last_uptime_ms_ = m.uptime_ms;
      }
      imu = imu_sub.value().receive();
    }

    auto rb = rb_sub.value().receive();
    bool has_rb = false;
    info_relay::msg::Robot robot{};
    while (rb.has_value() && rb.value().has_value()) {
      robot = rb.value().value().payload();
      has_rb = true;
      rb = rb_sub.value().receive();
    }
    if (has_rb) {
      std::lock_guard<std::mutex> lock(mutex_);
      robot_id_ = robot.robot_id;
      own_red_ = robot.own_team_red == 2 ? std::nullopt
                                         : std::optional<bool>(robot.own_team_red == 1);
      barrel_heat_ = robot.barrel_heat_17mm;
      heat_limit_ = robot.heat_limit;
      referee_.current_hp = robot.current_hp;
      referee_.maximum_hp = robot.maximum_hp;
      referee_.robot_status_valid = (robot.valid_mask & 0x01) != 0;
      referee_.game_status_valid = (robot.valid_mask & 0x02) != 0;
    }

    auto gm = gm_sub.value().receive();
    bool has_gm = false;
    info_relay::msg::Game game{};
    while (gm.has_value() && gm.value().has_value()) {
      game = gm.value().value().payload();
      has_gm = true;
      gm = gm_sub.value().receive();
    }
    if (has_gm) {
      std::lock_guard<std::mutex> lock(mutex_);
      game_started_ = game.in_battle != 0;
      referee_.game_progress = game.game_progress;
    }

    // 契约版本握手：只认第一帧，之后不再重复比较（版本在一次运行内不会变）
    auto hl = hl_sub.value().receive();
    bool checked = false;
    while (hl.has_value() && hl.value().has_value()) {
      const auto version = hl.value().value().payload().schema_version;
      if (!checked) {
        checked = true;
        if (version != info_relay::msg::kSchemaVersion) {
          schema_ok_ = false;
          tools::logger()->error(
            "[RelayClient] 契约版本不一致：relay={} 视觉={}，数据不可信，链路判失效", version,
            info_relay::msg::kSchemaVersion);
        }
      }
      hl = hl_sub.value().receive();
    }

    // ---- 下行：把主循环塞进来的指令发出去 ----
    std::optional<Command> out_cmd;
    std::optional<NavCommand> out_nav;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_cmd_) {
        out_cmd = pending_cmd_;
        pending_cmd_.reset();
      }
      if (pending_nav_) {
        out_nav = pending_nav_;
        pending_nav_.reset();
      }
    }
    if (out_cmd) {
      info_relay::msg::AimCmd m{};
      m.t_ns = static_cast<uint64_t>(to_ns(std::chrono::steady_clock::now()));
      m.yaw_rad = static_cast<float>(out_cmd->yaw);
      m.pitch_rad = static_cast<float>(out_cmd->pitch);
      m.control = out_cmd->control ? 1 : 0;
      // 链路不健康时强制停火，收敛到安全侧
      m.shoot = (out_cmd->shoot && link_ok()) ? 1 : 0;
      publish(aim_pub.value(), m);
    }
    if (out_nav) {
      info_relay::msg::NavCmd m{};
      m.t_ns = static_cast<uint64_t>(to_ns(std::chrono::steady_clock::now()));
      m.vx = static_cast<float>(out_nav->vx);
      m.vy = static_cast<float>(out_nav->vy);
      m.wz = static_cast<float>(out_nav->wz);
      m.source = 1;  // 1 = 视觉进程
      publish(nav_pub.value(), m);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
}

// 实现要点：样本按时间戳升序入队，找到跨过目标时刻的一对做线性插值；
// 目标时刻早于首样本或晚于末样本时直接返回对应端的样本，不做外推。
ImuState RelayClient::interpolate_locked(int64_t t_ns) const
{
  if (imu_samples_.empty()) return ImuState{};
  if (t_ns <= imu_samples_.front().t_ns) return imu_samples_.front().state;
  if (t_ns >= imu_samples_.back().t_ns) return imu_samples_.back().state;

  for (size_t i = 1; i < imu_samples_.size(); ++i) {
    const auto & b = imu_samples_[i];
    if (b.t_ns < t_ns) continue;
    const auto & a = imu_samples_[i - 1];
    const double span = static_cast<double>(b.t_ns - a.t_ns);
    const double alpha = span > 0 ? static_cast<double>(t_ns - a.t_ns) / span : 0.0;

    ImuState out;
    out.pitch = a.state.pitch + alpha * (b.state.pitch - a.state.pitch);
    out.roll = a.state.roll + alpha * (b.state.roll - a.state.roll);
    out.upper_yaw = a.state.upper_yaw + alpha * (b.state.upper_yaw - a.state.upper_yaw);
    out.lower_yaw = a.state.lower_yaw + alpha * (b.state.lower_yaw - a.state.lower_yaw);
    return out;
  }
  return imu_samples_.back().state;
}

ImuState RelayClient::imu_state_at(std::chrono::steady_clock::time_point t)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return interpolate_locked(to_ns(t));
}

Eigen::Quaterniond RelayClient::imu_at(std::chrono::steady_clock::time_point t)
{
  const ImuState s = imu_state_at(t);
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(s.lower_yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(s.pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(s.roll, Eigen::Vector3d::UnitX()));
}

void RelayClient::send(Command command) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_cmd_ = command;
}

void RelayClient::send(NavCommand command) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_nav_ = command;
}

bool RelayClient::link_ok() const
{
  if (!schema_ok_) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_imu_ns_ == 0) return false;
  const int64_t now_ns = to_ns(std::chrono::steady_clock::now());
  return (now_ns - last_imu_ns_) < static_cast<int64_t>(timeout_ms_) * 1000000;
}

bool RelayClient::can_shoot() const
{
  if (!link_ok()) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (heat_limit_ == 0) return true;  // 没有裁判数据时不拦
  return static_cast<int>(barrel_heat_) < static_cast<int>(heat_limit_) - 2;
}

bool RelayClient::is_game_started() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return game_started_;
}

std::optional<bool> RelayClient::own_team_red() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return own_red_;
}

uint8_t RelayClient::robot_id() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_id_;
}

RefereeState RelayClient::referee_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return referee_;
}

uint16_t RelayClient::barrel_heat() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return barrel_heat_;
}

uint16_t RelayClient::heat_limit() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return heat_limit_;
}

bool RelayClient::reset_pending()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool pending = reset_pending_;
  reset_pending_ = false;
  return pending;
}

}  // namespace io
