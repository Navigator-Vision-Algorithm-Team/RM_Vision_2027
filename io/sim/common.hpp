#ifndef IO__SIM__COMMON_HPP
#define IO__SIM__COMMON_HPP

// 数据源抽象层公共定义：模式枚举、统一单调时钟、YAML 辅助、
// 异常注入配置与注入器、各数据源的配置结构。
// 说明：本文件不依赖任何硬件 SDK，可被任意入口/测试引用。

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdint>
#include <random>
#include <string>

namespace io::sim
{

// ============================ 数据源模式 ============================
enum class SourceMode
{
  Real,   // 真实硬件（串口 / 海康等相机）
  Fake,   // 伪数据（内部生成，可注入异常）
  Replay  // 回放录制数据
};

inline SourceMode parse_source_mode(const std::string & s, SourceMode def)
{
  if (s == "real") return SourceMode::Real;
  if (s == "fake") return SourceMode::Fake;
  if (s == "replay") return SourceMode::Replay;
  return def;
}

inline const char * to_string(SourceMode m)
{
  switch (m) {
    case SourceMode::Real:
      return "real";
    case SourceMode::Fake:
      return "fake";
    case SourceMode::Replay:
      return "replay";
  }
  return "unknown";
}

// 日志标签：用于区分 真实数据 / 伪数据 / 回放数据
inline const char * log_tag(SourceMode m)
{
  switch (m) {
    case SourceMode::Real:
      return "REAL";
    case SourceMode::Fake:
      return "FAKE";
    case SourceMode::Replay:
      return "REPLAY";
  }
  return "UNKNOWN";
}

// ============================ 统一单调时钟 ============================
// 所有数据源、记录器、回放器共享同一单调时钟起点，保证串口与相机时间轴可对齐。
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

class SessionClock
{
public:
  static SessionClock & instance()
  {
    static SessionClock c;
    return c;
  }

  static TimePoint start_time() { return instance().start_; }

  static uint64_t now_rel_ns()
  {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   SteadyClock::now() - start_time())
                                   .count());
  }

  static TimePoint from_rel_ns(uint64_t ns)
  {
    return start_time() + std::chrono::nanoseconds(ns);
  }

private:
  SessionClock() : start_(SteadyClock::now()) {}
  TimePoint start_;
};

// ============================ YAML 辅助 ============================
inline bool node_has(const YAML::Node & y, const std::string & key)
{
  return static_cast<bool>(y) && static_cast<bool>(y[key]);
}

template <typename T>
inline T node_as(const YAML::Node & y, const std::string & key, const T & def)
{
  try {
    if (node_has(y, key)) return y[key].as<T>();
  } catch (...) {
  }
  return def;
}

// 依次尝试多个 key，返回第一个命中的值（用于 sursign 专属键 → 全局键回退）
template <typename T>
inline T node_as_any(
  const YAML::Node & y, const std::string & k1, const std::string & k2, const T & def)
{
  if (node_has(y, k1)) return node_as<T>(y, k1, def);
  if (!k2.empty() && node_has(y, k2)) return node_as<T>(y, k2, def);
  return def;
}

// ============================ 异常注入配置 ============================
struct FaultConfig
{
  // —— 串口 / IMU ——
  double imu_drop_rate = 0.0;         // 丢包概率 [0,1]
  double imu_delay_ms = 0.0;          // 额外延迟 (ms)
  int imu_spike_every = 0;            // 每 N 帧注入一次跳变 (0=关闭)
  double imu_spike_rad = 1.5708;      // 跳变幅度 (rad)
  double imu_corrupt_crc_rate = 0.0;  // 校验错误概率 [0,1]
  double imu_noise_rad = 0.0;         // 角度噪声标准差 (rad)

  // —— 相机 ——
  double camera_drop_rate = 0.0;        // 丢帧概率
  double camera_duplicate_rate = 0.0;   // 重复帧概率
  double camera_black_rate = 0.0;       // 黑屏概率
  double camera_overexpose_rate = 0.0;  // 过曝概率
  double camera_freeze_rate = 0.0;      // 冻结(重复上一帧)概率
  double camera_noise_sigma = 0.0;      // 像素噪声标准差
  double camera_delay_ms = 0.0;         // 帧延迟 (ms)

  bool enabled() const
  {
    return imu_drop_rate > 0 || imu_delay_ms > 0 || imu_spike_every > 0 ||
           imu_corrupt_crc_rate > 0 || imu_noise_rad > 0 || camera_drop_rate > 0 ||
           camera_duplicate_rate > 0 || camera_black_rate > 0 || camera_overexpose_rate > 0 ||
           camera_freeze_rate > 0 || camera_noise_sigma > 0 || camera_delay_ms > 0;
  }

  static FaultConfig load(const YAML::Node & yaml)
  {
    FaultConfig c;
    auto f = yaml["fault"];
    if (!static_cast<bool>(f)) return c;

    c.imu_drop_rate = node_as<double>(f, "imu_drop_rate", c.imu_drop_rate);
    c.imu_delay_ms = node_as<double>(f, "imu_delay_ms", c.imu_delay_ms);
    c.imu_spike_every = node_as<int>(f, "imu_spike_every", c.imu_spike_every);
    c.imu_spike_rad = node_as<double>(f, "imu_spike_rad", c.imu_spike_rad);
    c.imu_corrupt_crc_rate = node_as<double>(f, "imu_corrupt_crc_rate", c.imu_corrupt_crc_rate);
    c.imu_noise_rad = node_as<double>(f, "imu_noise_rad", c.imu_noise_rad);

    c.camera_drop_rate = node_as<double>(f, "camera_drop_rate", c.camera_drop_rate);
    c.camera_duplicate_rate = node_as<double>(f, "camera_duplicate_rate", c.camera_duplicate_rate);
    c.camera_black_rate = node_as<double>(f, "camera_black_rate", c.camera_black_rate);
    c.camera_overexpose_rate = node_as<double>(f, "camera_overexpose_rate", c.camera_overexpose_rate);
    c.camera_freeze_rate = node_as<double>(f, "camera_freeze_rate", c.camera_freeze_rate);
    c.camera_noise_sigma = node_as<double>(f, "camera_noise_sigma", c.camera_noise_sigma);
    c.camera_delay_ms = node_as<double>(f, "camera_delay_ms", c.camera_delay_ms);
    return c;
  }
};

// ============================ 故障注入器 ============================
// 使用固定种子，保证同一配置下异常序列可复现。
class FaultInjector
{
public:
  FaultInjector(const FaultConfig & cfg, int seed) : cfg_(cfg), rng_(static_cast<uint32_t>(seed)) {}

  const FaultConfig & config() const { return cfg_; }

  bool chance(double p)
  {
    if (p <= 0.0) return false;
    if (p >= 1.0) return true;
    return std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < p;
  }

  double gauss() { return normal_(rng_); }

private:
  FaultConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
};

// ============================ 各数据源配置 ============================
struct SerialSourceConfig
{
  SourceMode mode = SourceMode::Real;
  std::string port = "/dev/ttyACM0";
  uint32_t baud = 921600;
  // fake 模式下：
  //   external=true  → 使用外部进程（Python 虚拟下位机）创建的 pty，port 指向该 pts
  //   external=false → 使用 C++ 内置虚拟下位机生成真实协议字节
  bool external = false;
  double imu_hz = 500.0;        // 四元数(0x502)发送频率
  double chassis_hz = 50.0;     // 底盘速度(0x503)频率
  double status_hz = 1.0;       // 比赛状态(0x0001)频率
  double reset_period_s = 0.0;  // 复位信号周期(0=不注入)
  std::string replay_file;      // replay 模式：串口原始字节记录文件

  static SerialSourceConfig load(const YAML::Node & yaml, SourceMode fallback)
  {
    SerialSourceConfig c;
    c.mode = parse_source_mode(node_as<std::string>(yaml, "serial_mode", ""), fallback);
    c.port = node_as<std::string>(yaml, "com_port", c.port);
    c.baud = node_as<uint32_t>(yaml, "baud_rate", c.baud);
    c.external = node_as<bool>(yaml, "fake_serial_external", c.external);
    c.imu_hz = node_as<double>(yaml, "fake_imu_hz", c.imu_hz);
    c.chassis_hz = node_as<double>(yaml, "fake_chassis_hz", c.chassis_hz);
    c.status_hz = node_as<double>(yaml, "fake_status_hz", c.status_hz);
    c.reset_period_s = node_as<double>(yaml, "fake_reset_period_s", c.reset_period_s);
    c.replay_file = node_as<std::string>(
      yaml, "serial_replay_file", std::string("logs/virtual_session/serial.bin"));
    return c;
  }
};

struct CameraSourceConfig
{
  SourceMode mode = SourceMode::Fake;
  std::string kind = "synth";  // synth | sequence | video
  std::string dir;             // sequence 模式的图片目录
  std::string video;           // video 模式的视频文件
  int width = 1440;
  int height = 1080;
  double fps = 30.0;
  int pool = 4;            // fake 模式帧缓冲池大小（>=4 即可支撑 300fps 零拷贝产帧）
  std::string replay_dir;  // replay 模式：含 camera.tsv 的会话目录

  static CameraSourceConfig load(
    const YAML::Node & yaml, SourceMode fallback, const std::string & sursign = "")
  {
    CameraSourceConfig c;
    c.mode = parse_source_mode(node_as<std::string>(yaml, "camera_mode", ""), fallback);

    const std::string sfx = sursign.empty() ? "" : ("_" + sursign);
    c.kind = node_as_any<std::string>(yaml, "fake_camera_source" + sfx, "fake_camera_source", c.kind);
    c.dir = node_as_any<std::string>(yaml, "fake_camera_dir" + sfx, "fake_camera_dir", c.dir);
    c.video = node_as_any<std::string>(yaml, "fake_camera_video" + sfx, "fake_camera_video", c.video);
    c.width = node_as_any<int>(yaml, "camera_width" + sfx, "camera_width", c.width);
    c.height = node_as_any<int>(yaml, "camera_height" + sfx, "camera_height", c.height);
    c.fps = node_as_any<double>(yaml, "camera_frame_rate" + sfx, "camera_frame_rate", c.fps);
    c.pool = node_as_any<int>(yaml, "fake_camera_pool" + sfx, "fake_camera_pool", c.pool);
    c.replay_dir = node_as<std::string>(
      yaml, "camera_replay_dir", std::string("logs/virtual_session"));
    return c;
  }
};

struct RecordConfig
{
  bool enable = false;
  std::string dir = "logs/virtual_session";
  bool save_images = true;

  static RecordConfig load(const YAML::Node & yaml)
  {
    RecordConfig c;
    auto r = yaml["record"];
    if (!static_cast<bool>(r)) return c;
    c.enable = node_as<bool>(r, "enable", c.enable);
    c.dir = node_as<std::string>(r, "dir", c.dir);
    c.save_images = node_as<bool>(r, "save_images", c.save_images);
    return c;
  }
};

struct SimConfig
{
  SourceMode mode = SourceMode::Real;  // 总开关（各源可单独覆盖）
  int seed = 20261015;
  SerialSourceConfig serial;
  CameraSourceConfig camera;
  FaultConfig fault;
  RecordConfig record;

  static SimConfig load(const YAML::Node & yaml)
  {
    SimConfig c;
    c.mode = parse_source_mode(node_as<std::string>(yaml, "source_mode", "real"), SourceMode::Real);
    c.seed = node_as<int>(yaml, "fault_seed", c.seed);
    c.serial = SerialSourceConfig::load(yaml, c.mode);
    c.camera = CameraSourceConfig::load(yaml, c.mode);
    c.fault = FaultConfig::load(yaml);
    c.record = RecordConfig::load(yaml);
    return c;
  }
};

}  // namespace io::sim

#endif  // IO__SIM__COMMON_HPP
