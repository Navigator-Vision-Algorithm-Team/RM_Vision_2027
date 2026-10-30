// 虚拟数据源运行器：无需任何真实硬件即可跑通“四元数源 + 相机源”链路。
//
//   real   : 真实串口 + 真实相机
//   fake   : 内置虚拟下位机(真实协议字节) + 伪相机(合成/图片序列/视频)，可注入异常
//   replay : 回放录制好的串口原始字节 + 相机帧
//
// 业务侧只依赖 io::sim::QuaternionSource / io::sim::CameraSource 抽象接口。
// 用法: virtual_run configs/virtual_example.yaml [--show] [--duration=<秒>]
//       注意 cv::CommandLineParser 采用 --key=value 形式

#include <fmt/format.h>

#include <chrono>
#include <deque>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/sim/camera_source.hpp"
#include "io/sim/common.hpp"
#include "io/sim/quaternion_source.hpp"
#include "io/sim/recorder.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace
{

// 按时间戳对四元数做 slerp 插值，与图像帧对齐（与业务里 imu_at 同一套逻辑）
class QuaternionInterp
{
public:
  void push(const io::sim::QuaternionSample & s)
  {
    if (behind_.valid && behind_.timestamp > s.timestamp) return;
    ahead_ = behind_;
    behind_ = s;
  }

  Eigen::Quaterniond at(io::sim::TimePoint t) const
  {
    if (!behind_.valid) return Eigen::Quaterniond::Identity();
    if (!ahead_.valid || behind_.timestamp <= t) return behind_.q;

    const double ab = std::chrono::duration<double>(behind_.timestamp - ahead_.timestamp).count();
    if (ab <= 0.0) return behind_.q;
    double k = std::chrono::duration<double>(t - ahead_.timestamp).count() / ab;
    k = std::min(1.0, std::max(0.0, k));
    return ahead_.q.slerp(k, behind_.q).normalized();
  }

private:
  io::sim::QuaternionSample ahead_, behind_;
};

}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(
    argc, argv,
    "{help h usage ? | | 输出命令行参数说明}"
    "{@config-path   | configs/virtual_example.yaml | 位置参数，yaml配置文件路径 }"
    "{show           | | 显示图像窗口 }"
    "{duration       | 8.0 | 运行时长(秒)，用法 --duration=10 }");

  const auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  const bool show = cli.has("show");
  const double duration_s = cli.get<double>("duration");

  tools::Exiter exiter;

  const auto yaml = tools::load(config_path);
  auto sim = io::sim::SimConfig::load(yaml);

  // ---------------- 构建数据源（工厂按配置返回 real/fake/replay） ----------------
  auto imu = io::sim::make_quaternion_source(sim);
  auto cam = io::sim::make_camera_source(yaml, config_path, "main");

  // ---------------- 记录 ----------------
  std::shared_ptr<io::sim::SerialRecorder> serial_recorder;
  std::unique_ptr<io::sim::CameraRecorder> camera_recorder;
  if (sim.record.enable) {
    serial_recorder = std::make_shared<io::sim::SerialRecorder>(sim.record.dir);
    if (auto * sqs = dynamic_cast<io::sim::SerialQuaternionSource *>(imu.get()))
      sqs->attach_recorder(serial_recorder);
    camera_recorder = std::make_unique<io::sim::CameraRecorder>(sim.record.dir, sim.record.save_images);
  }

  imu->open();
  cam->open();

  tools::logger()->info(
    "================ [{}] virtual_run 启动 ================", io::sim::log_tag(imu->mode()));
  tools::logger()->info("QuaternionSource = {} (mode={})", imu->name(), io::sim::to_string(imu->mode()));
  tools::logger()->info("CameraSource     = {} (mode={})", cam->name(), io::sim::to_string(cam->mode()));

  // ---------------- 主循环 ----------------
  QuaternionInterp interp;
  io::sim::QuaternionSample qs;
  const auto t_start = io::sim::SteadyClock::now();

  long long frames = 0, timeouts = 0, injected_frames = 0, valid_q = 0;
  double max_frame_gap = 0.0;
  io::sim::TimePoint last_frame_ts{};

  while (!exiter.exit()) {
    const double elapsed = std::chrono::duration<double>(io::sim::SteadyClock::now() - t_start).count();
    if (elapsed >= duration_s) break;

    io::sim::Frame frame;
    if (!cam->read(frame, 200)) {
      timeouts++;
      continue;
    }
    frames++;
    if (frame.injected) injected_frames++;

    if (last_frame_ts != io::sim::TimePoint{}) {
      max_frame_gap = std::max(
        max_frame_gap,
        std::chrono::duration<double>(frame.timestamp - last_frame_ts).count());
    }
    last_frame_ts = frame.timestamp;

    // 抽干队列，保留最新四元数样本（非阻塞）
    while (imu->read(qs, 0)) {
      if (qs.valid) {
        interp.push(qs);
        valid_q++;
      }
    }

    const Eigen::Quaterniond q = interp.at(frame.timestamp);

    if (camera_recorder)
      camera_recorder->record(frame.data, frame.timestamp, frame.frame_id, frame.format);

    if (frames % 30 == 0) {
      const auto ypr = q.toRotationMatrix().eulerAngles(2, 1, 0);
      tools::logger()->info(
        "[{}] frame={} ts={:.3f}s {}x{} q=(w={:.3f}) yaw={:.2f}° {}",
        io::sim::log_tag(cam->mode()), frame.frame_id,
        std::chrono::duration<double>(frame.timestamp - t_start).count(), frame.width, frame.height,
        q.w(), ypr[0] * 57.3, frame.injected ? "[INJECTED]" : "");
    }

    if (show && !frame.data.empty()) {
      cv::Mat vis = frame.data.clone();
      cv::putText(
        vis, fmt::format("{} {}fps {}", io::sim::to_string(cam->mode()), static_cast<int>(frames / std::max(0.1, elapsed)),
                         frame.injected ? "INJECTED" : ""),
        {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
      cv::Mat small;
      cv::resize(vis, small, {}, 0.5, 0.5);
      cv::imshow("virtual_run", small);
      const int key = cv::waitKey(1);
      if (key == 'q' || key == 27) break;
    }
  }

  cam->close();
  imu->close();
  if (camera_recorder) camera_recorder->close();
  if (serial_recorder) serial_recorder->close();

  const double elapsed = std::chrono::duration<double>(io::sim::SteadyClock::now() - t_start).count();

  tools::logger()->info("================ [{}] 汇总 ================", io::sim::log_tag(imu->mode()));
  tools::logger()->info(
    "运行 {:.2f}s | 图像帧 {} (超时 {}) | 注入异常帧 {} | 有效四元数 {} | 平均 {:.1f} fps | 最大帧间隔 {:.1f} ms",
    elapsed, frames, timeouts, injected_frames, valid_q, frames / std::max(1e-3, elapsed),
    max_frame_gap * 1e3);

  if (sim.record.enable)
    tools::logger()->info(
      "记录目录: {} (serial.bin / camera.tsv / camera/) 使用 source_mode=replay 可复现本次运行",
      sim.record.dir);

  return 0;
}
