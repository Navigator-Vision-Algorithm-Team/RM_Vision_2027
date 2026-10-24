#include "io/sim/camera_source.hpp"

#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <stdexcept>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io::sim
{

namespace
{
std::chrono::steady_clock::duration capped(const std::chrono::steady_clock::duration & remain)
{
  const auto cap =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(2));
  return remain < cap ? remain : cap;
}

bool is_image_file(const std::string & path)
{
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos) return false;
  std::string ext = path.substr(dot);
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
  return ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp";
}

std::string read_camera_name(const std::string & config_path, const std::string & sursign)
{
  try {
    auto yaml = tools::load(config_path);
    const std::string key = sursign.empty() ? "camera_name" : "camera_name_" + sursign;
    return node_as<std::string>(yaml, key, "");
  } catch (...) {
    return "";
  }
}
}  // namespace

// ============================ RealHikCameraSource ============================
RealHikCameraSource::RealHikCameraSource(std::string config_path, std::string sursign)
: config_path_(std::move(config_path)), sursign_(std::move(sursign))
{
}

void RealHikCameraSource::open()
{
  const std::string name = read_camera_name(config_path_, sursign_);
  if (name == "fake" || name == "replay") {
    throw std::runtime_error(
      "[RealCamera] camera_name='" + name +
      "' 会导致递归；真实相机请使用 hikrobot/mindvision/Usbcamera，"
      "伪相机请通过 io::Camera 工厂的 fake/replay 分支创建。");
  }

  camera_ = std::make_unique<io::Camera>(config_path_, sursign_);
  tools::logger()->info(
    "[REAL] Camera '{}' opened (camera_name={})", camera_->device_name(), name);
}

bool RealHikCameraSource::read(Frame & out, int /*timeout_ms*/)
{
  if (!camera_) return false;

  out.data = cv::Mat();
  camera_->read(out.data, out.timestamp);
  if (out.data.empty()) return false;

  out.frame_id = frame_id_++;
  out.width = out.data.cols;
  out.height = out.data.rows;
  out.format = out.data.type();
  out.mode = SourceMode::Real;
  out.injected = false;
  return true;
}

void RealHikCameraSource::close()
{
  camera_.reset();
  tools::logger()->info("[REAL] Camera closed");
}

std::string RealHikCameraSource::device_name() const
{
  return camera_ ? camera_->device_name() : std::string("real");
}

// ============================ FakeCameraSource ============================
FakeCameraSource::FakeCameraSource(
  const CameraSourceConfig & cfg, const FaultConfig & fault, int seed, std::string tag)
: cfg_(cfg), fault_(fault), injector_(fault, seed)
{
  if (cfg_.width < 2) cfg_.width = 1440;
  if (cfg_.height < 2) cfg_.height = 1080;
  if (cfg_.fps <= 0.0) cfg_.fps = 30.0;
  if (cfg_.pool < 2) cfg_.pool = 2;
  tag_ = tag.empty() ? std::string("cam") : std::move(tag);
  device_name_ = "fake_" + cfg_.kind + "[" + tag_ + "]";
  cv::theRNG().state = static_cast<uint64_t>(seed);
}

FakeCameraSource::~FakeCameraSource() { close(); }

// 底图只渲染一次：这张图会被所有帧复用，所以这里可以画得足够"重"
void FakeCameraSource::build_base()
{
  const int w = cfg_.width, h = cfg_.height;

  base_.create(h, w, CV_8UC3);
  for (int y = 0; y < h; ++y) base_.row(y).setTo(cv::Scalar(40 + 30 * y / h, 60, 90 - 30 * y / h));

  const int cell = 80;
  for (int y = 0; y < h; y += cell)
    for (int x = 0; x < w; x += cell)
      if (((x / cell) + (y / cell)) % 2 == 0)
        cv::rectangle(
          base_, {x, y}, {std::min(x + cell, w - 1), std::min(y + cell, h - 1)},
          cv::Scalar(190, 190, 190), 2);

  // 中心十字 + 相机名：便于人工核对是哪一路
  cv::line(base_, {w / 2 - 80, h / 2}, {w / 2 + 80, h / 2}, cv::Scalar(0, 0, 255), 3);
  cv::line(base_, {w / 2, h / 2 - 80}, {w / 2, h / 2 + 80}, cv::Scalar(0, 0, 255), 3);
  cv::putText(
    base_, tag_, {w / 2 - 120, h / 2 - 120}, cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 255, 0), 6);

  // overlay 复原块：每帧贴回去，避免多帧叠加残影（~45KB，可忽略）
  overlay_roi_ = cv::Rect(4, 4, std::min(360, w - 8), std::min(70, h - 8));
  patch_ = base_(overlay_roi_).clone();
}

// 产帧热路径：只做一块 overlay 复原 + 一小段文字，没有全帧写入
void FakeCameraSource::render_overlay(cv::Mat & buf)
{
  if (patch_.empty() || buf.cols < overlay_roi_.x + overlay_roi_.width ||
      buf.rows < overlay_roi_.y + overlay_roi_.height)
    return;

  patch_.copyTo(buf(overlay_roi_));
  const double t =
    std::chrono::duration<double>(SteadyClock::now() - SessionClock::start_time()).count();
  cv::putText(
    buf, fmt::format("{} f={} t={:.2f}", tag_, frame_id_, t), {12, 46}, cv::FONT_HERSHEY_SIMPLEX,
    0.9, cv::Scalar(0, 255, 255), 2);
}

void FakeCameraSource::open()
{
  if (cfg_.kind == "sequence") {
    std::vector<cv::String> found;
    cv::glob(cfg_.dir + "/*", found, false);
    for (const auto & f : found)
      if (is_image_file(f)) files_.push_back(f);
    std::sort(files_.begin(), files_.end());
    if (files_.empty()) {
      tools::logger()->warn("[FAKE] No images in '{}', fallback to synth pattern", cfg_.dir);
      cfg_.kind = "synth";
    } else {
      tools::logger()->info("[FAKE] FakeCamera sequence source: {} images from '{}'", files_.size(), cfg_.dir);
    }
  } else if (cfg_.kind == "video") {
    video_.open(cfg_.video);
    if (!video_.isOpened()) {
      tools::logger()->warn("[FAKE] Cannot open video '{}', fallback to synth pattern", cfg_.video);
      cfg_.kind = "synth";
    } else {
      tools::logger()->info("[FAKE] FakeCamera video source: '{}'", cfg_.video);
    }
  }

  // 所有 kind 都预渲染底图（一次性）：序列/视频加载失败时用它兜底
  build_base();

  // synth：高帧率零拷贝路径 —— 缓冲池 + 单槽 holder
  if (cfg_.kind == "synth") {
    pool_.resize(static_cast<size_t>(cfg_.pool));
    pool_free_.assign(pool_.size(), true);
    for (auto & buf : pool_) buf = base_.clone();
    tools::logger()->info(
      "[FAKE] '{}' zero-copy pool ready: {} buffers of {}x{}", tag_, pool_.size(), cfg_.width,
      cfg_.height);
  }

  // 冻结注入需要上一帧内容，仅在开启时分配
  if (fault_.camera_freeze_rate > 0.0) freeze_.create(cfg_.height, cfg_.width, CV_8UC3);

  quit_ = false;
  thread_ = std::thread([this] { run(); });
}

cv::Mat FakeCameraSource::next_image()
{
  if (cfg_.kind == "sequence" && !files_.empty()) {
    cv::Mat img = cv::imread(files_[file_index_ % files_.size()], cv::IMREAD_COLOR);
    file_index_++;
    if (!img.empty()) return img;
  } else if (cfg_.kind == "video" && video_.isOpened()) {
    cv::Mat img;
    video_ >> img;
    if (img.empty()) {
      video_.set(cv::CAP_PROP_POS_FRAMES, 0);
      video_ >> img;
    }
    if (!img.empty()) return img;
  }
  // 兜底：预渲染底图（一次性构建），这里只做一次拷贝
  return base_.empty() ? cv::Mat() : base_.clone();
}

// 取一个空闲池缓冲；池内全忙返回 -1（产帧端让路，绝不阻塞消费者）
int FakeCameraSource::take_free_buffer()
{
  std::lock_guard<std::mutex> lock(holder_mutex_);
  for (size_t i = 0; i < pool_.size(); ++i) {
    if (pool_free_[i]) {
      pool_free_[i] = false;
      return static_cast<int>(i);
    }
  }
  return -1;
}

// 发布到单槽 holder：与真实相机一致，永远让消费者拿到"最新帧"。
// 被覆盖掉的旧帧计入 overwritten_ —— 这是"产得比消费快"的直接指标。
void FakeCameraSource::publish_frame(const cv::Mat & img, int pool_idx, bool injected)
{
  if (img.empty()) return;

  {
    std::lock_guard<std::mutex> lock(holder_mutex_);
    if (holder_idx_ >= 0) {
      pool_free_[static_cast<size_t>(holder_idx_)] = true;
      overwritten_++;
    }

    holder_.timestamp = SteadyClock::now();
    holder_.frame_id = frame_id_++;
    holder_.width = img.cols;
    holder_.height = img.rows;
    holder_.format = img.type();
    holder_.data = img;  // 池内缓冲共享，零拷贝
    holder_.mode = SourceMode::Fake;
    holder_.injected = injected;
    holder_idx_ = pool_idx;

    emitted_++;
    if (injected) injected_count_++;
  }

  holder_cv_.notify_all();
}

void FakeCameraSource::run()
{
  const double period = 1.0 / cfg_.fps;
  const auto t0 = SteadyClock::now();
  long long tick = 0;
  auto last_log = t0;
  long long win_emitted = 0, win_delivered = 0;
  const bool pooled = (cfg_.kind == "synth");

  tools::logger()->info(
    "[FAKE] '{}' started: {}x{} @{}fps kind={} pool={} faults={}", tag_, cfg_.width, cfg_.height,
    cfg_.fps, cfg_.kind, pooled ? static_cast<int>(pool_.size()) : 0,
    fault_.enabled() ? "on" : "off");

  while (!quit_) {
    const auto target =
      t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
             std::chrono::duration<double>(static_cast<double>(tick) * period));
    tick++;

    while (!quit_) {
      const auto now = SteadyClock::now();
      if (now >= target) break;
      std::this_thread::sleep_for(capped(target - now));
    }
    if (quit_) break;

    if (injector_.chance(fault_.camera_drop_rate)) {
      dropped_++;
      tools::logger()->warn("[FAULT:drop] camera frame dropped (count={})", dropped_);
      continue;
    }

    bool injected = false;
    cv::Mat img;
    int pool_idx = -1;

    if (pooled) {
      // 零拷贝路径：从池取一个空闲缓冲，只重画一小块 overlay
      pool_idx = take_free_buffer();
      if (pool_idx < 0) {
        starved_++;
        if (starved_ == 1 || starved_ % 5000 == 0)
          tools::logger()->warn(
            "[FAKE] '{}' pool exhausted ({} frames given up, consumer too slow?)", tag_, starved_);
        continue;
      }
      img = pool_[static_cast<size_t>(pool_idx)];
      render_overlay(img);
    } else {
      // 序列 / 视频：每帧本来就要读一张新图，无法零拷贝
      img = next_image();
      if (img.empty()) continue;
      img = img.clone();
    }

    if (!freeze_.empty() && injector_.chance(fault_.camera_freeze_rate)) {
      freeze_.copyTo(img);
      injected = true;
      tools::logger()->warn("[FAULT:freeze] camera frame frozen (count={})", injected_count_ + 1);
    }

    if (injector_.chance(fault_.camera_black_rate)) {
      img.setTo(cv::Scalar(0, 0, 0));
      injected = true;
      tools::logger()->warn("[FAULT:black] camera frame blackened");
    }
    if (injector_.chance(fault_.camera_overexpose_rate)) {
      cv::add(img, cv::Scalar(120, 120, 120), img);
      injected = true;
      tools::logger()->warn("[FAULT:expose] camera frame overexposed");
    }
    if (fault_.camera_noise_sigma > 0.0) {
      cv::Mat f, noise;
      img.convertTo(f, CV_32FC3);
      noise.create(f.size(), CV_32FC3);
      cv::randn(noise, 0.0, fault_.camera_noise_sigma);
      f += noise;
      f.convertTo(img, CV_8UC3);
      injected = true;
    }
    if (fault_.camera_delay_ms > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(fault_.camera_delay_ms));
      injected = true;
    }

    publish_frame(img, pool_idx, injected);

    if (injector_.chance(fault_.camera_duplicate_rate)) {
      duplicated_++;
      if (pooled) {
        // 复制到另一个空闲缓冲，避免与 holder 中的同一块缓冲冲突
        const int dup_idx = take_free_buffer();
        if (dup_idx >= 0) {
          img.copyTo(pool_[static_cast<size_t>(dup_idx)]);
          publish_frame(pool_[static_cast<size_t>(dup_idx)], dup_idx, true);
        } else {
          starved_++;
        }
      } else {
        publish_frame(img.clone(), -1, true);
      }
      tools::logger()->warn("[FAULT:dup] camera frame duplicated (count={})", duplicated_);
    }

    // 记录本帧内容供冻结注入使用（仅开启冻结时才有这次额外拷贝）
    if (!freeze_.empty()) img.copyTo(freeze_);

    const auto now = SteadyClock::now();
    if (std::chrono::duration<double>(now - last_log).count() >= 2.0) {
      const double dt = std::chrono::duration<double>(now - last_log).count();
      tools::logger()->info(
        "[FAKE] '{}' stats: emitted={} ({:.1f}fps) delivered={} ({:.1f}fps) overwritten={} "
        "starved={} dropped={} dup={} injected={}",
        tag_, emitted_, static_cast<double>(emitted_ - win_emitted) / dt, delivered_,
        static_cast<double>(delivered_ - win_delivered) / dt, overwritten_, starved_, dropped_,
        duplicated_, injected_count_);
      win_emitted = emitted_;
      win_delivered = delivered_;
      last_log = now;
    }
  }

  tools::logger()->info(
    "[FAKE] '{}' stopped: emitted={} delivered={} overwritten={} starved={} dropped={} dup={} "
    "injected={}",
    tag_, emitted_, delivered_, overwritten_, starved_, dropped_, duplicated_, injected_count_);
}

bool FakeCameraSource::read(Frame & out, int timeout_ms)
{
  const bool bounded = (timeout_ms > 0);
  const auto deadline =
    SteadyClock::now() + std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 0);

  while (!quit_) {
    {
      std::unique_lock<std::mutex> lock(holder_mutex_);

      auto slice = std::chrono::milliseconds(100);
      if (bounded) {
        const auto remain =
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - SteadyClock::now());
        if (remain.count() <= 0) return false;
        if (remain < slice) slice = remain;
      }

      holder_cv_.wait_for(lock, slice, [this] { return !holder_.data.empty() || quit_; });
      if (quit_) break;

      // 池化路径(synth)的 pool_idx >= 0，非池化路径(video/sequence)为 -1，
      // 因此“是否有新帧”只能看 holder_.data，不能用 holder_idx_ 判断
      if (!holder_.data.empty()) {
        // 关键：把池内缓冲换成本次独占的克隆。
        // 只有真正被消费的帧付这一次全帧拷贝，产帧端 1500fps 的路径上没有全帧写入。
        out = holder_;
        out.data = out.data.clone();
        // 只有池化路径持有的缓冲才需要归还，-1 时归还会造成越界写
        if (holder_idx_ >= 0) pool_free_[static_cast<size_t>(holder_idx_)] = true;
        holder_idx_ = -1;
        holder_ = Frame{};
        delivered_++;
        return true;
      }
    }

    if (bounded) return false;
    if (tools::exiting()) break;  // 与真实相机一致：退出信号可以打断阻塞
  }
  return false;
}

void FakeCameraSource::close()
{
  quit_ = true;
  holder_cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

// ============================ ReplayCameraSource ============================
ReplayCameraSource::ReplayCameraSource(
  const CameraSourceConfig & cfg, const FaultConfig & /*fault*/, int /*seed*/)
: cfg_(cfg), reader_(cfg.replay_dir)
{
  device_name_ = "replay";
}

ReplayCameraSource::~ReplayCameraSource() { close(); }

void ReplayCameraSource::open()
{
  quit_ = false;
  thread_ = std::thread([this] { run(); });
}

void ReplayCameraSource::run()
{
  std::string error;
  if (!reader_.load(error)) {
    tools::logger()->error("[REPLAY] Camera replay load failed: {}", error);
    return;
  }

  const auto & records = reader_.records();
  if (records.empty()) return;

  ready_ = true;
  const uint64_t base = records.front().t_ns;

  tools::logger()->info(
    "[REPLAY] Camera replay started: dir={} frames={} span={:.2f}s loop={}", cfg_.replay_dir,
    records.size(), reader_.duration_ns() / 1e9, loop_);

  do {
    const auto t0 = SteadyClock::now();
    for (size_t i = 0; i < records.size(); ++i) {
      if (quit_) break;

      const auto target = t0 + std::chrono::nanoseconds(records[i].t_ns - base);
      while (!quit_) {
        const auto now = SteadyClock::now();
        if (now >= target) break;
        std::this_thread::sleep_for(capped(target - now));
      }
      if (quit_) break;

      cv::Mat img = reader_.read_image(i);
      if (img.empty())
        img = cv::Mat(records[i].height, records[i].width, CV_8UC3, cv::Scalar(0, 0, 0));

      Frame f;
      f.timestamp = SteadyClock::now();
      f.frame_id = records[i].frame_id;
      f.width = img.cols;
      f.height = img.rows;
      f.format = img.type();
      f.data = img;
      f.mode = SourceMode::Replay;

      queue_.push(f);
      emitted_++;
    }
  } while (loop_ && !quit_);

  tools::logger()->info("[REPLAY] Camera replay finished: {} frames", emitted_);
}

bool ReplayCameraSource::read(Frame & out, int timeout_ms)
{
  if (timeout_ms > 0) return queue_.try_pop_for(out, std::chrono::milliseconds(timeout_ms));

  while (!quit_) {
    if (queue_.try_pop_for(out, std::chrono::milliseconds(100))) return true;
  }
  return false;
}

void ReplayCameraSource::close()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
}

// ============================ CameraSourceAdapter ============================
CameraSourceAdapter::CameraSourceAdapter(CameraSource::Ptr source, std::string device_name)
: source_(std::move(source))
{
  device_name_ = device_name.empty() ? (source_ ? source_->device_name() : std::string("fake"))
                                     : device_name;
  if (source_) source_->open();
  tools::logger()->info(
    "[{}] CameraSourceAdapter '{}' ready (source={}, camera={})",
    source_ ? log_tag(source_->mode()) : "?", device_name_, source_ ? source_->name() : "none",
    device_name_);
}

CameraSourceAdapter::~CameraSourceAdapter()
{
  if (source_) source_->close();
}

void CameraSourceAdapter::read(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  if (!source_) {
    img.release();
    timestamp = SteadyClock::now();
    return;
  }

  Frame f;
  if (!source_->read(f, 0)) {
    img.release();
    timestamp = SteadyClock::now();
    return;
  }

  img = f.data;
  timestamp = f.timestamp;

  if (f.injected)
    tools::logger()->warn(
      "[FAULT] frame {} delivered with injected anomaly ({}x{})", f.frame_id, f.width, f.height);
}

std::string CameraSourceAdapter::device_name() const { return device_name_; }

// ============================ 工厂 ============================
CameraSource::Ptr make_camera_source(
  const YAML::Node & yaml, const std::string & config_path, const std::string & sursign,
  SourceMode force, bool use_force)
{
  const SimConfig sim = SimConfig::load(yaml);
  CameraSourceConfig cfg = CameraSourceConfig::load(yaml, sim.mode, sursign);
  if (use_force) cfg.mode = force;

  switch (cfg.mode) {
    case SourceMode::Real:
      return std::make_unique<RealHikCameraSource>(config_path, sursign);
    case SourceMode::Fake:
      return std::make_unique<FakeCameraSource>(
        cfg, sim.fault, sim.seed, sursign.empty() ? std::string("main") : sursign);
    case SourceMode::Replay:
      return std::make_unique<ReplayCameraSource>(cfg, sim.fault, sim.seed);
  }
  return std::make_unique<FakeCameraSource>(
    cfg, sim.fault, sim.seed, sursign.empty() ? std::string("main") : sursign);
}

}  // namespace io::sim
