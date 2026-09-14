#include "hikrobot.hpp"

#include <algorithm>
#include <chrono>

#include "hik_camera_manager.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace
{
// 守护线程巡检周期
constexpr auto k_watchdog_tick = 100ms;
// 启动后一直没有收到首帧的超时时间
constexpr auto k_first_frame_timeout = 3s;
// 曾经出过帧、但连续无新帧的超时时间（判定掉流）
constexpr auto k_stream_timeout = 3s;

// 重启退避：0.5s → 1s → 2s 封顶，
// 避免多台相机同时失败时以约 10 次/秒的节奏冲击 SDK/USB。
void sleep_before_restart(int consecutive_failures)
{
  const int steps = std::min(consecutive_failures, 3);
  std::this_thread::sleep_for(500ms * (1 << (steps - 1)));
}

}  // namespace

namespace io
{
HikRobot::HikRobot(
  double exposure_ms, double gain,
  const std::string & device_name, const std::string & serial_number,
  double frame_rate, unsigned int transfer_size)
: exposure_us_(exposure_ms * 1e3), gain_(gain), frame_rate_(frame_rate),
  transfer_size_(transfer_size), serial_number_(serial_number), queue_(1)
{
  device_name_ = device_name;

  tools::logger()->info(
    "[{}] HikRobot config: serial='{}' exposure_ms={} gain={} frame_rate={} transfer_size={}",
    device_name_, serial_number_.empty() ? std::string("N/A") : serial_number_,
    exposure_ms, gain_, frame_rate_, transfer_size_);

  // 唯一的自有线程：负责相机的打开、参数下发、掉流巡检、停流与重启。
  // 海康 SDK 内部线程只通过 on_frame() 投递帧数据。
  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("[{}] Daemon thread started", device_name_);

    int consecutive_failures = 0;

    while (!daemon_quit_) {
      if (!capture_start()) {
        // 打开失败往往是设备信息已过期（相机被拔出/换口），触发重新枚举后重试
        HikCameraManager::instance().invalidate();
        restart_count_.fetch_add(1, std::memory_order_relaxed);
        sleep_before_restart(++consecutive_failures);
        continue;
      }

      consecutive_failures = 0;

      watchdog_frames();  // 掉流或无首帧时返回；daemon_quit_ 置位时退出

      capture_stop();

      if (!daemon_quit_) {
        restart_count_.fetch_add(1, std::memory_order_relaxed);
        sleep_before_restart(++consecutive_failures);
      }
    }

    tools::logger()->info("[{}] Daemon thread stopped", device_name_);
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();

  capture_stop();  // 兜底：正常退出路径上守护线程已完成清理

  tools::logger()->info("[{}] HikRobot destructed", device_name_);
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;

  while (true) {
    if (queue_.try_pop_for(data, 100ms)) {
      img = data.img;
      timestamp = data.timestamp;
      return;
    }

    if (tools::exiting()) {
      img.release();
      timestamp = std::chrono::steady_clock::now();
      return;
    }
  }
}

// static — called from SDK internal thread.  Must never throw (__stdcall boundary).
void __stdcall HikRobot::on_image_callback(
  unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo, void * pUser)
{
  if (!pUser) return;
  try {
    static_cast<HikRobot *>(pUser)->on_frame(pData, pFrameInfo);
  } catch (...) {
    // Absorb any exception — crossing __stdcall with an exception is UB.
  }
}

// 回调必须短、快、不阻塞：只做校验、拷贝、计数与非阻塞入队。
void HikRobot::on_frame(unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo)
{
  // 相机正在 shutdown，立即丢弃这一帧
  if (stopping_) return;

  // Defensive: SDK may deliver NULL or zero-dimension frames on error
  if (!pData || !pFrameInfo) return;

  // Sanity-check dimensions — anything exceeding 8K is garbage
  if (pFrameInfo->nWidth == 0 || pFrameInfo->nHeight == 0) return;
  if (pFrameInfo->nWidth > 8192 || pFrameInfo->nHeight > 8192) return;

  // MV_CC_RegisterImageCallBackForBGR 返回的是转换后的 BGR24 数据，
  // 因此一帧的字节数应为 width * height * 3。
  const unsigned int expected_bytes =
    static_cast<unsigned int>(pFrameInfo->nWidth) *
    static_cast<unsigned int>(pFrameInfo->nHeight) * 3;
  if (pFrameInfo->nFrameLen < expected_bytes) return;

  cv::Mat mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, pData);
  if (mat.empty()) return;

  CameraData data;
  data.img = mat.clone();
  data.timestamp = std::chrono::steady_clock::now();

  frame_counter_.fetch_add(1, std::memory_order_relaxed);

  if (!first_frame_received_.exchange(true)) {
    tools::logger()->info(
      "[{}] First frame received: {}x{} frame_num={} len={}",
      device_name_, pFrameInfo->nWidth, pFrameInfo->nHeight,
      pFrameInfo->nFrameNum, pFrameInfo->nFrameLen);
  }

  // 队列满时丢弃旧帧、保留最新帧，绝不阻塞 SDK 回调线程
  if (!data.img.empty()) {
    queue_.push(data);
  }
}

bool HikRobot::capture_start()
{
  stopping_ = false;
  first_frame_received_ = false;
  frame_counter_ = 0;
  grabbing_ = false;

  handle_ = HikCameraManager::instance().create_handle(serial_number_);
  if (!handle_) {
    tools::logger()->warn(
      "[{}] Failed to create handle: serial='{}'", device_name_,
      serial_number_.empty() ? std::string("N/A") : serial_number_);
    return false;
  }

  // SDK 调用结果统一日志：成功 info，失败 warn
  auto log_step = [this](const char * step, unsigned int ret_code) {
    if (ret_code == MV_OK)
      tools::logger()->info("[{}] MV_CC_{} OK, handle={}", device_name_, step, fmt::ptr(handle_));
    else
      tools::logger()->warn(
        "[{}] MV_CC_{} failed: {:#x}, handle={}", device_name_, step, ret_code, fmt::ptr(handle_));
  };

  // 致命步骤失败：统一走 capture_stop() 释放句柄与已打开的设备
  auto fail_cleanup = [this, &log_step](const char * step, unsigned int ret_code) {
    log_step(step, ret_code);
    capture_stop();
    return false;
  };

  unsigned int ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) return fail_cleanup("OpenDevice", ret);
  opened_ = true;

  // 采集模式与 USB 传输参数
  ret = MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
  log_step("SetEnumValue(TriggerMode=OFF)", ret);

  ret = MV_CC_SetEnumValue(handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
  log_step("SetEnumValue(AcquisitionMode=CONTINUOUS)", ret);

  // Reduce USB transfer channels per camera.  Default is 8 for our camera
  // model, but with 5 cameras that's 40 concurrent USB transfers competing
  // for xHCI scheduling.  2 channels × 5 cameras = 10 total — much more
  // manageable while still providing enough throughput for 15 fps Bayer data.
  // 出现usb通道爆炸的时候就设置这个地方。
  ret = MV_USB_SetTransferWays(handle_, 2);
  log_step("SetTransferWays(2)", ret);

  // Let the camera use its default pixel format rather than forcing BayerRG8.
  // The SDK's BGR callback handles conversion regardless of source format.
  // Different camera revisions may prefer different native formats.
  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);

  if (frame_rate_ > 0.0) {
    ret = MV_CC_SetFrameRate(handle_, static_cast<float>(frame_rate_));
    log_step("SetFrameRate", ret);
  }

  if (transfer_size_ > 0) {
    ret = MV_USB_SetTransferSize(handle_, transfer_size_);
    log_step("SetTransferSize", ret);
  }

  // Set internal buffer depth BEFORE registering callback.
  // SDK default is only 1 image node per camera — with N cameras streaming
  // simultaneously, a single buffer per camera is not enough to absorb USB
  // transfer jitter.  MVS typically uses 4-8 nodes per camera.
  ret = MV_CC_SetImageNodeNum(handle_, 5);
  log_step("SetImageNodeNum(5)", ret);

  // Register BGR callback BEFORE StartGrabbing.
  // In callback mode, the SDK manages its own USB transfer threads and delivers
  // frames via this callback — no polling threads compete for shared USB state.
  ret = MV_CC_RegisterImageCallBackForBGR(handle_, on_image_callback, this);
  if (ret != MV_OK) return fail_cleanup("RegisterImageCallBackForBGR", ret);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) return fail_cleanup("StartGrabbing", ret);

  grabbing_ = true;
  tools::logger()->info(
    "[{}] Capture started (callback mode), handle={}", device_name_, fmt::ptr(handle_));

  return true;
}

void HikRobot::capture_stop()
{
  stopping_ = true;  // 回调立即停止处理帧

  if (!handle_) return;

  if (grabbing_) {
    const unsigned int ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK)
      tools::logger()->warn("[{}] MV_CC_StopGrabbing failed: {:#x}", device_name_, ret);
    grabbing_ = false;
  }

  if (opened_) {
    const unsigned int ret = MV_CC_CloseDevice(handle_);
    if (ret != MV_OK)
      tools::logger()->warn("[{}] MV_CC_CloseDevice failed: {:#x}", device_name_, ret);
    opened_ = false;
  }

  const unsigned int ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK)
    tools::logger()->warn("[{}] MV_CC_DestroyHandle failed: {:#x}", device_name_, ret);

  handle_ = nullptr;

  tools::logger()->info(
    "[{}] Capture stopped, restart_count={}", device_name_,
    restart_count_.load(std::memory_order_relaxed));
}

// 巡检真实帧计数：出帧则刷新，连续无新帧或无首帧则返回，由守护线程重启相机。
void HikRobot::watchdog_frames()
{
  const auto start = std::chrono::steady_clock::now();
  auto last_frame_time = start;
  auto last_counter = frame_counter_.load(std::memory_order_relaxed);

  while (!daemon_quit_) {
    std::this_thread::sleep_for(k_watchdog_tick);

    const auto now = std::chrono::steady_clock::now();
    const auto counter = frame_counter_.load(std::memory_order_relaxed);
    if (counter != last_counter) {
      last_counter = counter;
      last_frame_time = now;
    }

    // 启动后 3s 没有首帧：dump 相机配置，交回守护线程重启
    if (!first_frame_received_ && now - start > k_first_frame_timeout) {
      dump_camera_config();
      tools::logger()->warn(
        "[{}] No frame received after {}ms, restarting", device_name_,
        std::chrono::duration_cast<std::chrono::milliseconds>(k_first_frame_timeout).count());
      return;
    }

    // 曾经出帧、但连续 3s 无新帧：判定掉流
    if (first_frame_received_ && now - last_frame_time > k_stream_timeout) {
      tools::logger()->warn(
        "[{}] Stream stalled for {}ms, restarting: serial='{}' frame_counter={} restart_count={}",
        device_name_,
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time).count(),
        serial_number_.empty() ? std::string("N/A") : serial_number_, counter,
        restart_count_.load(std::memory_order_relaxed));
      return;
    }
  }
}

void HikRobot::dump_camera_config()
{
  MVCC_ENUMVALUE trigger_mode{};
  MVCC_ENUMVALUE trigger_source{};
  MVCC_ENUMVALUE acquisition_mode{};
  MVCC_ENUMVALUE pixel_format{};

  MV_CC_GetEnumValue(handle_, "TriggerMode", &trigger_mode);
  MV_CC_GetEnumValue(handle_, "TriggerSource", &trigger_source);
  MV_CC_GetEnumValue(handle_, "AcquisitionMode", &acquisition_mode);
  MV_CC_GetEnumValue(handle_, "PixelFormat", &pixel_format);

  tools::logger()->warn(
    "[{}] Camera config: serial='{}' trigger_mode={} trigger_source={} "
    "acquisition_mode={} pixel_format={:#x} frame_rate={} opened={} grabbing={}",
    device_name_, serial_number_.empty() ? std::string("N/A") : serial_number_,
    trigger_mode.nCurValue, trigger_source.nCurValue, acquisition_mode.nCurValue,
    pixel_format.nCurValue, frame_rate_, opened_.load(), grabbing_.load());
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  const unsigned int ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK)
    tools::logger()->warn(
      "[{}] MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", device_name_, name, value, ret);
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  const unsigned int ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK)
    tools::logger()->warn(
      "[{}] MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", device_name_, name, value, ret);
}

}  // namespace io
