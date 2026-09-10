#include "hikrobot.hpp"

#include <cstring>
#include <mutex>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(
  double exposure_ms, double gain,
  const std::string & device_name, const std::string & serial_number,
  double frame_rate, unsigned int transfer_size)
: exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false),
  capturing_(false), capture_quit_(false), first_frame_received_(false),
  handle_(nullptr), serial_number_(serial_number), frame_rate_(frame_rate),
  transfer_size_(transfer_size)
{
  device_name_ = device_name;

  tools::logger()->info(
    "[{}] HikRobot config: serial='{}' exposure_ms={} gain={} frame_rate={} transfer_size={}",
    device_name_, serial_number_.empty() ? std::string("N/A") : serial_number_,
    exposure_ms, gain_, frame_rate_, transfer_size_);

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("[{}] Daemon thread started", device_name_);

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      tools::logger()->warn("[{}] Capture stopped unexpectedly, restarting...", device_name_);
      capture_stop();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("[{}] Daemon thread stopped", device_name_);
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
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

void HikRobot::on_frame(unsigned char * pData, MV_FRAME_OUT_INFO_EX * pFrameInfo)
{
  // Bail out if shutdown is in progress (capture_stop already running)
  if (capture_quit_) return;

  // Defensive: SDK may deliver NULL or zero-dimension frames on error
  if (!pData || !pFrameInfo) return;

  // Sanity-check dimensions — anything exceeding 8K is garbage
  if (pFrameInfo->nWidth == 0 || pFrameInfo->nHeight == 0) return;
  if (pFrameInfo->nWidth > 8192 || pFrameInfo->nHeight > 8192) return;

  // Verify the delivered buffer is large enough.  nFrameLen == 0 is also
  // invalid (can happen on corrupted / status-only frames from SDK).
  unsigned int expected_bytes =
    static_cast<unsigned int>(pFrameInfo->nWidth) *
    static_cast<unsigned int>(pFrameInfo->nHeight) * 3;
  if (pFrameInfo->nFrameLen < expected_bytes) return;

  cv::Mat mat(
    pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, pData);
  if (mat.empty()) return;

  CameraData data;
  data.img = mat.clone();
  data.timestamp = std::chrono::steady_clock::now();

  if (!first_frame_received_.exchange(true)) {
    tools::logger()->info(
      "[{}] First frame received: {}x{} frame_num={} len={}",
      device_name_, pFrameInfo->nWidth, pFrameInfo->nHeight,
      pFrameInfo->nFrameNum, pFrameInfo->nFrameLen);
  }

  if (!data.img.empty()) queue_.push(data);
}

void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;
  first_frame_received_ = false;

  unsigned int ret;
  auto log_step = [this](const char * step, unsigned int ret_code) {
    if (ret_code == MV_OK)
      tools::logger()->info("[{}] MV_CC_{} OK, handle={}", device_name_, step, fmt::ptr(handle_));
    else
      tools::logger()->warn("[{}] MV_CC_{} failed: {:#x}, handle={}", device_name_, step, ret_code, fmt::ptr(handle_));
  };

  // Step 1: Enumerate devices — ONCE globally, never while another camera is streaming.
  // Calling EnumDevices while other cameras are actively streaming can disrupt their
  // USB transfers (bus scan resets the hub).  MVS enumerates once then opens all cameras.
  static std::once_flag enum_once;
  static MV_CC_DEVICE_INFO_LIST cached_list;
  static unsigned int cached_count = 0;
  static bool enum_ok = false;

  //枚举相机，并且缓存相机信息，避免多次枚举相机导致的USB总线竞争
  std::call_once(enum_once, []() {
    memset(&cached_list, 0, sizeof(cached_list));
    unsigned int r = MV_CC_EnumDevices(MV_USB_DEVICE, &cached_list);
    if (r == MV_OK && cached_list.nDeviceNum > 0) {
      enum_ok = true;
      cached_count = cached_list.nDeviceNum;
      for (unsigned int i = 0; i < cached_list.nDeviceNum; i++) {
        auto * info = &cached_list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo;
        tools::logger()->info("[Global] Found camera [{}]: serial='{}' model='{}'", i,
                              reinterpret_cast<char *>(info->chSerialNumber),
                              reinterpret_cast<char *>(info->chModelName));
      }
    } else {
      tools::logger()->warn("[Global] MV_CC_EnumDevices failed or no cameras: {:#x}", r);
    }
  });

  if (!enum_ok) {
    tools::logger()->warn("[{}] Device enumeration failed or no cameras", device_name_);
    return;
  }

  // Step 2: Match camera by serial number in the cached list
  int target_index = -1;
  for (unsigned int i = 0; i < cached_count; i++) {
    auto * usb_info = &cached_list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo;
    std::string sn(reinterpret_cast<char *>(usb_info->chSerialNumber));
    if (sn == serial_number_) {
      target_index = static_cast<int>(i);
      break;
    }
  }
  if (target_index < 0) {
    tools::logger()->warn(
      "[{}] Camera with serial '{}' not found ({} device(s) in cache)", device_name_,
      serial_number_, cached_count);
    return;
  }

  // Step 3: Create handle from cached device info
  ret = MV_CC_CreateHandle(&handle_, cached_list.pDeviceInfo[target_index]);
  log_step("CreateHandle", ret);
  if (ret != MV_OK) return;

  // Step 4: Open device
  ret = MV_CC_OpenDevice(handle_);
  log_step("OpenDevice", ret);
  if (ret != MV_OK) {
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return;
  }

  // Step 5: Set capture mode parameters
  ret = MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
  log_step("SetEnumValue(TriggerMode=OFF)", ret);

  ret = MV_CC_SetEnumValue(handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
  log_step("SetEnumValue(AcquisitionMode=CONTINUOUS)", ret);

  // Reduce USB transfer channels per camera.  Default is 8 for our camera
  // model, but with 5 cameras that's 40 concurrent USB transfers competing
  // for xHCI scheduling.  2 channels × 5 cameras = 10 total — much more
  // manageable while still providing enough throughput for 15 fps Bayer data.
  //出现usb通道爆炸的时候就设置这个地方。
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
    unsigned int frame_rate_ret = MV_CC_SetFrameRate(handle_, static_cast<float>(frame_rate_));
    if (frame_rate_ret != MV_OK) {
      tools::logger()->warn("[{}] MV_CC_SetFrameRate({}) failed: {:#x}", device_name_, frame_rate_, frame_rate_ret);
    } else {
      tools::logger()->info("[{}] MV_CC_SetFrameRate({}) OK", device_name_, frame_rate_);
    }
  }

  if (transfer_size_ > 0) {
    unsigned int transfer_ret = MV_USB_SetTransferSize(handle_, transfer_size_);
    if (transfer_ret != MV_OK) {
      tools::logger()->warn("[{}] MV_USB_SetTransferSize({}) failed: {:#x}", device_name_, transfer_size_, transfer_ret);
    } else {
      tools::logger()->info("[{}] MV_USB_SetTransferSize({}) OK", device_name_, transfer_size_);
    }
  }

  // Step 6: Set internal buffer depth BEFORE registering callback.
  // SDK default is only 1 image node per camera — with N cameras streaming
  // simultaneously, a single buffer per camera is not enough to absorb USB
  // transfer jitter.  MVS typically uses 4-8 nodes per camera.
  ret = MV_CC_SetImageNodeNum(handle_, 5);
  log_step("SetImageNodeNum(5)", ret);

  // Step 7: Register BGR callback BEFORE StartGrabbing.
  // In callback mode, the SDK manages its own USB transfer threads and delivers
  // frames via this callback — no polling threads compete for shared USB state.
  ret = MV_CC_RegisterImageCallBackForBGR(handle_, on_image_callback, this);
  log_step("RegisterImageCallBackForBGR", ret);
  if (ret != MV_OK) return;

  // Step 8: Start grabbing
  ret = MV_CC_StartGrabbing(handle_);
  log_step("StartGrabbing", ret);
  if (ret != MV_OK) return;

  // Step 9: Spawn a lightweight watch thread (no polling — frames arrive via callback)
  capture_thread_ = std::thread{[this] {
    tools::logger()->info("[{}] Capture thread started (callback mode)", device_name_);

    capturing_ = true;

    // Diagnostic: if no frame arrives within 3 seconds, dump camera config
    auto start = std::chrono::steady_clock::now();
    bool warned = false;

    while (!capture_quit_) {
      std::this_thread::sleep_for(500ms);

      if (!warned && !first_frame_received_ &&
          std::chrono::steady_clock::now() - start > 3s) {
        warned = true;

        MVCC_ENUMVALUE trigger_mode{};
        MVCC_ENUMVALUE trigger_source{};
        MVCC_ENUMVALUE acquisition_mode{};
        MVCC_ENUMVALUE pixel_format{};

        MV_CC_GetEnumValue(handle_, "TriggerMode", &trigger_mode);
        MV_CC_GetEnumValue(handle_, "TriggerSource", &trigger_source);
        MV_CC_GetEnumValue(handle_, "AcquisitionMode", &acquisition_mode);
        MV_CC_GetEnumValue(handle_, "PixelFormat", &pixel_format);

        tools::logger()->warn(
          "[{}] No frame received within 3s — serial='{}' trigger_mode={} trigger_source={} "
          "acquisition_mode={} pixel_format={:#x} frame_rate={}",
          device_name_, serial_number_.empty() ? std::string("N/A") : serial_number_,
          trigger_mode.nCurValue, trigger_source.nCurValue, acquisition_mode.nCurValue,
          pixel_format.nCurValue, frame_rate_);
      }
    }

    capturing_ = false;
    tools::logger()->info("[{}] Capture thread stopped", device_name_);
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;

  if (handle_) {
    unsigned int ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK) {
      tools::logger()->warn("[{}] MV_CC_StopGrabbing failed: {:#x}", device_name_, ret);
    } else {
      tools::logger()->info("[{}] MV_CC_StopGrabbing OK", device_name_);
    }
  }

  if (capture_thread_.joinable()) capture_thread_.join();

  if (handle_) {
    unsigned int ret = MV_CC_CloseDevice(handle_);
    if (ret != MV_OK) {
      tools::logger()->warn("[{}] MV_CC_CloseDevice failed: {:#x}", device_name_, ret);
    } else {
      tools::logger()->info("[{}] MV_CC_CloseDevice OK", device_name_);
    }

    ret = MV_CC_DestroyHandle(handle_);
    if (ret != MV_OK) {
      tools::logger()->warn("[{}] MV_CC_DestroyHandle failed: {:#x}", device_name_, ret);
    } else {
      tools::logger()->info("[{}] MV_CC_DestroyHandle OK", device_name_);
    }

    handle_ = nullptr;
  }
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("[{}] MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", device_name_, name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("[{}] MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", device_name_, name, value, ret);
    return;
  }
}

}  // namespace io
