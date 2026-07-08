#include "hikrobot.hpp"

#include <cstring>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(
  double exposure_ms, double gain,
  const std::string & device_name, const std::string & serial_number,
  double frame_rate, unsigned int grab_timeout_ms, unsigned int transfer_size)
: exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false),
  capturing_(false), capture_quit_(false),
  handle_(nullptr), serial_number_(serial_number), frame_rate_(frame_rate),
  fetch_interval_ms_(frame_rate > 0.0 ? static_cast<unsigned int>(std::max(1.0, 1000.0 / frame_rate)) : 1),
  grab_timeout_ms_(grab_timeout_ms),
  transfer_size_(transfer_size)
{
  device_name_ = device_name;

  tools::logger()->info(
    "[{}] HikRobot config: serial='{}' exposure_ms={} gain={} frame_rate={} fetch_interval_ms={} grab_timeout_ms={} transfer_size={}",
    device_name_, serial_number_.empty() ? std::string("N/A") : serial_number_,
    exposure_ms, gain_, frame_rate_, fetch_interval_ms_, grab_timeout_ms_, transfer_size_);

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

void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;
  auto log_step = [this](const char * step, unsigned int ret_code) {
    if (ret_code == MV_OK)
      tools::logger()->info("[{}] MV_CC_{} OK, handle={}", device_name_, step, fmt::ptr(handle_));
    else
      tools::logger()->warn("[{}] MV_CC_{} failed: {:#x}, handle={}", device_name_, step, ret_code, fmt::ptr(handle_));
  };

  // Step 1: Enumerate devices
  MV_CC_DEVICE_INFO_LIST device_list;
  memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  log_step("EnumDevices", ret);
  if (ret != MV_OK) return;

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("[{}] No cameras found during enumeration", device_name_);
    return;
  }

  // Log all found cameras every time (helps diagnose enumeration races)
  for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
    auto * usb_info = &device_list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo;
    tools::logger()->info("[{}] Found camera [{}]: serial='{}' model='{}'", device_name_, i,
                          reinterpret_cast<char *>(usb_info->chSerialNumber),
                          reinterpret_cast<char *>(usb_info->chModelName));
  }

  // Step 2: Match camera by serial number, or use first camera if not configured
  int target_index = 0;
  if (!serial_number_.empty()) {
    target_index = -1;
    for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
      auto * usb_info = &device_list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo;
      std::string sn(reinterpret_cast<char *>(usb_info->chSerialNumber));
      if (sn == serial_number_) {
        target_index = static_cast<int>(i);
        break;
      }
    }
    if (target_index < 0) {
      tools::logger()->warn(
        "[{}] Camera with serial '{}' not found ({} device(s) enumerated)", device_name_,
        serial_number_, device_list.nDeviceNum);
      return;
    }
  } else {
    tools::logger()->warn(
      "[{}] No serial_number configured — using first enumerated camera (index 0)", device_name_);
  }

  // Step 3: Create handle
  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[target_index]);
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

  // Step 5: Set critical capture mode parameters BEFORE StartGrabbing
  // Must explicitly set TriggerMode=OFF — if camera was left in triggered mode
  // by MVS, StartGrabbing succeeds but no frames arrive (MV_E_NODATA forever).
  ret = MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
  log_step("SetEnumValue(TriggerMode=OFF)", ret);

  ret = MV_CC_SetEnumValue(handle_, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
  log_step("SetEnumValue(AcquisitionMode=CONTINUOUS)", ret);

  // Explicitly set pixel format so we don't depend on camera's saved state.
  // PixelType_Gvsp_BayerRG8 = 0x01080009 — the camera's native Bayer format,
  // which GetImageForBGR converts to BGR in software.
  ret = MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
  log_step("SetEnumValue(PixelFormat=BayerRG8)", ret);

  // Set image parameters
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

  // Step 6: Start grabbing
  ret = MV_CC_StartGrabbing(handle_);
  log_step("StartGrabbing", ret);
  if (ret != MV_OK) return;

  // Step 7: Verify frame dimensions
  MVCC_INTVALUE width_info{};
  MVCC_INTVALUE height_info{};
  ret = MV_CC_GetIntValue(handle_, "Width", &width_info);
  log_step("GetIntValue(Width)", ret);
  if (ret != MV_OK) return;
  ret = MV_CC_GetIntValue(handle_, "Height", &height_info);
  log_step("GetIntValue(Height)", ret);
  if (ret != MV_OK) return;

  auto frame_width = width_info.nCurValue;
  auto frame_height = height_info.nCurValue;

  capture_thread_ = std::thread{[this, frame_width, frame_height] {
    tools::logger()->info("[{}] Capture thread started", device_name_);

    capturing_ = true;

    auto log_mode_snapshot = [this] {
      MVCC_ENUMVALUE trigger_mode{};
      MVCC_ENUMVALUE trigger_source{};
      MVCC_ENUMVALUE acquisition_mode{};
      MVCC_ENUMVALUE pixel_format{};

      auto log_enum = [this](const char * key, MVCC_ENUMVALUE & value) {
        unsigned int ret = MV_CC_GetEnumValue(handle_, key, &value);
        if (ret != MV_OK) {
          tools::logger()->warn("[{}] MV_CC_GetEnumValue({}) failed: {:#x}", device_name_, key, ret);
        }
      };

      log_enum("TriggerMode", trigger_mode);
      log_enum("TriggerSource", trigger_source);
      log_enum("AcquisitionMode", acquisition_mode);
      log_enum("PixelFormat", pixel_format);

      tools::logger()->warn(
        "[{}] No frame received yet — serial='{}' trigger_mode={} trigger_source={} acquisition_mode={} pixel_format={} frame_rate={} grab_timeout_ms={} fetch_interval_ms={}",
        device_name_, serial_number_.empty() ? std::string("N/A") : serial_number_,
        trigger_mode.nCurValue, trigger_source.nCurValue, acquisition_mode.nCurValue,
        pixel_format.nCurValue, frame_rate_, grab_timeout_ms_, fetch_interval_ms_);
    };

    int consecutive_nodata = 0;
    bool has_last_frame_num = false;
    unsigned int last_frame_num = 0;

    while (!capture_quit_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(fetch_interval_ms_));

      unsigned int ret;
      unsigned int nMsec = grab_timeout_ms_;
      MV_FRAME_OUT_INFO_EX frame_info;
      memset(&frame_info, 0, sizeof(frame_info));
      cv::Mat img;

      img.create(frame_height, frame_width, CV_8UC3);
      ret = MV_CC_GetImageForBGR(handle_, img.data, img.total() * img.elemSize(), &frame_info, nMsec);
      if (ret != MV_OK) {
        if (ret == MV_E_NODATA || ret == MV_E_GC_TIMEOUT) {
          consecutive_nodata++;
          if (!has_last_frame_num && consecutive_nodata == 1) {
            log_mode_snapshot();
          }
          if (consecutive_nodata % 50 == 1) {
            tools::logger()->warn(
              "[{}] GetImageForBGR transient no-data/timeout: {:#x}, last frame num={}",
              device_name_, ret, has_last_frame_num ? std::to_string(last_frame_num) : std::string("N/A"));
          }
          std::this_thread::sleep_for(5ms);
          continue;
        }

        tools::logger()->warn("[{}] MV_CC_GetImageForBGR fatal error: {:#x}", device_name_, ret);
        break;
      }

      consecutive_nodata = 0;

      if (has_last_frame_num && frame_info.nFrameNum != last_frame_num + 1) {
        tools::logger()->warn(
          "[{}] Frame jump: last={} current={} delta={}", device_name_, last_frame_num,
          frame_info.nFrameNum, frame_info.nFrameNum - last_frame_num);
      }

      last_frame_num = frame_info.nFrameNum;
      has_last_frame_num = true;

      auto timestamp = std::chrono::steady_clock::now();

      queue_.push({img, timestamp});
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