#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(
  double exposure_ms, double gain, const std::string & vid_pid,
  const std::string & device_name, int device_index, const std::string & serial_number,
  double frame_rate, unsigned int grab_timeout_ms, unsigned int transfer_size)
: exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false),
  vid_(-1), pid_(-1), handle_(nullptr), device_index_(device_index),
  serial_number_(serial_number), frame_rate_(frame_rate),
  fetch_interval_ms_(frame_rate > 0.0 ? static_cast<unsigned int>(std::max(1.0, 1000.0 / frame_rate)) : 1),
  grab_timeout_ms_(grab_timeout_ms),
  transfer_size_(transfer_size)
{
  device_name_ = device_name;
  set_vid_pid(vid_pid);

  auto serial_display = serial_number_.empty() ? std::string("N/A") : serial_number_;
  auto vid_display = vid_ >= 0 ? fmt::format("{:04x}", vid_) : std::string("N/A");
  auto pid_display = pid_ >= 0 ? fmt::format("{:04x}", pid_) : std::string("N/A");
  tools::logger()->info(
    "HikRobot config summary: device='{}' serial='{}' index={} vid_pid={}:{} exposure_ms={} gain={} frame_rate={} fetch_interval_ms={} grab_timeout_ms={} transfer_size={}",
    device_name_.empty() ? std::string("N/A") : device_name_, serial_display, device_index_,
    vid_display, pid_display, exposure_ms, gain_, frame_rate_, fetch_interval_ms_,
    grab_timeout_ms_, transfer_size_);

  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
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

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  // Log all found cameras for identification (first enumeration only)
  static bool first_enum = true;
  if (first_enum) {
    for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
      auto * usb_info = &device_list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo;
      tools::logger()->info("Found camera [{}]: serial='{}' model='{}'", i,
                            reinterpret_cast<char *>(usb_info->chSerialNumber),
                            reinterpret_cast<char *>(usb_info->chModelName));
    }
    first_enum = false;
  }

  // Match camera by serial number (preferred) or fall back to device index
  int target_index = -1;
  if (!serial_number_.empty()) {
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
        "Camera with serial '{}' not found ({} device(s) enumerated)", serial_number_,
        device_list.nDeviceNum);
      return;
    }
  } else {
    if (device_index_ >= static_cast<int>(device_list.nDeviceNum)) {
      tools::logger()->warn(
        "Camera index {} out of range (found {} device(s))", device_index_,
        device_list.nDeviceNum);
      return;
    }
    target_index = device_index_;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[target_index]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return;
  }

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);

  if (frame_rate_ > 0.0) {
    unsigned int frame_rate_ret = MV_CC_SetFrameRate(handle_, static_cast<float>(frame_rate_));
    if (frame_rate_ret != MV_OK) {
      tools::logger()->warn("MV_CC_SetFrameRate({}) failed: {:#x}", frame_rate_, frame_rate_ret);
    }
  }

  if (transfer_size_ > 0) {
    unsigned int transfer_ret = MV_USB_SetTransferSize(handle_, transfer_size_);
    if (transfer_ret != MV_OK) {
      tools::logger()->warn(
        "MV_USB_SetTransferSize({}) failed: {:#x}", transfer_size_, transfer_ret);
    }
  }

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  MVCC_INTVALUE width_info{};
  MVCC_INTVALUE height_info{};
  ret = MV_CC_GetIntValue(handle_, "Width", &width_info);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_GetIntValue(Width) failed: {:#x}", ret);
    return;
  }
  ret = MV_CC_GetIntValue(handle_, "Height", &height_info);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_GetIntValue(Height) failed: {:#x}", ret);
    return;
  }

  auto frame_width = width_info.nCurValue;
  auto frame_height = height_info.nCurValue;

  capture_thread_ = std::thread{[this, frame_width, frame_height] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    auto log_mode_snapshot = [this] {
      MVCC_ENUMVALUE trigger_mode{};
      MVCC_ENUMVALUE trigger_source{};
      MVCC_ENUMVALUE acquisition_mode{};

      auto log_enum = [this](const char * key, MVCC_ENUMVALUE & value) {
        unsigned int ret = MV_CC_GetEnumValue(handle_, key, &value);
        if (ret != MV_OK) {
          tools::logger()->warn("MV_CC_GetEnumValue({}) failed: {:#x}", key, ret);
        }
      };

      log_enum("TriggerMode", trigger_mode);
      log_enum("TriggerSource", trigger_source);
      log_enum("AcquisitionMode", acquisition_mode);

      tools::logger()->warn(
        "HikRobot has not received any frame yet. device='{}' serial='{}' index={} trigger_mode={} trigger_source={} acquisition_mode={} frame_rate={} grab_timeout_ms={} fetch_interval_ms={}",
        device_name_, serial_number_.empty() ? std::string("N/A") : serial_number_, device_index_,
        trigger_mode.nCurValue, trigger_source.nCurValue, acquisition_mode.nCurValue, frame_rate_,
        grab_timeout_ms_, fetch_interval_ms_);
    };

    int consecutive_nodata = 0;
    bool has_last_frame_num = false;
    unsigned int last_frame_num = 0;

    while (!capture_quit_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(fetch_interval_ms_));

      unsigned int ret;
      unsigned int nMsec = grab_timeout_ms_;
      MV_FRAME_OUT_INFO_EX frame_info;
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
              "MV_CC_GetImageForBGR returned transient no-data/timeout: {:#x}, last frame num={}",
              ret, has_last_frame_num ? std::to_string(last_frame_num) : std::string("N/A"));
          }
          std::this_thread::sleep_for(5ms);
          continue;
        }

        tools::logger()->warn("MV_CC_GetImageForBGR failed: {:#x}", ret);
        break;
      }

      consecutive_nodata = 0;

      if (has_last_frame_num && frame_info.nFrameNum != last_frame_num + 1) {
        tools::logger()->warn(
          "HikRobot frame jump detected: last={} current={} delta={}", last_frame_num,
          frame_info.nFrameNum, frame_info.nFrameNum - last_frame_num);
      }

      last_frame_num = frame_info.nFrameNum;
      has_last_frame_num = true;

      auto timestamp = std::chrono::steady_clock::now();

      queue_.push({img, timestamp});
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;

  if (handle_) {
    unsigned int ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK) {
      tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    }
  }

  if (capture_thread_.joinable()) capture_thread_.join();

  if (handle_) {
    unsigned int ret = MV_CC_CloseDevice(handle_);
    if (ret != MV_OK) {
      tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    }

    ret = MV_CC_DestroyHandle(handle_);
    if (ret != MV_OK) {
      tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
    }

    handle_ = nullptr;
  }
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io