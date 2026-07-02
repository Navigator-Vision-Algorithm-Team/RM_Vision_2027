#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(
  double exposure_ms, double gain, const std::string & vid_pid,
  const std::string & device_name, int device_index, const std::string & serial_number)
: exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false),
  vid_(-1), pid_(-1), handle_(nullptr), device_index_(device_index),
  serial_number_(serial_number)
{
  device_name_ = device_name;
  set_vid_pid(vid_pid);
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
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
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
  MV_CC_SetFrameRate(handle_, 30);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    MV_FRAME_OUT raw;
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;

    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      unsigned int ret;
      unsigned int nMsec = 100;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      const auto & frame_info = raw.stFrameInfo;
      auto pixel_type = frame_info.enPixelType;
      cv::Mat img;

      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> bayer_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR}};
      auto it = bayer_map.find(pixel_type);
      if (it != bayer_map.end()) {
        cv::Mat raw_mat(cv::Size(frame_info.nWidth, frame_info.nHeight), CV_8UC1, raw.pBufAddr);
        cv::cvtColor(raw_mat, img, it->second);
      } else if (pixel_type == PixelType_Gvsp_BGR8_Packed) {
        img = cv::Mat(cv::Size(frame_info.nWidth, frame_info.nHeight), CV_8UC3, raw.pBufAddr).clone();
      } else if (pixel_type == PixelType_Gvsp_RGB8_Packed) {
        cv::Mat raw_mat(cv::Size(frame_info.nWidth, frame_info.nHeight), CV_8UC3, raw.pBufAddr);
        cv::cvtColor(raw_mat, img, cv::COLOR_RGB2BGR);
      } else {
        cv::Mat raw_mat(cv::Size(frame_info.nWidth, frame_info.nHeight), CV_8UC1, raw.pBufAddr);
        cv::cvtColor(raw_mat, img, cv::COLOR_GRAY2BGR);
      }

      queue_.push({img, timestamp});

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  if (!handle_) return;

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
    return;
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