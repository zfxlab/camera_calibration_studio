#include "camera_calibration_studio/hik_camera.hpp"

#include <MvCameraControl.h>
#include <MvErrorDefine.h>

#include <algorithm>
#include <cstring>
#include <sstream>

namespace calibration_studio {
namespace {

template <std::size_t Size>
std::string boundedString(const unsigned char (&value)[Size]) {
  const auto* begin = reinterpret_cast<const char*>(value);
  const auto* end = std::find(begin, begin + Size, '\0');
  return std::string(begin, end);
}

CameraDescriptor descriptorFrom(const MV_CC_DEVICE_INFO& info) {
  CameraDescriptor result;
  if (info.nTLayerType == MV_GIGE_DEVICE) {
    result.transport = "gige";
    result.serial_number = boundedString(info.SpecialInfo.stGigEInfo.chSerialNumber);
    result.model_name = boundedString(info.SpecialInfo.stGigEInfo.chModelName);
    result.user_name = boundedString(info.SpecialInfo.stGigEInfo.chUserDefinedName);
  } else {
    result.transport = "usb";
    result.serial_number = boundedString(info.SpecialInfo.stUsb3VInfo.chSerialNumber);
    result.model_name = boundedString(info.SpecialInfo.stUsb3VInfo.chModelName);
    result.user_name = boundedString(info.SpecialInfo.stUsb3VInfo.chUserDefinedName);
  }
  return result;
}

std::string sdkError(const int code) {
  std::ostringstream stream;
  stream << "MVS error 0x" << std::hex << std::uppercase << static_cast<unsigned int>(code);
  return stream.str();
}

class FrameGuard {
 public:
  FrameGuard(void* handle, MV_FRAME_OUT& frame) : handle_(handle), frame_(frame) {}
  ~FrameGuard() { MV_CC_FreeImageBuffer(handle_, &frame_); }
 private:
  void* handle_;
  MV_FRAME_OUT& frame_;
};

}  // namespace

HikCamera::HikCamera() = default;

HikCamera::~HikCamera() { close(); }

std::vector<CameraDescriptor> HikCamera::enumerate(std::string& error) {
  MV_CC_DEVICE_INFO_LIST list{};
  const int status = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &list);
  if (status != MV_OK) {
    error = "camera enumeration failed: " + sdkError(status);
    return {};
  }
  std::vector<CameraDescriptor> result;
  for (unsigned int index = 0; index < list.nDeviceNum; ++index) {
    if (list.pDeviceInfo[index] != nullptr) {
      result.push_back(descriptorFrom(*list.pDeviceInfo[index]));
    }
  }
  error.clear();
  return result;
}

bool HikCamera::open(const std::string& serial_number, const CameraSettings& settings,
                     std::string& error) {
  close();
  MV_CC_DEVICE_INFO_LIST list{};
  int status = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &list);
  if (status != MV_OK) {
    error = "camera enumeration failed: " + sdkError(status);
    return false;
  }
  const MV_CC_DEVICE_INFO* selected = nullptr;
  for (unsigned int index = 0; index < list.nDeviceNum; ++index) {
    if (list.pDeviceInfo[index] != nullptr &&
        descriptorFrom(*list.pDeviceInfo[index]).serial_number == serial_number) {
      selected = list.pDeviceInfo[index];
      break;
    }
  }
  if (selected == nullptr) {
    error = "camera serial number not found: " + serial_number;
    return false;
  }
  status = MV_CC_CreateHandle(&handle_, selected);
  if (status != MV_OK) {
    handle_ = nullptr;
    error = "create camera handle failed: " + sdkError(status);
    return false;
  }
  status = MV_CC_OpenDevice(handle_);
  if (status != MV_OK) {
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    error = "open camera failed: " + sdkError(status);
    return false;
  }

  const auto fail = [this, &error](const std::string& operation, const int code) {
    error = operation + " failed: " + sdkError(code);
    close();
    return false;
  };
  if ((status = MV_CC_SetEnumValueByString(handle_, "TriggerMode", "Off")) != MV_OK) {
    return fail("disable trigger mode", status);
  }
  if ((status = MV_CC_SetEnumValueByString(handle_, "ExposureAuto", "Off")) != MV_OK) {
    return fail("disable auto exposure", status);
  }
  if ((status = MV_CC_SetFloatValue(handle_, "ExposureTime",
                                    static_cast<float>(settings.exposure_time_us))) != MV_OK) {
    return fail("set exposure", status);
  }
  if ((status = MV_CC_SetEnumValueByString(handle_, "GainAuto", "Off")) != MV_OK) {
    return fail("disable auto gain", status);
  }
  if ((status = MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(settings.gain))) != MV_OK) {
    return fail("set gain", status);
  }
  if (settings.frame_rate > 0.0) {
    if ((status = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true)) != MV_OK) {
      return fail("enable frame-rate control", status);
    }
    if ((status = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate",
                                      static_cast<float>(settings.frame_rate))) != MV_OK) {
      return fail("set frame rate", status);
    }
  }
  if ((status = MV_CC_SetImageNodeNum(handle_, settings.sdk_buffer_count)) != MV_OK) {
    return fail("set SDK buffer count", status);
  }
  status = MV_CC_StartGrabbing(handle_);
  if (status != MV_OK) {
    return fail("start acquisition", status);
  }

  serial_number_ = serial_number;
  stop_requested_.store(false);
  running_.store(true);
  setLastError({});
  capture_thread_ = std::thread(&HikCamera::captureLoop, this);
  error.clear();
  return true;
}

void HikCamera::close() noexcept {
  stop_requested_.store(true);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  running_.store(false);
  if (handle_ != nullptr) {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
  serial_number_.clear();
  std::lock_guard<std::mutex> lock(frame_mutex_);
  latest_frame_ = {};
}

std::string HikCamera::serialNumber() const { return serial_number_; }

bool HikCamera::latestFrame(CapturedFrame& frame) const {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  if (latest_frame_.image.empty()) {
    return false;
  }
  frame = latest_frame_;
  frame.image = latest_frame_.image.clone();
  return true;
}

std::string HikCamera::lastError() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

void HikCamera::setLastError(std::string value) {
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = std::move(value);
}

void HikCamera::captureLoop() {
  std::uint64_t sequence = 0;
  while (!stop_requested_.load()) {
    MV_FRAME_OUT raw{};
    const int status = MV_CC_GetImageBuffer(handle_, &raw, 250);
    const auto status_code = static_cast<unsigned int>(status);
    if (status_code == MV_E_NODATA || status_code == MV_E_GC_TIMEOUT) {
      continue;
    }
    if (status != MV_OK) {
      setLastError("get image failed: " + sdkError(status));
      break;
    }
    FrameGuard guard(handle_, raw);
    const unsigned int width = raw.stFrameInfo.nWidth;
    const unsigned int height = raw.stFrameInfo.nHeight;
    if (width == 0 || height == 0 || raw.pBufAddr == nullptr) {
      continue;
    }
    cv::Mat mono(static_cast<int>(height), static_cast<int>(width), CV_8UC1);
    const std::size_t output_size = static_cast<std::size_t>(width) * height;
    if (raw.stFrameInfo.enPixelType == PixelType_Gvsp_Mono8 &&
        raw.stFrameInfo.nFrameLen >= output_size) {
      std::memcpy(mono.data, raw.pBufAddr, output_size);
    } else {
      MV_CC_PIXEL_CONVERT_PARAM conversion{};
      conversion.nWidth = width;
      conversion.nHeight = height;
      conversion.enSrcPixelType = raw.stFrameInfo.enPixelType;
      conversion.pSrcData = raw.pBufAddr;
      conversion.nSrcDataLen = raw.stFrameInfo.nFrameLen;
      conversion.enDstPixelType = PixelType_Gvsp_Mono8;
      conversion.pDstBuffer = mono.data;
      conversion.nDstBufferSize = static_cast<unsigned int>(output_size);
      const int conversion_status = MV_CC_ConvertPixelType(handle_, &conversion);
      if (conversion_status != MV_OK) {
        setLastError("pixel conversion failed: " + sdkError(conversion_status));
        continue;
      }
    }
    CapturedFrame frame;
    frame.image = std::move(mono);
    frame.sequence = ++sequence;
    frame.received_at = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = std::move(frame);
    }
  }
  running_.store(false);
}

}  // namespace calibration_studio
