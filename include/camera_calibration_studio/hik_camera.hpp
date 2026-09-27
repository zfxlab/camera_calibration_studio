#pragma once

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace calibration_studio {

struct CameraDescriptor {
  std::string serial_number;
  std::string model_name;
  std::string user_name;
  std::string transport;
};

struct CameraSettings {
  double exposure_time_us{10000.0};
  double gain{0.0};
  double frame_rate{10.0};
  unsigned int sdk_buffer_count{4};
};

struct CapturedFrame {
  cv::Mat image;
  std::uint64_t sequence{0};
  std::chrono::steady_clock::time_point received_at{};
};

class HikCamera {
 public:
  HikCamera();
  ~HikCamera();
  HikCamera(const HikCamera&) = delete;
  HikCamera& operator=(const HikCamera&) = delete;

  static std::vector<CameraDescriptor> enumerate(std::string& error);
  bool open(const std::string& serial_number, const CameraSettings& settings, std::string& error);
  void close() noexcept;
  [[nodiscard]] bool running() const noexcept { return running_.load(); }
  [[nodiscard]] std::string serialNumber() const;
  bool latestFrame(CapturedFrame& frame) const;
  [[nodiscard]] std::string lastError() const;

 private:
  void captureLoop();
  void setLastError(std::string value);

  void* handle_{nullptr};
  std::string serial_number_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  mutable std::mutex frame_mutex_;
  CapturedFrame latest_frame_;
  mutable std::mutex error_mutex_;
  std::string last_error_;
};

}  // namespace calibration_studio

