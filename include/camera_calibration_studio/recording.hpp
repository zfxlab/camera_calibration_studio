#pragma once

#include "camera_calibration_studio/hik_camera.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace calibration_studio {

struct RecordingStats {
  std::size_t accepted{0};
  std::size_t written{0};
  std::size_t dropped{0};
  std::size_t pending{0};
};

class ImageSequenceRecorder {
 public:
  ImageSequenceRecorder() = default;
  ~ImageSequenceRecorder();
  ImageSequenceRecorder(const ImageSequenceRecorder&) = delete;
  ImageSequenceRecorder& operator=(const ImageSequenceRecorder&) = delete;

  bool startMono(const std::filesystem::path& directory, const std::string& serial,
                 double recording_rate_hz, std::string& error);
  bool startStereo(const std::filesystem::path& directory,
                   const std::string& left_serial, const std::string& right_serial,
                   double recording_rate_hz, std::string& error);
  bool submitMono(const CapturedFrame& frame);
  bool submitStereo(const CapturedFrame& left, const CapturedFrame& right);
  void stop();

  [[nodiscard]] bool recording() const noexcept { return recording_.load(); }
  [[nodiscard]] std::filesystem::path directory() const;
  [[nodiscard]] RecordingStats stats() const;
  [[nodiscard]] std::string lastError() const;

 private:
  struct Packet {
    cv::Mat left;
    cv::Mat right;
    std::uint64_t left_sequence{0};
    std::uint64_t right_sequence{0};
    std::chrono::steady_clock::time_point left_time{};
    std::chrono::steady_clock::time_point right_time{};
  };

  bool start(const std::filesystem::path& directory, bool stereo,
             const std::string& left_serial, const std::string& right_serial,
             double recording_rate_hz, std::string& error);
  bool enqueue(Packet packet);
  void writeLoop();
  void setError(const std::string& error);

  static constexpr std::size_t kMaximumQueue = 24;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Packet> queue_;
  std::filesystem::path directory_;
  std::ofstream manifest_;
  std::thread worker_;
  std::chrono::steady_clock::duration minimum_period_{};
  std::chrono::steady_clock::time_point last_accepted_time_{};
  std::atomic<bool> recording_{false};
  bool stopping_{false};
  bool stereo_{false};
  std::size_t accepted_{0};
  std::size_t written_{0};
  std::size_t dropped_{0};
  std::string last_error_;
  std::uint64_t last_left_sequence_{0};
  std::uint64_t last_right_sequence_{0};
};

}  // namespace calibration_studio
