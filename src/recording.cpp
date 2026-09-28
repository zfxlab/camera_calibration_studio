#include "camera_calibration_studio/recording.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <iomanip>
#include <sstream>

namespace calibration_studio {
namespace {

std::string indexedName(const std::size_t index) {
  std::ostringstream stream;
  stream << std::setw(6) << std::setfill('0') << index << ".png";
  return stream.str();
}

long long timestampNanoseconds(const std::chrono::steady_clock::time_point time) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

}  // namespace

ImageSequenceRecorder::~ImageSequenceRecorder() { stop(); }

bool ImageSequenceRecorder::startMono(const std::filesystem::path& directory,
                                      const std::string& serial,
                                      const double recording_rate_hz,
                                      std::string& error) {
  return start(directory, false, serial, {}, recording_rate_hz, error);
}

bool ImageSequenceRecorder::startStereo(const std::filesystem::path& directory,
                                        const std::string& left_serial,
                                        const std::string& right_serial,
                                        const double recording_rate_hz,
                                        std::string& error) {
  return start(directory, true, left_serial, right_serial, recording_rate_hz, error);
}

bool ImageSequenceRecorder::start(const std::filesystem::path& directory, const bool stereo,
                                  const std::string& left_serial,
                                  const std::string& right_serial,
                                  const double recording_rate_hz, std::string& error) {
  stop();
  if (directory.empty() || left_serial.empty() || (stereo && right_serial.empty()) ||
      !std::isfinite(recording_rate_hz) || recording_rate_hz <= 0.0) {
    error = "invalid recording configuration";
    return false;
  }
  try {
    std::filesystem::create_directories(directory / (stereo ? "left" : "frames"));
    if (stereo) std::filesystem::create_directories(directory / "right");
    manifest_.open(directory / "manifest.csv", std::ios::out | std::ios::trunc);
    if (!manifest_) {
      error = "cannot create recording manifest";
      return false;
    }
    manifest_ << (stereo
                      ? "index,left_file,right_file,left_sequence,right_sequence,"
                        "left_received_ns,right_received_ns,delta_ms\n"
                      : "index,file,sequence,received_ns\n");
    manifest_ << (stereo ? "# cameras," + left_serial + ',' + right_serial + '\n'
                         : "# camera," + left_serial + '\n');
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    directory_ = directory;
    stereo_ = stereo;
    stopping_ = false;
    queue_.clear();
    accepted_ = 0;
    written_ = 0;
    dropped_ = 0;
    last_error_.clear();
    last_accepted_time_ = {};
    minimum_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / recording_rate_hz));
  }
  recording_.store(true);
    last_left_sequence_ = 0;
    last_right_sequence_ = 0;
  worker_ = std::thread(&ImageSequenceRecorder::writeLoop, this);
  error.clear();
  return true;
}

bool ImageSequenceRecorder::submitMono(const CapturedFrame& frame) {
  if (!recording_.load() || stereo_ || frame.image.empty()) return false;
  Packet packet;
  packet.left = frame.image.clone();
  packet.left_sequence = frame.sequence;
  packet.left_time = frame.received_at;
  return enqueue(std::move(packet));
}

bool ImageSequenceRecorder::submitStereo(const CapturedFrame& left,
                                         const CapturedFrame& right) {
  if (!recording_.load() || !stereo_ || left.image.empty() || right.image.empty() ||
      left.image.size() != right.image.size()) {
    return false;
  }
  Packet packet;
  packet.left = left.image.clone();
  packet.right = right.image.clone();
  packet.left_sequence = left.sequence;
  packet.right_sequence = right.sequence;
  packet.left_time = left.received_at;
  packet.right_time = right.received_at;
  return enqueue(std::move(packet));
}

bool ImageSequenceRecorder::enqueue(Packet packet) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_ || !recording_.load()) return false;
  if (packet.left_sequence == last_left_sequence_ ||
      (stereo_ && packet.right_sequence == last_right_sequence_)) {
    return false;
  }
  if (last_accepted_time_ != std::chrono::steady_clock::time_point{} &&
      packet.left_time - last_accepted_time_ < minimum_period_) {
    return false;
  }
  if (queue_.size() >= kMaximumQueue) {
    ++dropped_;
    return false;
  }
  last_accepted_time_ = packet.left_time;
  last_left_sequence_ = packet.left_sequence;
  last_right_sequence_ = packet.right_sequence;
  queue_.push_back(std::move(packet));
  ++accepted_;
  condition_.notify_one();
  return true;
}

void ImageSequenceRecorder::stop() {
  recording_.store(false);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  condition_.notify_all();
  if (worker_.joinable()) worker_.join();
  if (manifest_.is_open()) manifest_.close();
}

std::filesystem::path ImageSequenceRecorder::directory() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return directory_;
}

RecordingStats ImageSequenceRecorder::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {accepted_, written_, dropped_, queue_.size()};
}

std::string ImageSequenceRecorder::lastError() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_error_;
}

void ImageSequenceRecorder::setError(const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_error_ = error;
}

void ImageSequenceRecorder::writeLoop() {
  while (true) {
    Packet packet;
    std::size_t index = 0;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
      if (queue_.empty()) {
        if (stopping_) break;
        continue;
      }
      packet = std::move(queue_.front());
      queue_.pop_front();
      index = written_ + 1;
    }

    const std::string name = indexedName(index);
    bool ok = false;
    try {
      if (stereo_) {
        ok = cv::imwrite((directory_ / "left" / name).string(), packet.left) &&
             cv::imwrite((directory_ / "right" / name).string(), packet.right);
      } else {
        ok = cv::imwrite((directory_ / "frames" / name).string(), packet.left);
      }
    } catch (const cv::Exception& exception) {
      setError(exception.what());
    }
    if (!ok) {
      setError("failed to write recording frame " + name);
      std::lock_guard<std::mutex> lock(mutex_);
      ++dropped_;
      continue;
    }

    const long long left_ns = timestampNanoseconds(packet.left_time);
    if (stereo_) {
      const long long right_ns = timestampNanoseconds(packet.right_time);
      const double delta_ms = std::abs(
          std::chrono::duration<double, std::milli>(packet.left_time - packet.right_time).count());
      manifest_ << index << ",left/" << name << ",right/" << name << ','
                << packet.left_sequence << ',' << packet.right_sequence << ',' << left_ns << ','
                << right_ns << ',' << delta_ms << '\n';
    } else {
      manifest_ << index << ",frames/" << name << ',' << packet.left_sequence << ','
                << left_ns << '\n';
    }
    manifest_.flush();
    std::lock_guard<std::mutex> lock(mutex_);
    ++written_;
  }
}

}  // namespace calibration_studio
