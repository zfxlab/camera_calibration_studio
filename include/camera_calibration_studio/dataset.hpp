#pragma once

#include <opencv2/core.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace calibration_studio {

std::vector<std::filesystem::path> listImages(const std::filesystem::path& directory);

class DatasetWriter {
 public:
  explicit DatasetWriter(std::filesystem::path root);

  [[nodiscard]] const std::filesystem::path& root() const noexcept { return root_; }
  bool saveMono(const std::string& camera_label, const cv::Mat& image,
                std::filesystem::path& saved_path, std::string& error);
  bool saveStereoPair(const cv::Mat& left, const cv::Mat& right,
                      std::filesystem::path& left_path,
                      std::filesystem::path& right_path,
                      std::string& error);

 private:
  std::filesystem::path root_;
  std::size_t nextIndex(const std::filesystem::path& directory) const;
};

}  // namespace calibration_studio

