#include "camera_calibration_studio/dataset.hpp"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>

namespace calibration_studio {
namespace {

bool isImageExtension(std::string extension) {
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
  return extension == ".png" || extension == ".tif" || extension == ".tiff" ||
         extension == ".bmp" || extension == ".jpg" || extension == ".jpeg";
}

std::string indexedName(const std::size_t index) {
  std::ostringstream stream;
  stream << std::setw(6) << std::setfill('0') << index << ".png";
  return stream.str();
}

}  // namespace

std::vector<std::filesystem::path> listImages(const std::filesystem::path& directory) {
  std::vector<std::filesystem::path> result;
  std::error_code code;
  if (!std::filesystem::is_directory(directory, code)) {
    return result;
  }
  for (const auto& entry : std::filesystem::directory_iterator(directory, code)) {
    if (entry.is_regular_file() && isImageExtension(entry.path().extension().string())) {
      result.push_back(entry.path());
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

DatasetWriter::DatasetWriter(std::filesystem::path root) : root_(std::move(root)) {}

std::size_t DatasetWriter::nextIndex(const std::filesystem::path& directory) const {
  std::size_t candidate = 1;
  while (std::filesystem::exists(directory / indexedName(candidate))) {
    ++candidate;
  }
  return candidate;
}

bool DatasetWriter::saveMono(const std::string& camera_label, const cv::Mat& image,
                             std::filesystem::path& saved_path, std::string& error) {
  if (image.empty()) {
    error = "cannot save an empty mono image";
    return false;
  }
  try {
    const auto directory = root_ / "mono" / camera_label;
    std::filesystem::create_directories(directory);
    saved_path = directory / indexedName(nextIndex(directory));
    if (!cv::imwrite(saved_path.string(), image)) {
      error = "OpenCV failed to write " + saved_path.string();
      return false;
    }
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
  error.clear();
  return true;
}

bool DatasetWriter::saveStereoPair(const cv::Mat& left, const cv::Mat& right,
                                   std::filesystem::path& left_path,
                                   std::filesystem::path& right_path,
                                   std::string& error) {
  if (left.empty() || right.empty()) {
    error = "cannot save an empty stereo image";
    return false;
  }
  if (left.size() != right.size()) {
    error = "left and right image sizes differ";
    return false;
  }
  try {
    const auto left_directory = root_ / "stereo" / "left";
    const auto right_directory = root_ / "stereo" / "right";
    std::filesystem::create_directories(left_directory);
    std::filesystem::create_directories(right_directory);
    const auto index = std::max(nextIndex(left_directory), nextIndex(right_directory));
    const auto name = indexedName(index);
    left_path = left_directory / name;
    right_path = right_directory / name;
    const auto left_temporary = left_directory / (name + ".tmp.png");
    const auto right_temporary = right_directory / (name + ".tmp.png");
    if (!cv::imwrite(left_temporary.string(), left) ||
        !cv::imwrite(right_temporary.string(), right)) {
      std::filesystem::remove(left_temporary);
      std::filesystem::remove(right_temporary);
      error = "OpenCV failed to write the stereo pair";
      return false;
    }
    std::filesystem::rename(left_temporary, left_path);
    std::filesystem::rename(right_temporary, right_path);
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
  error.clear();
  return true;
}

}  // namespace calibration_studio

