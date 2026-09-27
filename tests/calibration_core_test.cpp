#include "camera_calibration_studio/calibration.hpp"
#include "camera_calibration_studio/dataset.hpp"

#include <opencv2/core.hpp>

#include <filesystem>
#include <iostream>

int main() {
  calibration_studio::Intrinsics invalid;
  if (invalid.valid()) {
    std::cerr << "default intrinsics must be invalid\n";
    return 1;
  }
  calibration_studio::Intrinsics valid;
  valid.image_size = {1440, 1080};
  valid.camera_matrix = (cv::Mat_<double>(3, 3) << 1000.0, 0.0, 720.0,
                          0.0, 1000.0, 540.0, 0.0, 0.0, 1.0);
  valid.distortion = cv::Mat::zeros(1, 5, CV_64F);
  if (!valid.valid()) {
    std::cerr << "well-formed intrinsics must be valid\n";
    return 1;
  }
  const auto temporary = std::filesystem::temp_directory_path() /
                         "camera_calibration_studio_intrinsics_test.yaml";
  std::string error;
  if (!calibration_studio::saveIntrinsics(temporary, "test_camera", valid, error)) {
    std::cerr << "failed to save intrinsics: " << error << '\n';
    return 1;
  }
  calibration_studio::Intrinsics loaded;
  if (!calibration_studio::loadIntrinsics(temporary, loaded, error) ||
      cv::norm(valid.camera_matrix - loaded.camera_matrix) > 1e-12 ||
      cv::norm(valid.distortion - loaded.distortion) > 1e-12) {
    std::cerr << "intrinsics round trip failed: " << error << '\n';
    return 1;
  }
  std::filesystem::remove(temporary);
  if (!calibration_studio::listImages("/path/that/does/not/exist").empty()) {
    std::cerr << "missing directory must return no images\n";
    return 1;
  }
  return 0;
}
