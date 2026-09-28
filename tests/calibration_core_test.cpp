#include "camera_calibration_studio/calibration.hpp"
#include "camera_calibration_studio/dataset.hpp"
#include "camera_calibration_studio/focus_analyzer.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

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
  cv::Mat sharp(256, 256, CV_8UC1);
  for (int row = 0; row < sharp.rows; ++row) {
    for (int column = 0; column < sharp.cols; ++column) {
      sharp.at<unsigned char>(row, column) =
          ((row / 16 + column / 16) % 2 == 0) ? 0 : 255;
    }
  }
  cv::Mat blurred;
  cv::GaussianBlur(sharp, blurred, cv::Size(21, 21), 5.0);
  calibration_studio::FocusAnalyzer sharp_analyzer(1.0);
  calibration_studio::FocusAnalyzer blurred_analyzer(1.0);
  const auto sharp_result = sharp_analyzer.process(sharp, {});
  const auto blurred_result = blurred_analyzer.process(blurred, {});
  if (!(sharp_result.current_score > blurred_result.current_score * 2.0)) {
    std::cerr << "sharp image must score above blurred image\n";
    return 1;
  }
  calibration_studio::FocusAnalyzer constant_analyzer(1.0);
  const auto constant_result = constant_analyzer.process(
      cv::Mat(128, 128, CV_8UC1, cv::Scalar(127)), {});
  if (constant_result.current_score != 0.0 || constant_result.roi.area() <= 0) {
    std::cerr << "constant image focus result is invalid\n";
    return 1;
  }
  return 0;
}
