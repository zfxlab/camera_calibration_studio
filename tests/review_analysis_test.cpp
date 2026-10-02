#include "camera_calibration_studio/review_analysis.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

cv::Mat circleGrid(const cv::Size size, const cv::Point origin) {
  cv::Mat image(size, CV_8UC1, cv::Scalar(255));
  for (int row = 0; row < 7; ++row) {
    for (int column = 0; column < 7; ++column) {
      cv::circle(image, origin + cv::Point(column * 50, row * 50), 13,
                 cv::Scalar(0), cv::FILLED, cv::LINE_AA);
    }
  }
  return image;
}

cv::Mat projectedCircleGrid(const double rotation_x_deg, const double rotation_y_deg,
                            const double offset_x, const double offset_y) {
  constexpr double to_radians = CV_PI / 180.0;
  const double x = rotation_x_deg * to_radians;
  const double y = rotation_y_deg * to_radians;
  const cv::Matx33d rotation_x(1.0, 0.0, 0.0,
                               0.0, std::cos(x), -std::sin(x),
                               0.0, std::sin(x), std::cos(x));
  const cv::Matx33d rotation_y(std::cos(y), 0.0, std::sin(y),
                               0.0, 1.0, 0.0,
                               -std::sin(y), 0.0, std::cos(y));
  cv::Mat rotation(rotation_y * rotation_x);
  cv::Mat rotation_vector;
  cv::Rodrigues(rotation, rotation_vector);
  const cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << 900.0, 0.0, 400.0,
                                0.0, 900.0, 300.0,
                                0.0, 0.0, 1.0);
  const cv::Vec3d translation(-0.09 + offset_x, -0.09 + offset_y, 0.75);
  std::vector<cv::Point3f> objects;
  for (int row = 0; row < 7; ++row) {
    for (int column = 0; column < 7; ++column) {
      objects.emplace_back(static_cast<float>(column * 0.03),
                           static_cast<float>(row * 0.03), 0.0F);
    }
  }
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(objects, rotation_vector, translation, camera_matrix, cv::Mat(),
                    image_points);
  cv::Mat image(600, 800, CV_8UC1, cv::Scalar(255));
  for (const auto& point : image_points) {
    cv::circle(image, point, 10, cv::Scalar(0), cv::FILLED, cv::LINE_AA);
  }
  return image;
}

}  // namespace

int main() {
  const auto directory = std::filesystem::temp_directory_path() /
                         "camera_calibration_studio_review_analysis_test";
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory);
  const std::vector<std::filesystem::path> paths{
      directory / "base.png", directory / "duplicate.png", directory / "diverse.png",
      directory / "wrong_size.png", directory / "no_grid.png", directory / "missing.png"};
  cv::imwrite(paths[0].string(), circleGrid({800, 600}, {160, 140}));
  cv::imwrite(paths[1].string(), circleGrid({800, 600}, {163, 143}));
  cv::imwrite(paths[2].string(), circleGrid({800, 600}, {280, 180}));
  cv::imwrite(paths[3].string(), circleGrid({640, 480}, {120, 90}));
  cv::imwrite(paths[4].string(), cv::Mat(600, 800, CV_8UC1, cv::Scalar(127)));

  const auto results = calibration_studio::analyzeMonoFrames(paths);
  if (results.size() != paths.size()) {
    std::cerr << "analysis result count differs from input count\n";
    return 1;
  }
  std::size_t recommended = 0;
  std::size_t duplicates = 0;
  std::size_t size_mismatches = 0;
  std::size_t missing_patterns = 0;
  std::size_t unreadable = 0;
  for (const auto& result : results) {
    if (result.recommended) ++recommended;
    if (result.decision == calibration_studio::MonoFrameDecision::duplicate) ++duplicates;
    if (result.decision == calibration_studio::MonoFrameDecision::size_mismatch) {
      ++size_mismatches;
    }
    if (result.decision == calibration_studio::MonoFrameDecision::pattern_not_found) {
      ++missing_patterns;
    }
    if (result.decision == calibration_studio::MonoFrameDecision::unreadable) ++unreadable;
  }
  if (recommended != 2 || duplicates != 1 || size_mismatches != 1 ||
      missing_patterns != 1 || unreadable != 1 || results[1].duplicate_of.empty()) {
    std::cerr << "unexpected analysis decisions: recommended=" << recommended
              << ", duplicates=" << duplicates << ", size=" << size_mismatches
              << ", no-grid=" << missing_patterns << ", unreadable=" << unreadable << '\n';
    return 1;
  }

  calibration_studio::MonoReviewOptions limited_options;
  limited_options.target_frame_count = 1;
  const auto limited = calibration_studio::analyzeMonoFrames(paths, limited_options);
  recommended = 0;
  std::size_t redundant = 0;
  for (const auto& result : limited) {
    if (result.recommended) ++recommended;
    if (result.decision == calibration_studio::MonoFrameDecision::redundant) ++redundant;
  }
  if (recommended != 1 || redundant != 1) {
    std::cerr << "target count was not enforced: recommended=" << recommended
              << ", redundant=" << redundant << '\n';
    return 1;
  }

  const std::vector<std::pair<double, double>> tilts{
      {0.0, 0.0},   {20.0, 0.0},  {-20.0, 0.0}, {0.0, 20.0},
      {0.0, -20.0}, {25.0, 15.0}, {-25.0, 15.0}, {25.0, -15.0},
      {-25.0, -15.0}, {10.0, 30.0}, {-10.0, -30.0}, {5.0, -20.0}};
  std::vector<std::filesystem::path> pose_paths;
  for (std::size_t index = 0; index < tilts.size(); ++index) {
    const auto path = directory / ("pose_" + std::to_string(index) + ".png");
    const double offset_x = (static_cast<int>(index % 3) - 1) * 0.015;
    const double offset_y = (static_cast<int>(index % 2) == 0 ? -1.0 : 1.0) * 0.01;
    cv::imwrite(path.string(), projectedCircleGrid(tilts[index].first, tilts[index].second,
                                                   offset_x, offset_y));
    pose_paths.push_back(path);
  }
  calibration_studio::MonoReviewOptions pose_options;
  pose_options.target_frame_count = 8;
  const auto pose_results = calibration_studio::analyzeMonoFrames(pose_paths, pose_options);
  std::size_t estimated_poses = 0;
  bool selected_positive_x = false;
  bool selected_negative_x = false;
  bool selected_positive_y = false;
  bool selected_negative_y = false;
  for (const auto& result : pose_results) {
    if (result.pose_estimated) ++estimated_poses;
    if (!result.recommended) continue;
    selected_positive_x = selected_positive_x || result.tilt_x_deg > 10.0;
    selected_negative_x = selected_negative_x || result.tilt_x_deg < -10.0;
    selected_positive_y = selected_positive_y || result.tilt_y_deg > 10.0;
    selected_negative_y = selected_negative_y || result.tilt_y_deg < -10.0;
  }
  if (estimated_poses < 8 || !selected_positive_x || !selected_negative_x ||
      !selected_positive_y || !selected_negative_y) {
    std::cerr << "pose-aware selection did not preserve all tilt directions: poses="
              << estimated_poses << '\n';
    return 1;
  }
  std::filesystem::remove_all(directory);
  return 0;
}
