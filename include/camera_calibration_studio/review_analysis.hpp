#pragma once

#include "camera_calibration_studio/calibration.hpp"

#include <opencv2/core.hpp>

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace calibration_studio {

enum class MonoFrameDecision {
  keep,
  duplicate,
  redundant,
  unreadable,
  size_mismatch,
  pattern_not_found,
};

struct MonoFrameAnalysis {
  std::filesystem::path path;
  cv::Size image_size;
  MonoFrameDecision decision{MonoFrameDecision::unreadable};
  bool recommended{false};
  bool low_sharpness{false};
  bool exposure_warning{false};
  int detected_points{0};
  double sharpness{0.0};
  double mean_brightness{0.0};
  double dark_ratio{0.0};
  double bright_ratio{0.0};
  cv::Point2d normalized_center;
  double image_coverage{0.0};
  double board_angle_deg{0.0};
  bool pose_estimated{false};
  double tilt_x_deg{0.0};
  double tilt_y_deg{0.0};
  double total_tilt_deg{0.0};
  double pose_reprojection_error_px{0.0};
  std::filesystem::path duplicate_of;
  double duplicate_distance{0.0};
};

struct MonoReviewOptions {
  MonoOptions target;
  std::size_t target_frame_count{30};
  std::size_t preliminary_calibration_limit{40};
  // RMS movement of corresponding grid points in normalized image coordinates.
  double duplicate_distance_threshold{0.025};
  // Dimensionless distance in position/scale/rotation/tilt pose space.
  double pose_duplicate_distance_threshold{0.45};
  double relative_sharpness_warning{0.35};
  double dark_ratio_warning{0.50};
  double bright_ratio_warning{0.20};
};

using ReviewProgressCallback =
    std::function<void(const std::string&, std::size_t, std::size_t)>;

std::vector<MonoFrameAnalysis> analyzeMonoFrames(
    const std::vector<std::filesystem::path>& images,
    const MonoReviewOptions& options = {},
    const ReviewProgressCallback& progress = {});

}  // namespace calibration_studio
