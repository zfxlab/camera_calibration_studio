#include "camera_calibration_studio/review_analysis.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <utility>

namespace calibration_studio {
namespace {

struct WorkingFrame {
  MonoFrameAnalysis result;
  std::vector<cv::Point2f> image_points;
  std::vector<cv::Point2f> normalized_points;
  double projective_x{0.0};
  double projective_y{0.0};
};

std::vector<cv::Point3f> circleObjectPoints(const MonoOptions& options) {
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<std::size_t>(options.pattern_size.area()));
  for (int row = 0; row < options.pattern_size.height; ++row) {
    for (int column = 0; column < options.pattern_size.width; ++column) {
      points.emplace_back(static_cast<float>(column * options.spacing_m),
                          static_cast<float>(row * options.spacing_m), 0.0F);
    }
  }
  return points;
}

double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2 != 0) return *middle;
  const double upper = *middle;
  return 0.5 * (upper + *std::max_element(values.begin(), middle));
}

std::size_t transformedIndex(const int row, const int column, const int side,
                             const int transform) {
  int transformed_row = row;
  int transformed_column = column;
  switch (transform) {
    case 1:
      transformed_row = side - 1 - column;
      transformed_column = row;
      break;
    case 2:
      transformed_row = side - 1 - row;
      transformed_column = side - 1 - column;
      break;
    case 3:
      transformed_row = column;
      transformed_column = side - 1 - row;
      break;
    case 4:
      transformed_column = side - 1 - column;
      break;
    case 5:
      transformed_row = side - 1 - row;
      break;
    case 6:
      transformed_row = column;
      transformed_column = row;
      break;
    case 7:
      transformed_row = side - 1 - column;
      transformed_column = side - 1 - row;
      break;
    default:
      break;
  }
  return static_cast<std::size_t>(transformed_row * side + transformed_column);
}

double gridDistance(const WorkingFrame& first, const WorkingFrame& second,
                    const cv::Size pattern_size) {
  if (first.normalized_points.size() != second.normalized_points.size() ||
      first.normalized_points.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const bool square = pattern_size.width == pattern_size.height;
  const int transform_count = square ? 8 : 2;
  double best = std::numeric_limits<double>::infinity();
  for (int transform = 0; transform < transform_count; ++transform) {
    double sum = 0.0;
    for (int row = 0; row < pattern_size.height; ++row) {
      for (int column = 0; column < pattern_size.width; ++column) {
        const std::size_t index = static_cast<std::size_t>(row * pattern_size.width + column);
        std::size_t other = index;
        if (square) {
          other = transformedIndex(row, column, pattern_size.width, transform);
        } else if (transform == 1) {
          other = first.normalized_points.size() - 1 - index;
        }
        const cv::Point2f difference =
            first.normalized_points[index] - second.normalized_points[other];
        sum += difference.dot(difference);
      }
    }
    best = std::min(best, std::sqrt(sum / first.normalized_points.size()));
  }
  return best;
}

double angleDifference(const double first, const double second) {
  double difference = std::fmod(std::abs(first - second), 360.0);
  if (difference > 180.0) difference = 360.0 - difference;
  return difference;
}

double poseDistance(const WorkingFrame& first, const WorkingFrame& second) {
  if (!first.result.pose_estimated || !second.result.pose_estimated) {
    return std::numeric_limits<double>::infinity();
  }
  const auto square = [](const double value) { return value * value; };
  const double first_area = std::max(first.result.image_coverage, 1e-9);
  const double second_area = std::max(second.result.image_coverage, 1e-9);
  return std::sqrt(
      square((first.result.normalized_center.x - second.result.normalized_center.x) / 0.18) +
      square((first.result.normalized_center.y - second.result.normalized_center.y) / 0.18) +
      square(std::log(first_area / second_area) / 0.55) +
      1.5 * square((first.result.tilt_x_deg - second.result.tilt_x_deg) / 15.0) +
      1.5 * square((first.result.tilt_y_deg - second.result.tilt_y_deg) / 15.0) +
      0.25 * square(angleDifference(first.result.board_angle_deg,
                                    second.result.board_angle_deg) /
                    45.0));
}

double frameDistance(const WorkingFrame& first, const WorkingFrame& second,
                     const cv::Size pattern_size) {
  if (first.result.pose_estimated && second.result.pose_estimated) {
    return poseDistance(first, second);
  }
  return gridDistance(first, second, pattern_size);
}

bool areDuplicates(const WorkingFrame& first, const WorkingFrame& second,
                   const MonoReviewOptions& options, const double distance) {
  return first.result.pose_estimated && second.result.pose_estimated
             ? distance <= options.pose_duplicate_distance_threshold
             : distance <= options.duplicate_distance_threshold;
}

void estimateBoardPoses(std::vector<WorkingFrame>& working,
                        const std::vector<std::size_t>& candidates,
                        const MonoReviewOptions& options, const cv::Size image_size,
                        const ReviewProgressCallback& progress) {
  if (candidates.size() < 8 || image_size.empty()) return;
  const auto object_points = circleObjectPoints(options.target);
  std::vector<std::size_t> preliminary;
  preliminary.reserve(std::min(candidates.size(), options.preliminary_calibration_limit));
  const auto add_preliminary = [&preliminary, &options](const std::size_t candidate) {
    if (preliminary.size() < options.preliminary_calibration_limit &&
        std::find(preliminary.begin(), preliminary.end(), candidate) == preliminary.end()) {
      preliminary.push_back(candidate);
    }
  };
  add_preliminary(candidates.front());
  const auto add_extreme = [&working, &candidates, &add_preliminary](const auto& score) {
    const auto best = std::max_element(candidates.begin(), candidates.end(),
                                      [&working, &score](const auto left, const auto right) {
      return score(working[left]) < score(working[right]);
    });
    if (best != candidates.end()) add_preliminary(*best);
  };
  add_extreme([](const auto& frame) { return frame.projective_x; });
  add_extreme([](const auto& frame) { return -frame.projective_x; });
  add_extreme([](const auto& frame) { return frame.projective_y; });
  add_extreme([](const auto& frame) { return -frame.projective_y; });
  add_extreme([](const auto& frame) { return frame.result.image_coverage; });
  add_extreme([](const auto& frame) { return -frame.result.image_coverage; });
  while (preliminary.size() < std::min(candidates.size(), options.preliminary_calibration_limit)) {
    std::size_t best = candidates.front();
    double best_distance = -1.0;
    for (const std::size_t candidate : candidates) {
      if (std::find(preliminary.begin(), preliminary.end(), candidate) != preliminary.end()) {
        continue;
      }
      double nearest = std::numeric_limits<double>::infinity();
      for (const std::size_t selected : preliminary) {
        nearest = std::min(nearest,
                           gridDistance(working[candidate], working[selected],
                                        options.target.pattern_size));
      }
      if (nearest > best_distance) {
        best_distance = nearest;
        best = candidate;
      }
    }
    preliminary.push_back(best);
  }

  if (preliminary.size() < 8) return;
  if (progress) progress("Preliminary calibration", 0, preliminary.size());
  std::vector<std::vector<cv::Point3f>> object_views(preliminary.size(), object_points);
  std::vector<std::vector<cv::Point2f>> image_views;
  image_views.reserve(preliminary.size());
  for (const std::size_t candidate : preliminary) {
    image_views.push_back(working[candidate].image_points);
  }

  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distortion;
  std::vector<cv::Mat> rotations;
  std::vector<cv::Mat> translations;
  try {
    const int flags = options.target.rational_model ? cv::CALIB_RATIONAL_MODEL : 0;
    const double rms = cv::calibrateCamera(
        object_views, image_views, image_size, camera_matrix, distortion, rotations,
        translations, flags,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 1e-7));
    if (!std::isfinite(rms) || !cv::checkRange(camera_matrix) ||
        !cv::checkRange(distortion)) {
      return;
    }

    std::vector<double> pose_errors;
    pose_errors.reserve(candidates.size());
    for (std::size_t pose_index = 0; pose_index < candidates.size(); ++pose_index) {
      const std::size_t candidate = candidates[pose_index];
      cv::Mat rotation_vector;
      cv::Mat translation_vector;
      if (!cv::solvePnP(object_points, working[candidate].image_points, camera_matrix,
                        distortion, rotation_vector, translation_vector, false,
                        cv::SOLVEPNP_ITERATIVE)) {
        pose_errors.push_back(std::numeric_limits<double>::infinity());
        continue;
      }
      std::vector<cv::Point2f> projected;
      cv::projectPoints(object_points, rotation_vector, translation_vector, camera_matrix,
                        distortion, projected);
      const double error = cv::norm(working[candidate].image_points, projected, cv::NORM_L2) /
                           std::sqrt(static_cast<double>(projected.size()));
      pose_errors.push_back(error);

      cv::Mat rotation;
      cv::Rodrigues(rotation_vector, rotation);
      cv::Vec3d normal(rotation.at<double>(0, 2), rotation.at<double>(1, 2),
                       rotation.at<double>(2, 2));
      if (normal[2] < 0.0) normal = -normal;
      auto& result = working[candidate].result;
      result.pose_estimated = std::isfinite(error);
      result.tilt_x_deg = std::atan2(normal[0], std::abs(normal[2])) * 180.0 / CV_PI;
      result.tilt_y_deg = std::atan2(normal[1], std::abs(normal[2])) * 180.0 / CV_PI;
      result.total_tilt_deg =
          std::atan2(std::hypot(normal[0], normal[1]), std::abs(normal[2])) * 180.0 / CV_PI;
      result.pose_reprojection_error_px = error;
      if (progress && (pose_index % 5 == 0 || pose_index + 1 == candidates.size())) {
        progress("Estimating poses", pose_index + 1, candidates.size());
      }
    }

    std::vector<double> finite_errors;
    for (const double error : pose_errors) {
      if (std::isfinite(error)) finite_errors.push_back(error);
    }
    const double error_limit = std::max(2.0, median(finite_errors) * 3.0);
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (!std::isfinite(pose_errors[index]) || pose_errors[index] > error_limit) {
        working[candidates[index]].result.pose_estimated = false;
      }
    }
  } catch (const cv::Exception&) {
    // Image-space diversity remains available as a safe fallback.
  }
}

}  // namespace

std::vector<MonoFrameAnalysis> analyzeMonoFrames(
    const std::vector<std::filesystem::path>& images, const MonoReviewOptions& options,
    const ReviewProgressCallback& progress) {
  std::vector<WorkingFrame> working;
  working.reserve(images.size());
  std::map<std::pair<int, int>, std::size_t> size_counts;
  std::vector<double> valid_sharpness;
  const std::size_t expected_points =
      static_cast<std::size_t>(options.target.pattern_size.area());

  for (std::size_t image_index = 0; image_index < images.size(); ++image_index) {
    const auto& path = images[image_index];
    const auto report_detection_progress = [&progress, image_index, &images] {
      if (progress && (image_index % 5 == 0 || image_index + 1 == images.size())) {
        progress("Detecting circle grids", image_index + 1, images.size());
      }
    };
    WorkingFrame frame;
    frame.result.path = path;
    const cv::Mat gray = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
      working.push_back(std::move(frame));
      report_detection_progress();
      continue;
    }

    frame.result.image_size = gray.size();
    ++size_counts[{gray.cols, gray.rows}];
    cv::Mat gradient_x;
    cv::Mat gradient_y;
    cv::Sobel(gray, gradient_x, CV_32F, 1, 0, 3);
    cv::Sobel(gray, gradient_y, CV_32F, 0, 1, 3);
    frame.result.sharpness =
        cv::mean(gradient_x.mul(gradient_x) + gradient_y.mul(gradient_y))[0];
    frame.result.mean_brightness = cv::mean(gray)[0];
    frame.result.dark_ratio = static_cast<double>(cv::countNonZero(gray < 10)) /
                              static_cast<double>(gray.total());
    frame.result.bright_ratio = static_cast<double>(cv::countNonZero(gray > 245)) /
                                static_cast<double>(gray.total());

    std::vector<cv::Point2f> centers;
    const bool found = cv::findCirclesGrid(
        gray, options.target.pattern_size, centers,
        cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
    frame.result.detected_points = static_cast<int>(centers.size());
    if (!found || centers.size() != expected_points) {
      frame.result.decision = MonoFrameDecision::pattern_not_found;
      working.push_back(std::move(frame));
      report_detection_progress();
      continue;
    }

    cv::Point2d center;
    for (const auto& point : centers) center += cv::Point2d(point.x, point.y);
    center *= 1.0 / static_cast<double>(centers.size());
    frame.result.normalized_center =
        {center.x / static_cast<double>(gray.cols), center.y / static_cast<double>(gray.rows)};
    const cv::Rect bounds = cv::boundingRect(centers);
    frame.result.image_coverage = static_cast<double>(bounds.area()) /
                                  static_cast<double>(gray.total());
    const cv::Point2f row_axis = centers[options.target.pattern_size.width - 1] - centers[0];
    frame.result.board_angle_deg =
        std::atan2(row_axis.y, row_axis.x) * 180.0 / CV_PI;
    frame.image_points = centers;
    const int width = options.target.pattern_size.width;
    const int height = options.target.pattern_size.height;
    const cv::Point2f& top_left = centers.front();
    const cv::Point2f& top_right = centers[static_cast<std::size_t>(width - 1)];
    const cv::Point2f& bottom_left = centers[static_cast<std::size_t>((height - 1) * width)];
    const cv::Point2f& bottom_right = centers.back();
    const double top = cv::norm(top_right - top_left);
    const double bottom = cv::norm(bottom_right - bottom_left);
    const double left = cv::norm(bottom_left - top_left);
    const double right = cv::norm(bottom_right - top_right);
    frame.projective_x = std::log(std::max(left, 1e-6) / std::max(right, 1e-6));
    frame.projective_y = std::log(std::max(top, 1e-6) / std::max(bottom, 1e-6));
    frame.normalized_points.reserve(centers.size());
    for (const auto& point : centers) {
      frame.normalized_points.emplace_back(point.x / static_cast<float>(gray.cols),
                                           point.y / static_cast<float>(gray.rows));
    }
    valid_sharpness.push_back(frame.result.sharpness);
    frame.result.decision = MonoFrameDecision::keep;
    working.push_back(std::move(frame));
    report_detection_progress();
  }

  cv::Size reference_size;
  std::size_t largest_count = 0;
  for (const auto& entry : size_counts) {
    if (entry.second > largest_count) {
      largest_count = entry.second;
      reference_size = {entry.first.first, entry.first.second};
    }
  }
  const double median_sharpness = median(valid_sharpness);
  for (auto& frame : working) {
    if (!frame.result.image_size.empty() && frame.result.image_size != reference_size) {
      frame.result.decision = MonoFrameDecision::size_mismatch;
    }
    frame.result.low_sharpness =
        frame.result.decision == MonoFrameDecision::keep && median_sharpness > 0.0 &&
        frame.result.sharpness < median_sharpness * options.relative_sharpness_warning;
    frame.result.exposure_warning =
        frame.result.decision == MonoFrameDecision::keep &&
        (frame.result.dark_ratio > options.dark_ratio_warning ||
         frame.result.bright_ratio > options.bright_ratio_warning);
  }

  std::vector<std::size_t> candidates;
  for (std::size_t index = 0; index < working.size(); ++index) {
    if (working[index].result.decision == MonoFrameDecision::keep) candidates.push_back(index);
  }
  std::stable_sort(candidates.begin(), candidates.end(), [&working](const auto left,
                                                                    const auto right) {
    return working[left].result.sharpness > working[right].result.sharpness;
  });
  estimateBoardPoses(working, candidates, options, reference_size, progress);

  std::vector<std::size_t> representatives;
  for (const std::size_t candidate : candidates) {
    double nearest_distance = std::numeric_limits<double>::infinity();
    std::size_t nearest = 0;
    for (const std::size_t representative : representatives) {
      const double distance = frameDistance(working[candidate], working[representative],
                                            options.target.pattern_size);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = representative;
      }
    }
    if (representatives.empty() ||
        !areDuplicates(working[candidate], working[nearest], options, nearest_distance)) {
      representatives.push_back(candidate);
    }
  }

  // Representatives are already ordered by sharpness. Seed with the sharpest frame,
  // then repeatedly add the frame farthest from the selected set. This limits the
  // result while preserving global coverage of the observed pose trajectory.
  const std::size_t selection_limit =
      std::min(representatives.size(), std::max<std::size_t>(1, options.target_frame_count));
  std::vector<std::size_t> selected;
  const auto add_selected = [&selected, selection_limit](const std::size_t candidate) {
    if (selected.size() < selection_limit &&
        std::find(selected.begin(), selected.end(), candidate) == selected.end()) {
      selected.push_back(candidate);
    }
  };
  if (!representatives.empty()) add_selected(representatives.front());

  // Explicitly seed the selection with available tilt extremes. Pure farthest-point
  // sampling tends to spend its early choices on board translation and scale.
  const auto extreme = [&working, &representatives](const auto& score) {
    std::size_t best = representatives.front();
    double best_score = -std::numeric_limits<double>::infinity();
    for (const std::size_t candidate : representatives) {
      if (!working[candidate].result.pose_estimated) continue;
      const double value = score(working[candidate].result);
      if (value > best_score) {
        best_score = value;
        best = candidate;
      }
    }
    return std::make_pair(best, best_score);
  };
  if (!representatives.empty()) {
    const auto positive_x = extreme([](const auto& result) { return result.tilt_x_deg; });
    const auto negative_x = extreme([](const auto& result) { return -result.tilt_x_deg; });
    const auto positive_y = extreme([](const auto& result) { return result.tilt_y_deg; });
    const auto negative_y = extreme([](const auto& result) { return -result.tilt_y_deg; });
    const auto maximum_tilt = extreme([](const auto& result) { return result.total_tilt_deg; });
    const auto minimum_tilt = extreme([](const auto& result) { return -result.total_tilt_deg; });
    if (positive_x.second >= 5.0) add_selected(positive_x.first);
    if (negative_x.second >= 5.0) add_selected(negative_x.first);
    if (positive_y.second >= 5.0) add_selected(positive_y.first);
    if (negative_y.second >= 5.0) add_selected(negative_y.first);
    if (maximum_tilt.second >= 10.0) add_selected(maximum_tilt.first);
    if (std::isfinite(minimum_tilt.second)) add_selected(minimum_tilt.first);
  }
  while (selected.size() < selection_limit) {
    std::size_t best_candidate = representatives.front();
    double best_distance = -1.0;
    for (const std::size_t candidate : representatives) {
      if (std::find(selected.begin(), selected.end(), candidate) != selected.end()) continue;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (const std::size_t chosen : selected) {
        nearest_distance = std::min(
            nearest_distance,
            frameDistance(working[candidate], working[chosen], options.target.pattern_size));
      }
      if (nearest_distance > best_distance) {
        best_distance = nearest_distance;
        best_candidate = candidate;
      }
    }
    selected.push_back(best_candidate);
  }

  for (const std::size_t candidate : candidates) {
    if (std::find(selected.begin(), selected.end(), candidate) != selected.end()) {
      working[candidate].result.decision = MonoFrameDecision::keep;
      working[candidate].result.recommended = true;
      continue;
    }
    double nearest_distance = std::numeric_limits<double>::infinity();
    std::size_t nearest = 0;
    for (const std::size_t chosen : selected) {
      const double distance =
          frameDistance(working[candidate], working[chosen], options.target.pattern_size);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = chosen;
      }
    }
    working[candidate].result.decision =
        areDuplicates(working[candidate], working[nearest], options, nearest_distance)
            ? MonoFrameDecision::duplicate
            : MonoFrameDecision::redundant;
    working[candidate].result.duplicate_of = working[nearest].result.path;
    working[candidate].result.duplicate_distance = nearest_distance;
  }

  std::vector<MonoFrameAnalysis> results;
  results.reserve(working.size());
  for (auto& frame : working) results.push_back(std::move(frame.result));
  return results;
}

}  // namespace calibration_studio
