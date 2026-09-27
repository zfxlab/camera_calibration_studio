#include "camera_calibration_studio/calibration.hpp"
#include "camera_calibration_studio/dataset.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>

namespace calibration_studio {
namespace {

using ObjectViews = std::vector<std::vector<cv::Point3f>>;
using ImageViews = std::vector<std::vector<cv::Point2f>>;

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

bool solveMono(const ObjectViews& object_views, const ImageViews& image_views,
               const cv::Size image_size, const int flags, Intrinsics& result,
               std::vector<double>& errors, std::string& error) {
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distortion;
  std::vector<cv::Mat> rotations;
  std::vector<cv::Mat> translations;
  try {
    result.rms_px = cv::calibrateCamera(object_views, image_views, image_size,
                                        camera_matrix, distortion, rotations, translations,
                                        flags,
                                        cv::TermCriteria(cv::TermCriteria::COUNT |
                                                             cv::TermCriteria::EPS,
                                                         100, 1e-10));
    errors.clear();
    for (std::size_t index = 0; index < object_views.size(); ++index) {
      std::vector<cv::Point2f> projected;
      cv::projectPoints(object_views[index], rotations[index], translations[index],
                        camera_matrix, distortion, projected);
      const double l2 = cv::norm(image_views[index], projected, cv::NORM_L2);
      errors.push_back(l2 / std::sqrt(static_cast<double>(projected.size())));
    }
  } catch (const cv::Exception& exception) {
    error = std::string("OpenCV mono calibration failed: ") + exception.what();
    return false;
  }
  result.image_size = image_size;
  result.camera_matrix = camera_matrix;
  result.distortion = distortion;
  return true;
}

double median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

cv::Mat readMatrixNode(const cv::FileNode& node, const int default_rows,
                       const int default_columns) {
  if (node.empty()) {
    return {};
  }
  cv::Mat matrix;
  try {
    node >> matrix;
  } catch (const cv::Exception&) {
    matrix.release();
  }
  if (!matrix.empty()) {
    matrix.convertTo(matrix, CV_64F);
    return matrix;
  }
  const cv::FileNode data = node["data"];
  if (data.empty() || !data.isSeq()) {
    return {};
  }
  const int rows = node["rows"].empty() ? default_rows : static_cast<int>(node["rows"]);
  const int columns = node["cols"].empty() ? default_columns : static_cast<int>(node["cols"]);
  std::vector<double> values;
  data >> values;
  if (rows <= 0 || columns <= 0 || values.size() != static_cast<std::size_t>(rows * columns)) {
    return {};
  }
  matrix = cv::Mat(rows, columns, CV_64F, values.data()).clone();
  return matrix;
}

void writeSequence(std::ostream& stream, const cv::Mat& input) {
  cv::Mat matrix;
  input.reshape(1, 1).convertTo(matrix, CV_64F);
  stream << '[';
  for (int index = 0; index < matrix.cols; ++index) {
    if (index != 0) {
      stream << ", ";
    }
    stream << matrix.at<double>(0, index);
  }
  stream << ']';
}

std::map<int, std::array<cv::Point3f, 4>> arucoObjectCorners(
    const ArucoBoardOptions& options) {
  const float half_marker = static_cast<float>(options.marker_length_m * 0.5);
  const float half_x = static_cast<float>(options.center_spacing_x_m * 0.5);
  const float half_y = static_cast<float>(options.center_spacing_y_m * 0.5);
  const std::array<cv::Point2f, 4> centers{{{-half_x, -half_y}, {half_x, -half_y},
                                            {-half_x, half_y}, {half_x, half_y}}};
  std::map<int, std::array<cv::Point3f, 4>> result;
  for (std::size_t index = 0; index < options.marker_ids.size(); ++index) {
    const auto center = centers[index];
    result.emplace(options.marker_ids[index], std::array<cv::Point3f, 4>{{
                                                  {center.x - half_marker, center.y - half_marker, 0.0F},
                                                  {center.x + half_marker, center.y - half_marker, 0.0F},
                                                  {center.x + half_marker, center.y + half_marker, 0.0F},
                                                  {center.x - half_marker, center.y + half_marker, 0.0F}}});
  }
  return result;
}

using MarkerMap = std::map<int, std::array<cv::Point2f, 4>>;

MarkerMap detectMarkers(const cv::Mat& image, const ArucoBoardOptions& options) {
  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  }
  const auto dictionary = cv::aruco::getPredefinedDictionary(options.dictionary_id);
  const auto parameters = cv::aruco::DetectorParameters::create();
  parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(gray, dictionary, corners, ids, parameters);
  MarkerMap result;
  for (std::size_t index = 0; index < ids.size(); ++index) {
    if (corners[index].size() == 4 &&
        std::find(options.marker_ids.begin(), options.marker_ids.end(), ids[index]) !=
            options.marker_ids.end()) {
      result.emplace(ids[index], std::array<cv::Point2f, 4>{{corners[index][0], corners[index][1],
                                                             corners[index][2], corners[index][3]}});
    }
  }
  return result;
}

}  // namespace

bool Intrinsics::valid() const {
  return image_size.width > 0 && image_size.height > 0 && camera_matrix.rows == 3 &&
         camera_matrix.cols == 3 && !distortion.empty() &&
         std::isfinite(camera_matrix.at<double>(0, 0)) && camera_matrix.at<double>(0, 0) > 0.0 &&
         std::isfinite(camera_matrix.at<double>(1, 1)) && camera_matrix.at<double>(1, 1) > 0.0;
}

bool calibrateMono(const std::vector<std::filesystem::path>& images,
                   const MonoOptions& options, Intrinsics& result, std::string& error) {
  if (options.pattern_size.width < 2 || options.pattern_size.height < 2 ||
      !(options.spacing_m > 0.0)) {
    error = "invalid circle-grid geometry";
    return false;
  }
  ObjectViews object_views;
  ImageViews image_views;
  std::vector<std::filesystem::path> accepted;
  std::vector<std::filesystem::path> rejected;
  cv::Size image_size;
  const auto object_points = circleObjectPoints(options);
  for (const auto& path : images) {
    const cv::Mat image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
    if (image.empty() || (!image_size.empty() && image.size() != image_size)) {
      rejected.push_back(path);
      continue;
    }
    image_size = image.size();
    std::vector<cv::Point2f> centers;
    const bool found = cv::findCirclesGrid(
        image, options.pattern_size, centers,
        cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
    if (!found || centers.size() != object_points.size()) {
      rejected.push_back(path);
      continue;
    }
    object_views.push_back(object_points);
    image_views.push_back(std::move(centers));
    accepted.push_back(path);
  }
  if (accepted.size() < 8) {
    error = "at least 8 valid circle-grid images are required; found " +
            std::to_string(accepted.size());
    return false;
  }

  const int flags = options.rational_model ? cv::CALIB_RATIONAL_MODEL : 0;
  std::vector<double> view_errors;
  if (!solveMono(object_views, image_views, image_size, flags, result, view_errors, error)) {
    return false;
  }

  const double rejection_threshold = std::max(1.5, median(view_errors) * 2.5);
  ObjectViews filtered_objects;
  ImageViews filtered_images;
  std::vector<std::filesystem::path> filtered_paths;
  for (std::size_t index = 0; index < view_errors.size(); ++index) {
    if (view_errors[index] <= rejection_threshold || accepted.size() <= 8) {
      filtered_objects.push_back(object_views[index]);
      filtered_images.push_back(image_views[index]);
      filtered_paths.push_back(accepted[index]);
    } else {
      rejected.push_back(accepted[index]);
    }
  }
  if (filtered_paths.size() != accepted.size()) {
    if (!solveMono(filtered_objects, filtered_images, image_size, flags, result, view_errors, error)) {
      return false;
    }
  }
  result.used_images = std::move(filtered_paths);
  result.rejected_images = std::move(rejected);
  result.per_view_error_px = std::move(view_errors);
  if (!result.valid()) {
    error = "mono calibration produced invalid intrinsics";
    return false;
  }
  error.clear();
  return true;
}

bool calibrateStereoAruco(const std::filesystem::path& left_directory,
                          const std::filesystem::path& right_directory,
                          const Intrinsics& left, const Intrinsics& right,
                          const ArucoBoardOptions& options, StereoResult& result,
                          std::string& error) {
  if (!left.valid() || !right.valid()) {
    error = "left or right intrinsics are invalid";
    return false;
  }
  if (left.image_size != right.image_size) {
    error = "left and right intrinsic image sizes differ";
    return false;
  }
  const auto left_files = listImages(left_directory);
  std::map<std::string, std::filesystem::path> right_files;
  for (const auto& path : listImages(right_directory)) {
    right_files.emplace(path.filename().string(), path);
  }
  const auto board = arucoObjectCorners(options);
  ObjectViews object_views;
  ImageViews left_views;
  ImageViews right_views;
  result.rejected_pairs.clear();

  for (const auto& left_path : left_files) {
    const auto match = right_files.find(left_path.filename().string());
    if (match == right_files.end()) {
      result.rejected_pairs.push_back(left_path.filename().string() + ": missing right image");
      continue;
    }
    const cv::Mat left_image = cv::imread(left_path.string(), cv::IMREAD_UNCHANGED);
    const cv::Mat right_image = cv::imread(match->second.string(), cv::IMREAD_UNCHANGED);
    if (left_image.empty() || right_image.empty() || left_image.size() != left.image_size ||
        right_image.size() != right.image_size) {
      result.rejected_pairs.push_back(left_path.filename().string() + ": invalid image size");
      continue;
    }
    const auto left_markers = detectMarkers(left_image, options);
    const auto right_markers = detectMarkers(right_image, options);
    std::vector<int> common;
    for (const auto& marker : left_markers) {
      if (right_markers.count(marker.first) != 0U) {
        common.push_back(marker.first);
      }
    }
    if (common.size() < static_cast<std::size_t>(options.minimum_common_markers)) {
      result.rejected_pairs.push_back(left_path.filename().string() + ": insufficient common markers");
      continue;
    }
    std::sort(common.begin(), common.end());
    std::vector<cv::Point3f> objects;
    std::vector<cv::Point2f> left_points;
    std::vector<cv::Point2f> right_points;
    for (const int id : common) {
      const auto& object_corners = board.at(id);
      for (std::size_t corner = 0; corner < 4; ++corner) {
        objects.push_back(object_corners[corner]);
        left_points.push_back(left_markers.at(id)[corner]);
        right_points.push_back(right_markers.at(id)[corner]);
      }
    }
    object_views.push_back(std::move(objects));
    left_views.push_back(std::move(left_points));
    right_views.push_back(std::move(right_points));
  }
  if (object_views.size() < 5) {
    error = "at least 5 valid stereo ArUco pairs are required; found " +
            std::to_string(object_views.size());
    return false;
  }

  cv::Mat left_matrix = left.camera_matrix.clone();
  cv::Mat left_distortion = left.distortion.clone();
  cv::Mat right_matrix = right.camera_matrix.clone();
  cv::Mat right_distortion = right.distortion.clone();
  try {
    result.rms_px = cv::stereoCalibrate(
        object_views, left_views, right_views, left_matrix, left_distortion,
        right_matrix, right_distortion, left.image_size, result.rotation_right_left,
        result.translation_right_left, result.essential, result.fundamental,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-10));
    cv::stereoRectify(left_matrix, left_distortion, right_matrix, right_distortion,
                      left.image_size, result.rotation_right_left,
                      result.translation_right_left, result.rectification_left,
                      result.rectification_right, result.projection_left,
                      result.projection_right, result.disparity_to_depth,
                      cv::CALIB_ZERO_DISPARITY, -1.0, left.image_size);

    double total_error = 0.0;
    double maximum_error = 0.0;
    std::size_t error_count = 0;
    for (std::size_t view = 0; view < left_views.size(); ++view) {
      std::vector<cv::Vec3f> right_lines;
      std::vector<cv::Vec3f> left_lines;
      cv::computeCorrespondEpilines(left_views[view], 1, result.fundamental, right_lines);
      cv::computeCorrespondEpilines(right_views[view], 2, result.fundamental, left_lines);
      for (std::size_t index = 0; index < left_views[view].size(); ++index) {
        const auto distance = [](const cv::Point2f& point, const cv::Vec3f& line) {
          const double denominator = std::hypot(line[0], line[1]);
          return denominator > 0.0
                     ? std::abs(line[0] * point.x + line[1] * point.y + line[2]) / denominator
                     : std::numeric_limits<double>::infinity();
        };
        const double symmetric =
            0.5 * (distance(left_views[view][index], left_lines[index]) +
                   distance(right_views[view][index], right_lines[index]));
        total_error += symmetric;
        maximum_error = std::max(maximum_error, symmetric);
        ++error_count;
      }
    }
    result.mean_epipolar_error_px = total_error / static_cast<double>(error_count);
    result.max_epipolar_error_px = maximum_error;
    result.baseline_m = cv::norm(result.translation_right_left);
    result.used_pairs = object_views.size();
  } catch (const cv::Exception& exception) {
    error = std::string("OpenCV stereo calibration failed: ") + exception.what();
    return false;
  }
  if (!std::isfinite(result.rms_px) || !std::isfinite(result.baseline_m) ||
      result.baseline_m <= 0.0) {
    error = "stereo calibration produced a degenerate result";
    return false;
  }
  error.clear();
  return true;
}

bool saveIntrinsics(const std::filesystem::path& path, const std::string& camera_name,
                    const Intrinsics& intrinsics, std::string& error) {
  if (!intrinsics.valid()) {
    error = "cannot save invalid intrinsics";
    return false;
  }
  try {
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream stream(path);
    if (!stream) {
      error = "cannot open output path: " + path.string();
      return false;
    }
    stream << std::setprecision(16);
    stream << "%YAML:1.0\n---\n";
    stream << "image_width: " << intrinsics.image_size.width << '\n';
    stream << "image_height: " << intrinsics.image_size.height << '\n';
    stream << "camera_name: " << camera_name << '\n';
    stream << "distortion_model: plumb_bob\n";
    stream << "camera_matrix:\n  rows: 3\n  cols: 3\n  data: ";
    writeSequence(stream, intrinsics.camera_matrix);
    stream << "\ndistortion_coefficients:\n  rows: 1\n  cols: "
           << intrinsics.distortion.total() << "\n  data: ";
    writeSequence(stream, intrinsics.distortion);
    stream << "\nrectification_matrix:\n  rows: 3\n  cols: 3\n  data: ";
    writeSequence(stream, cv::Mat::eye(3, 3, CV_64F));
    cv::Mat projection = cv::Mat::zeros(3, 4, CV_64F);
    intrinsics.camera_matrix.copyTo(projection(cv::Rect(0, 0, 3, 3)));
    stream << "\nprojection_matrix:\n  rows: 3\n  cols: 4\n  data: ";
    writeSequence(stream, projection);
    stream << "\ncalibration_metrics:\n  rms_px: " << intrinsics.rms_px
           << "\n  used_images: " << intrinsics.used_images.size()
           << "\n  rejected_images: " << intrinsics.rejected_images.size() << '\n';
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
  error.clear();
  return true;
}

bool loadIntrinsics(const std::filesystem::path& path, Intrinsics& intrinsics,
                    std::string& error) {
  try {
    cv::FileStorage storage(path.string(), cv::FileStorage::READ);
    if (!storage.isOpened()) {
      error = "cannot open intrinsics file: " + path.string();
      return false;
    }
    intrinsics.image_size.width = static_cast<int>(storage["image_width"]);
    intrinsics.image_size.height = static_cast<int>(storage["image_height"]);
    intrinsics.camera_matrix = readMatrixNode(storage["camera_matrix"], 3, 3);
    intrinsics.distortion = readMatrixNode(storage["distortion_coefficients"], 1, 5);
    const cv::FileNode metrics = storage["calibration_metrics"];
    if (!metrics.empty() && !metrics["rms_px"].empty()) {
      intrinsics.rms_px = static_cast<double>(metrics["rms_px"]);
    }
  } catch (const cv::Exception& exception) {
    error = exception.what();
    return false;
  }
  if (!intrinsics.valid()) {
    error = "intrinsics file is incomplete or invalid: " + path.string();
    return false;
  }
  error.clear();
  return true;
}

bool saveStereoResult(const std::filesystem::path& path, const StereoResult& result,
                      std::string& error) {
  try {
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }
    cv::FileStorage storage(path.string(), cv::FileStorage::WRITE |
                                               cv::FileStorage::FORMAT_YAML);
    if (!storage.isOpened()) {
      error = "cannot open output path: " + path.string();
      return false;
    }
    storage << "format_version" << 1;
    storage << "transform_convention" << "X_right = R_right_left * X_left + t_right_left";
    storage << "length_unit" << "meter";
    storage << "rotation_right_left" << result.rotation_right_left;
    storage << "translation_right_left_m" << result.translation_right_left;
    storage << "essential_matrix" << result.essential;
    storage << "fundamental_matrix" << result.fundamental;
    storage << "rectification_left" << result.rectification_left;
    storage << "rectification_right" << result.rectification_right;
    storage << "projection_left" << result.projection_left;
    storage << "projection_right" << result.projection_right;
    storage << "disparity_to_depth" << result.disparity_to_depth;
    storage << "metrics" << "{";
    storage << "used_pairs" << static_cast<int>(result.used_pairs);
    storage << "stereo_rms_px" << result.rms_px;
    storage << "mean_epipolar_error_px" << result.mean_epipolar_error_px;
    storage << "max_epipolar_error_px" << result.max_epipolar_error_px;
    storage << "baseline_m" << result.baseline_m;
    storage << "}";
  } catch (const cv::Exception& exception) {
    error = exception.what();
    return false;
  }
  error.clear();
  return true;
}

}  // namespace calibration_studio
