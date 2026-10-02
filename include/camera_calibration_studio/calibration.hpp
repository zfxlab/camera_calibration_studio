#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace calibration_studio {

struct MonoOptions {
  cv::Size pattern_size{7, 7};
  double spacing_m{0.03};
  bool rational_model{false};
  double outlier_minimum_error_px{0.15};
  double outlier_mad_scale{3.0};
};

struct Intrinsics {
  cv::Size image_size;
  cv::Mat camera_matrix;
  cv::Mat distortion;
  double rms_px{0.0};
  std::vector<double> per_view_error_px;
  std::vector<std::filesystem::path> used_images;
  std::vector<std::filesystem::path> rejected_images;

  [[nodiscard]] bool valid() const;
};

struct ArucoBoardOptions {
  int dictionary_id{8};  // cv::aruco::DICT_6X6_50
  std::array<int, 4> marker_ids{{1, 2, 3, 4}};
  double marker_length_m{0.20};
  double center_spacing_x_m{1.10};
  double center_spacing_y_m{0.70};
  int minimum_common_markers{3};
};

struct StereoResult {
  cv::Mat rotation_right_left;
  cv::Mat translation_right_left;
  cv::Mat essential;
  cv::Mat fundamental;
  cv::Mat rectification_left;
  cv::Mat rectification_right;
  cv::Mat projection_left;
  cv::Mat projection_right;
  cv::Mat disparity_to_depth;
  double rms_px{0.0};
  double mean_epipolar_error_px{0.0};
  double max_epipolar_error_px{0.0};
  double baseline_m{0.0};
  std::size_t used_pairs{0};
  std::vector<std::string> rejected_pairs;
};

bool calibrateMono(const std::vector<std::filesystem::path>& images,
                   const MonoOptions& options,
                   Intrinsics& result,
                   std::string& error);

bool calibrateStereoAruco(const std::filesystem::path& left_directory,
                          const std::filesystem::path& right_directory,
                          const Intrinsics& left,
                          const Intrinsics& right,
                          const ArucoBoardOptions& options,
                          StereoResult& result,
                          std::string& error);

bool saveIntrinsics(const std::filesystem::path& path,
                    const std::string& camera_name,
                    const Intrinsics& intrinsics,
                    std::string& error);

bool loadIntrinsics(const std::filesystem::path& path,
                    Intrinsics& intrinsics,
                    std::string& error);

bool saveStereoResult(const std::filesystem::path& path,
                      const StereoResult& result,
                      std::string& error);

}  // namespace calibration_studio
