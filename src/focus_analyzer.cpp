#include "camera_calibration_studio/focus_analyzer.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <stdexcept>

namespace calibration_studio {

FocusAnalyzer::FocusAnalyzer(const double smoothing_alpha,
                             const int dark_threshold,
                             const int bright_threshold)
    : smoothing_alpha_(smoothing_alpha),
      dark_threshold_(dark_threshold),
      bright_threshold_(bright_threshold) {
  if (!(smoothing_alpha_ > 0.0 && smoothing_alpha_ <= 1.0) ||
      dark_threshold_ < 0 || bright_threshold_ > 255 ||
      dark_threshold_ >= bright_threshold_) {
    throw std::invalid_argument("invalid focus analyzer settings");
  }
}

FocusResult FocusAnalyzer::process(const cv::Mat& image,
                                   const cv::Rect& requested_roi) {
  if (image.empty()) {
    throw std::invalid_argument("cannot analyze an empty image");
  }
  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  }
  const cv::Rect bounds(0, 0, gray.cols, gray.rows);
  cv::Rect roi = requested_roi & bounds;
  if (roi.width < 8 || roi.height < 8) {
    const int width = std::max(8, gray.cols * 3 / 10);
    const int height = std::max(8, gray.rows * 3 / 10);
    roi = cv::Rect((gray.cols - width) / 2, (gray.rows - height) / 2,
                   std::min(width, gray.cols), std::min(height, gray.rows));
  }
  const cv::Mat sample = gray(roi);
  cv::Mat gradient_x;
  cv::Mat gradient_y;
  cv::Sobel(sample, gradient_x, CV_32F, 1, 0, 3);
  cv::Sobel(sample, gradient_y, CV_32F, 0, 1, 3);
  const double score = cv::mean(gradient_x.mul(gradient_x) +
                                gradient_y.mul(gradient_y))[0];
  if (!initialized_) {
    smoothed_score_ = score;
    peak_score_ = score;
    initialized_ = true;
  } else {
    smoothed_score_ = smoothing_alpha_ * score +
                      (1.0 - smoothing_alpha_) * smoothed_score_;
    peak_score_ = std::max(peak_score_, smoothed_score_);
  }

  FocusResult result;
  result.current_score = score;
  result.smoothed_score = smoothed_score_;
  result.peak_score = peak_score_;
  result.relative_score = peak_score_ > 0.0 ? smoothed_score_ / peak_score_ : 0.0;
  result.mean_brightness = cv::mean(sample)[0];
  result.dark_ratio = static_cast<double>(cv::countNonZero(sample < dark_threshold_)) /
                      static_cast<double>(sample.total());
  result.bright_ratio = static_cast<double>(cv::countNonZero(sample > bright_threshold_)) /
                        static_cast<double>(sample.total());
  result.roi = roi;
  return result;
}

void FocusAnalyzer::reset() noexcept {
  initialized_ = false;
  smoothed_score_ = 0.0;
  peak_score_ = 0.0;
}

}  // namespace calibration_studio

