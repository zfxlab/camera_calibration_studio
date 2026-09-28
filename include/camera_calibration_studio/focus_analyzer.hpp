#pragma once

#include <opencv2/core.hpp>

namespace calibration_studio {

struct FocusResult {
  double current_score{0.0};
  double smoothed_score{0.0};
  double peak_score{0.0};
  double relative_score{0.0};
  double mean_brightness{0.0};
  double dark_ratio{0.0};
  double bright_ratio{0.0};
  cv::Rect roi;
};

class FocusAnalyzer {
 public:
  explicit FocusAnalyzer(double smoothing_alpha = 0.20,
                         int dark_threshold = 10,
                         int bright_threshold = 245);

  FocusResult process(const cv::Mat& image, const cv::Rect& requested_roi);
  void reset() noexcept;

 private:
  double smoothing_alpha_;
  int dark_threshold_;
  int bright_threshold_;
  bool initialized_{false};
  double smoothed_score_{0.0};
  double peak_score_{0.0};
};

}  // namespace calibration_studio

