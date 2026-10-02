#pragma once

#include "camera_calibration_studio/focus_analyzer.hpp"
#include "camera_calibration_studio/hik_camera.hpp"
#include "camera_calibration_studio/recording.hpp"
#include "camera_calibration_studio/review_analysis.hpp"

#include <QLabel>
#include <QWidget>

#include <functional>
#include <memory>
#include <vector>

class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QTimer;
class QTabWidget;

namespace calibration_studio {

class PreviewLabel final : public QLabel {
 public:
  explicit PreviewLabel(QWidget* parent = nullptr);
  void showFrame(const cv::Mat& image);
  void setSelectionEnabled(bool enabled) noexcept { selection_enabled_ = enabled; }
  void setImageRoi(const cv::Rect& roi);
  std::function<void(const cv::Rect&)> roi_selected;

 protected:
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  QRect imageDisplayRect() const;
  cv::Rect widgetRectToImage(const QRect& rectangle) const;
  cv::Mat last_image_;
  cv::Rect image_roi_;
  QPoint drag_start_;
  QPoint drag_end_;
  bool dragging_{false};
  bool selection_enabled_{false};
};

class MonoCameraPage final : public QWidget {
  Q_OBJECT
 public:
  explicit MonoCameraPage(QWidget* parent = nullptr);
  ~MonoCameraPage() override;
  void deactivate();

 private slots:
  void refreshDevices();
  void toggleCamera();
  void updateFrame();
  void chooseSession();
  void captureImage();
  void toggleRecording();
  void chooseRecording();
  void loadRecording();
  void analyzeReviewFrames();
  void importSelectedFrames();
  void resetFocus();
  void chooseImages();
  void chooseOutput();
  void calibrate();

 private:
  CameraSettings settings() const;
  void setStreamingUi(bool streaming);
  void status(const QString& text, bool error = false);

  void stopRecording(bool open_review);
  void setReviewSelection(bool selected);
  void applyReviewAnalysis(const std::vector<MonoFrameAnalysis>& results,
                           std::size_t generation);
  void loadCalibrationImages(const QString& directory);
  void setCalibrationSelection(bool selected);
  void updateCalibrationSelectionSummary();
  void applyCalibrationResult(const Intrinsics& result);
  void updateCalibrationResultPreview();
  std::unique_ptr<HikCamera> camera_;
  FocusAnalyzer focus_analyzer_;
  cv::Rect focus_roi_;
  FocusResult last_focus_;
  QTimer* timer_{nullptr};
  ImageSequenceRecorder recorder_;
  QComboBox* device_{nullptr};
  QDoubleSpinBox* exposure_{nullptr};
  QDoubleSpinBox* gain_{nullptr};
  QDoubleSpinBox* frame_rate_{nullptr};
  QDoubleSpinBox* recording_rate_{nullptr};
  QSpinBox* buffer_count_{nullptr};
  QPushButton* record_button_{nullptr};
  QTabWidget* stages_{nullptr};
  QPushButton* start_stop_{nullptr};
  PreviewLabel* preview_{nullptr};
  QLabel* roi_preview_{nullptr};
  QLabel* focus_values_{nullptr};
  QLabel* status_{nullptr};
  QLineEdit* session_{nullptr};
  QListWidget* captures_{nullptr};
  QLabel* recording_status_{nullptr};
  QLineEdit* recording_path_{nullptr};
  QListWidget* review_list_{nullptr};
  PreviewLabel* review_preview_{nullptr};
  QLabel* review_summary_{nullptr};
  QPushButton* analyze_review_button_{nullptr};
  QSpinBox* target_frames_{nullptr};
  std::size_t review_generation_{0};
  bool review_analysis_running_{false};
  QLineEdit* images_{nullptr};
  QLineEdit* camera_name_{nullptr};
  QLineEdit* output_{nullptr};
  QPushButton* calibrate_button_{nullptr};
  QTabWidget* calibration_details_{nullptr};
  QListWidget* calibration_images_list_{nullptr};
  PreviewLabel* calibration_image_preview_{nullptr};
  QLabel* calibration_selection_summary_{nullptr};
  QLabel* calibration_result_summary_{nullptr};
  QListWidget* calibration_result_list_{nullptr};
  PreviewLabel* calibration_result_preview_{nullptr};
  QCheckBox* show_undistorted_{nullptr};
  Intrinsics last_intrinsics_;
};

class StereoCameraPage final : public QWidget {
  Q_OBJECT
 public:
  explicit StereoCameraPage(QWidget* parent = nullptr);
  ~StereoCameraPage() override;
  void deactivate();

 private slots:
  void refreshDevices();
  void toggleCameras();
  void updateFrames();
  void chooseSession();
  void toggleRecording();
  void chooseRecording();
  void loadRecording();
  void importSelectedPairs();
  void capturePair();
  void chooseLeftImages();
  void chooseRightImages();
  void chooseLeftIntrinsics();
  void chooseRightIntrinsics();
  void chooseOutput();
  void calibrate();

 private:
  void stopRecording(bool open_review);
  void setReviewSelection(bool selected);
  CameraSettings settings() const;
  void setStreamingUi(bool streaming);
  void status(const QString& text, bool error = false);

  std::unique_ptr<HikCamera> left_camera_;
  std::unique_ptr<HikCamera> right_camera_;
  ImageSequenceRecorder recorder_;
  QTimer* timer_{nullptr};
  QComboBox* left_device_{nullptr};
  QComboBox* right_device_{nullptr};
  QDoubleSpinBox* exposure_{nullptr};
  QDoubleSpinBox* gain_{nullptr};
  QDoubleSpinBox* frame_rate_{nullptr};
  QDoubleSpinBox* recording_rate_{nullptr};
  QSpinBox* buffer_count_{nullptr};
  QPushButton* record_button_{nullptr};
  QTabWidget* stages_{nullptr};
  QPushButton* start_stop_{nullptr};
  PreviewLabel* left_preview_{nullptr};
  PreviewLabel* right_preview_{nullptr};
  QLabel* pair_status_{nullptr};
  QLabel* status_{nullptr};
  QLineEdit* session_{nullptr};
  QListWidget* captures_{nullptr};
  QLineEdit* left_images_{nullptr};
  QLineEdit* right_images_{nullptr};
  QLineEdit* left_intrinsics_{nullptr};
  QLineEdit* right_intrinsics_{nullptr};
  QLabel* recording_status_{nullptr};
  QLineEdit* recording_path_{nullptr};
  QListWidget* review_list_{nullptr};
  PreviewLabel* left_review_preview_{nullptr};
  PreviewLabel* right_review_preview_{nullptr};
  QLineEdit* output_{nullptr};
  QPushButton* calibrate_button_{nullptr};
};

}  // namespace calibration_studio
