#pragma once

#include "camera_calibration_studio/hik_camera.hpp"

#include <QMainWindow>

#include <memory>
#include <vector>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QTabWidget;
class QTimer;

namespace calibration_studio {

class MainWindow final : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

 private slots:
  void refreshDevices();
  void toggleCameras();
  void updatePreviews();
  void chooseSessionDirectory();
  void captureLeftMono();
  void captureRightMono();
  void captureStereoPair();
  void chooseMonoImages();
  void chooseMonoOutput();
  void runMonoCalibration();
  void chooseStereoLeftImages();
  void chooseStereoRightImages();
  void chooseLeftIntrinsics();
  void chooseRightIntrinsics();
  void chooseStereoOutput();
  void runStereoCalibration();

 private:
  void buildUi();
  CameraSettings settingsFromUi() const;
  void setCaptureControlsEnabled(bool enabled);
  void showStatus(const QString& message, bool error = false);
  static void setPreview(QLabel* label, const cv::Mat& image);

  std::unique_ptr<HikCamera> left_camera_;
  std::unique_ptr<HikCamera> right_camera_;
  std::vector<CameraDescriptor> devices_;
  QTimer* preview_timer_{nullptr};
  QComboBox* left_device_{nullptr};
  QComboBox* right_device_{nullptr};
  QDoubleSpinBox* exposure_{nullptr};
  QDoubleSpinBox* gain_{nullptr};
  QDoubleSpinBox* frame_rate_{nullptr};
  QSpinBox* buffer_count_{nullptr};
  QPushButton* start_stop_{nullptr};
  QLabel* left_preview_{nullptr};
  QLabel* right_preview_{nullptr};
  QLineEdit* session_directory_{nullptr};
  QPushButton* capture_left_{nullptr};
  QPushButton* capture_right_{nullptr};
  QPushButton* capture_pair_{nullptr};
  QListWidget* capture_list_{nullptr};

  QLineEdit* mono_images_{nullptr};
  QLineEdit* mono_output_{nullptr};
  QLineEdit* mono_camera_name_{nullptr};
  QPushButton* mono_run_{nullptr};

  QLineEdit* stereo_left_images_{nullptr};
  QLineEdit* stereo_right_images_{nullptr};
  QLineEdit* stereo_left_intrinsics_{nullptr};
  QLineEdit* stereo_right_intrinsics_{nullptr};
  QLineEdit* stereo_output_{nullptr};
  QPushButton* stereo_run_{nullptr};
};

}  // namespace calibration_studio

