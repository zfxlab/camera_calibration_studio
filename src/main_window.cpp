#include "camera_calibration_studio/main_window.hpp"

#include "camera_calibration_studio/calibration.hpp"
#include "camera_calibration_studio/dataset.hpp"

#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMetaObject>
#include <QPointer>
#include <QPushButton>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <filesystem>

namespace calibration_studio {
namespace {

QWidget* pathRow(QLineEdit*& edit, const QString& button_text, QObject* receiver,
                 const char* slot) {
  auto* widget = new QWidget;
  auto* layout = new QHBoxLayout(widget);
  layout->setContentsMargins(0, 0, 0, 0);
  edit = new QLineEdit;
  auto* button = new QPushButton(button_text);
  layout->addWidget(edit, 1);
  layout->addWidget(button);
  QObject::connect(button, SIGNAL(clicked(bool)), receiver, slot);
  return widget;
}

QString defaultSessionDirectory() {
  return QString::fromStdString((std::filesystem::current_path() / "sessions").string()) + "/" +
         QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
}

}  // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), left_camera_(std::make_unique<HikCamera>()),
      right_camera_(std::make_unique<HikCamera>()) {
  buildUi();
  refreshDevices();
  preview_timer_ = new QTimer(this);
  preview_timer_->setInterval(100);
  connect(preview_timer_, &QTimer::timeout, this, &MainWindow::updatePreviews);
  preview_timer_->start();
}

MainWindow::~MainWindow() {
  left_camera_->close();
  right_camera_->close();
}

void MainWindow::buildUi() {
  setWindowTitle("Camera Calibration Studio");
  resize(1400, 900);
  auto* tabs = new QTabWidget;
  setCentralWidget(tabs);

  auto* capture_tab = new QWidget;
  auto* capture_layout = new QVBoxLayout(capture_tab);
  auto* device_group = new QGroupBox("MVS cameras");
  auto* device_layout = new QHBoxLayout(device_group);
  left_device_ = new QComboBox;
  right_device_ = new QComboBox;
  auto* refresh = new QPushButton("Refresh");
  start_stop_ = new QPushButton("Start cameras");
  exposure_ = new QDoubleSpinBox;
  exposure_->setRange(10.0, 1000000.0);
  exposure_->setValue(10000.0);
  exposure_->setSuffix(" us");
  gain_ = new QDoubleSpinBox;
  gain_->setRange(0.0, 30.0);
  frame_rate_ = new QDoubleSpinBox;
  frame_rate_->setRange(0.1, 200.0);
  frame_rate_->setValue(10.0);
  frame_rate_->setSuffix(" FPS");
  buffer_count_ = new QSpinBox;
  buffer_count_->setRange(1, 30);
  buffer_count_->setValue(4);
  device_layout->addWidget(new QLabel("Left/mono:"));
  device_layout->addWidget(left_device_, 1);
  device_layout->addWidget(new QLabel("Right:"));
  device_layout->addWidget(right_device_, 1);
  device_layout->addWidget(new QLabel("Exposure:"));
  device_layout->addWidget(exposure_);
  device_layout->addWidget(new QLabel("Gain:"));
  device_layout->addWidget(gain_);
  device_layout->addWidget(new QLabel("Rate:"));
  device_layout->addWidget(frame_rate_);
  device_layout->addWidget(new QLabel("Buffers:"));
  device_layout->addWidget(buffer_count_);
  device_layout->addWidget(refresh);
  device_layout->addWidget(start_stop_);
  capture_layout->addWidget(device_group);
  connect(refresh, &QPushButton::clicked, this, &MainWindow::refreshDevices);
  connect(start_stop_, &QPushButton::clicked, this, &MainWindow::toggleCameras);

  auto* previews = new QHBoxLayout;
  left_preview_ = new QLabel("Left camera is stopped");
  right_preview_ = new QLabel("Right camera is stopped");
  for (auto* label : {left_preview_, right_preview_}) {
    label->setAlignment(Qt::AlignCenter);
    label->setMinimumSize(480, 360);
    label->setStyleSheet("QLabel { background: #202020; color: #dddddd; border: 1px solid #555; }");
    previews->addWidget(label, 1);
  }
  capture_layout->addLayout(previews, 1);

  auto* save_group = new QGroupBox("Dataset capture");
  auto* save_layout = new QVBoxLayout(save_group);
  auto* session_row = new QHBoxLayout;
  session_directory_ = new QLineEdit(defaultSessionDirectory());
  auto* choose_session = new QPushButton("Choose session");
  session_row->addWidget(new QLabel("Session:"));
  session_row->addWidget(session_directory_, 1);
  session_row->addWidget(choose_session);
  save_layout->addLayout(session_row);
  connect(choose_session, &QPushButton::clicked, this, &MainWindow::chooseSessionDirectory);
  auto* capture_buttons = new QHBoxLayout;
  capture_left_ = new QPushButton("Capture left mono");
  capture_right_ = new QPushButton("Capture right mono");
  capture_pair_ = new QPushButton("Capture stereo pair");
  capture_buttons->addWidget(capture_left_);
  capture_buttons->addWidget(capture_right_);
  capture_buttons->addWidget(capture_pair_);
  save_layout->addLayout(capture_buttons);
  connect(capture_left_, &QPushButton::clicked, this, &MainWindow::captureLeftMono);
  connect(capture_right_, &QPushButton::clicked, this, &MainWindow::captureRightMono);
  connect(capture_pair_, &QPushButton::clicked, this, &MainWindow::captureStereoPair);
  capture_list_ = new QListWidget;
  capture_list_->setMaximumHeight(130);
  save_layout->addWidget(capture_list_);
  capture_layout->addWidget(save_group);
  setCaptureControlsEnabled(false);
  tabs->addTab(capture_tab, "Capture");

  auto* mono_tab = new QWidget;
  auto* mono_layout = new QFormLayout(mono_tab);
  mono_layout->addRow("Circle-grid images:",
                      pathRow(mono_images_, "Browse", this, SLOT(chooseMonoImages())));
  mono_layout->addRow("Camera name:", mono_camera_name_ = new QLineEdit("camera"));
  mono_layout->addRow("Output CameraInfo YAML:",
                      pathRow(mono_output_, "Browse", this, SLOT(chooseMonoOutput())));
  auto* mono_description = new QLabel(
      "Uses a 7 x 7 symmetric circle grid with 0.03 m center spacing. "
      "At least 8 valid images are required; 20-30 diverse views are recommended.");
  mono_description->setWordWrap(true);
  mono_layout->addRow(mono_description);
  mono_run_ = new QPushButton("Calibrate mono camera");
  mono_layout->addRow(mono_run_);
  connect(mono_run_, &QPushButton::clicked, this, &MainWindow::runMonoCalibration);
  tabs->addTab(mono_tab, "Mono calibration");

  auto* stereo_tab = new QWidget;
  auto* stereo_layout = new QFormLayout(stereo_tab);
  stereo_layout->addRow("Left images:",
                        pathRow(stereo_left_images_, "Browse", this,
                                SLOT(chooseStereoLeftImages())));
  stereo_layout->addRow("Right images:",
                        pathRow(stereo_right_images_, "Browse", this,
                                SLOT(chooseStereoRightImages())));
  stereo_layout->addRow("Left intrinsics:",
                        pathRow(stereo_left_intrinsics_, "Browse", this,
                                SLOT(chooseLeftIntrinsics())));
  stereo_layout->addRow("Right intrinsics:",
                        pathRow(stereo_right_intrinsics_, "Browse", this,
                                SLOT(chooseRightIntrinsics())));
  stereo_layout->addRow("Output stereo YAML:",
                        pathRow(stereo_output_, "Browse", this,
                                SLOT(chooseStereoOutput())));
  auto* stereo_description = new QLabel(
      "Uses DICT_6X6_50 markers [1,2,3,4], marker length 0.20 m, and center "
      "spacing 1.10 x 0.70 m. Intrinsics remain fixed during stereo calibration.");
  stereo_description->setWordWrap(true);
  stereo_layout->addRow(stereo_description);
  stereo_run_ = new QPushButton("Calibrate stereo cameras");
  stereo_layout->addRow(stereo_run_);
  connect(stereo_run_, &QPushButton::clicked, this, &MainWindow::runStereoCalibration);
  tabs->addTab(stereo_tab, "Stereo calibration");

  statusBar()->showMessage("Ready");
}

CameraSettings MainWindow::settingsFromUi() const {
  CameraSettings settings;
  settings.exposure_time_us = exposure_->value();
  settings.gain = gain_->value();
  settings.frame_rate = frame_rate_->value();
  settings.sdk_buffer_count = static_cast<unsigned int>(buffer_count_->value());
  return settings;
}

void MainWindow::refreshDevices() {
  if (left_camera_->running() || right_camera_->running()) {
    showStatus("Stop cameras before refreshing the device list", true);
    return;
  }
  std::string error;
  devices_ = HikCamera::enumerate(error);
  left_device_->clear();
  right_device_->clear();
  right_device_->addItem("None", QString{});
  for (const auto& device : devices_) {
    const QString label = QString::fromStdString(device.serial_number + "  " + device.model_name +
                                                 "  [" + device.transport + "]");
    const QString serial = QString::fromStdString(device.serial_number);
    left_device_->addItem(label, serial);
    right_device_->addItem(label, serial);
  }
  if (right_device_->count() > 2) {
    right_device_->setCurrentIndex(2);
  }
  showStatus(error.empty() ? QString("Found %1 MVS camera(s)").arg(devices_.size())
                           : QString::fromStdString(error),
             !error.empty());
}

void MainWindow::toggleCameras() {
  if (left_camera_->running() || right_camera_->running()) {
    left_camera_->close();
    right_camera_->close();
    start_stop_->setText("Start cameras");
    setCaptureControlsEnabled(false);
    showStatus("Cameras stopped");
    return;
  }
  const QString left_serial = left_device_->currentData().toString();
  const QString right_serial = right_device_->currentData().toString();
  if (left_serial.isEmpty()) {
    showStatus("Select a left/mono camera", true);
    return;
  }
  if (!right_serial.isEmpty() && left_serial == right_serial) {
    showStatus("Left and right cameras must be different", true);
    return;
  }
  std::string error;
  const auto settings = settingsFromUi();
  if (!left_camera_->open(left_serial.toStdString(), settings, error)) {
    showStatus(QString::fromStdString(error), true);
    return;
  }
  if (!right_serial.isEmpty() && !right_camera_->open(right_serial.toStdString(), settings, error)) {
    left_camera_->close();
    showStatus(QString::fromStdString(error), true);
    return;
  }
  start_stop_->setText("Stop cameras");
  setCaptureControlsEnabled(true);
  showStatus(right_serial.isEmpty() ? "Mono camera started" : "Stereo cameras started");
}

void MainWindow::updatePreviews() {
  CapturedFrame frame;
  if (left_camera_->latestFrame(frame)) {
    setPreview(left_preview_, frame.image);
  }
  if (right_camera_->latestFrame(frame)) {
    setPreview(right_preview_, frame.image);
  }
}

void MainWindow::setPreview(QLabel* label, const cv::Mat& image) {
  if (image.empty()) {
    return;
  }
  QImage view;
  if (image.channels() == 1) {
    view = QImage(image.data, image.cols, image.rows, static_cast<int>(image.step),
                  QImage::Format_Grayscale8).copy();
  } else {
    cv::Mat rgb;
    cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
    view = QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
                  QImage::Format_RGB888).copy();
  }
  label->setPixmap(QPixmap::fromImage(view).scaled(label->size(), Qt::KeepAspectRatio,
                                                   Qt::SmoothTransformation));
}

void MainWindow::chooseSessionDirectory() {
  const QString path = QFileDialog::getExistingDirectory(this, "Choose session directory",
                                                          session_directory_->text());
  if (!path.isEmpty()) {
    session_directory_->setText(path);
  }
}

void MainWindow::captureLeftMono() {
  CapturedFrame frame;
  if (!left_camera_->latestFrame(frame)) {
    showStatus("No left frame is available", true);
    return;
  }
  DatasetWriter writer(session_directory_->text().toStdString());
  std::filesystem::path path;
  std::string error;
  const std::string label = left_camera_->serialNumber().empty() ? "left" : left_camera_->serialNumber();
  if (!writer.saveMono(label, frame.image, path, error)) {
    showStatus(QString::fromStdString(error), true);
    return;
  }
  capture_list_->addItem(QString::fromStdString(path.string()));
  mono_images_->setText(QString::fromStdString(path.parent_path().string()));
  showStatus("Saved " + QString::fromStdString(path.string()));
}

void MainWindow::captureRightMono() {
  CapturedFrame frame;
  if (!right_camera_->latestFrame(frame)) {
    showStatus("No right frame is available", true);
    return;
  }
  DatasetWriter writer(session_directory_->text().toStdString());
  std::filesystem::path path;
  std::string error;
  const std::string label = right_camera_->serialNumber().empty() ? "right" : right_camera_->serialNumber();
  if (!writer.saveMono(label, frame.image, path, error)) {
    showStatus(QString::fromStdString(error), true);
    return;
  }
  capture_list_->addItem(QString::fromStdString(path.string()));
  mono_images_->setText(QString::fromStdString(path.parent_path().string()));
  showStatus("Saved " + QString::fromStdString(path.string()));
}

void MainWindow::captureStereoPair() {
  CapturedFrame left;
  CapturedFrame right;
  if (!left_camera_->latestFrame(left) || !right_camera_->latestFrame(right)) {
    showStatus("Both cameras must have a current frame", true);
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto left_age = std::chrono::duration<double>(now - left.received_at).count();
  const auto right_age = std::chrono::duration<double>(now - right.received_at).count();
  const auto pair_delta = std::abs(std::chrono::duration<double>(left.received_at - right.received_at).count());
  if (left_age > 0.5 || right_age > 0.5 || pair_delta > 0.15) {
    showStatus(QString("Stereo frames are stale or unsynchronized (delta=%1 ms)")
                   .arg(pair_delta * 1000.0, 0, 'f', 1),
               true);
    return;
  }
  DatasetWriter writer(session_directory_->text().toStdString());
  std::filesystem::path left_path;
  std::filesystem::path right_path;
  std::string error;
  if (!writer.saveStereoPair(left.image, right.image, left_path, right_path, error)) {
    showStatus(QString::fromStdString(error), true);
    return;
  }
  capture_list_->addItem(QString("Pair: %1").arg(QString::fromStdString(left_path.filename().string())));
  stereo_left_images_->setText(QString::fromStdString(left_path.parent_path().string()));
  stereo_right_images_->setText(QString::fromStdString(right_path.parent_path().string()));
  showStatus(QString("Saved stereo pair; receive delta=%1 ms").arg(pair_delta * 1000.0, 0, 'f', 1));
}

void MainWindow::chooseMonoImages() {
  const QString path = QFileDialog::getExistingDirectory(this, "Choose mono image directory");
  if (!path.isEmpty()) mono_images_->setText(path);
}

void MainWindow::chooseMonoOutput() {
  const QString path = QFileDialog::getSaveFileName(this, "Save camera calibration", {}, "YAML (*.yaml)");
  if (!path.isEmpty()) mono_output_->setText(path);
}

void MainWindow::runMonoCalibration() {
  const QString images = mono_images_->text();
  const QString output = mono_output_->text();
  const QString name = mono_camera_name_->text();
  if (images.isEmpty() || output.isEmpty()) {
    showStatus("Choose mono images and an output file", true);
    return;
  }
  mono_run_->setEnabled(false);
  showStatus("Running mono calibration...");
  QPointer<MainWindow> self(this);
  auto* thread = QThread::create([self, images, output, name] {
    Intrinsics result;
    std::string error;
    bool success = calibrateMono(listImages(images.toStdString()), {}, result, error);
    if (success) {
      success = saveIntrinsics(output.toStdString(), name.toStdString(), result, error);
    }
    const QString message = success
                                ? QString("Mono calibration saved: RMS=%1 px, used=%2, rejected=%3")
                                      .arg(result.rms_px, 0, 'f', 4)
                                      .arg(result.used_images.size())
                                      .arg(result.rejected_images.size())
                                : QString::fromStdString(error);
    if (self) {
      QMetaObject::invokeMethod(self, [self, success, message] {
        if (!self) return;
        self->mono_run_->setEnabled(true);
        self->showStatus(message, !success);
      });
    }
  });
  connect(thread, &QThread::finished, thread, &QObject::deleteLater);
  thread->start();
}

void MainWindow::chooseStereoLeftImages() {
  const QString path = QFileDialog::getExistingDirectory(this, "Choose left image directory");
  if (!path.isEmpty()) stereo_left_images_->setText(path);
}

void MainWindow::chooseStereoRightImages() {
  const QString path = QFileDialog::getExistingDirectory(this, "Choose right image directory");
  if (!path.isEmpty()) stereo_right_images_->setText(path);
}

void MainWindow::chooseLeftIntrinsics() {
  const QString path = QFileDialog::getOpenFileName(this, "Choose left intrinsics", {}, "YAML (*.yaml)");
  if (!path.isEmpty()) stereo_left_intrinsics_->setText(path);
}

void MainWindow::chooseRightIntrinsics() {
  const QString path = QFileDialog::getOpenFileName(this, "Choose right intrinsics", {}, "YAML (*.yaml)");
  if (!path.isEmpty()) stereo_right_intrinsics_->setText(path);
}

void MainWindow::chooseStereoOutput() {
  const QString path = QFileDialog::getSaveFileName(this, "Save stereo calibration", {}, "YAML (*.yaml)");
  if (!path.isEmpty()) stereo_output_->setText(path);
}

void MainWindow::runStereoCalibration() {
  const QString left_images = stereo_left_images_->text();
  const QString right_images = stereo_right_images_->text();
  const QString left_file = stereo_left_intrinsics_->text();
  const QString right_file = stereo_right_intrinsics_->text();
  const QString output = stereo_output_->text();
  if (left_images.isEmpty() || right_images.isEmpty() || left_file.isEmpty() ||
      right_file.isEmpty() || output.isEmpty()) {
    showStatus("Choose stereo image directories, both intrinsics, and an output file", true);
    return;
  }
  stereo_run_->setEnabled(false);
  showStatus("Running stereo calibration...");
  QPointer<MainWindow> self(this);
  auto* thread = QThread::create([self, left_images, right_images, left_file, right_file, output] {
    Intrinsics left;
    Intrinsics right;
    StereoResult result;
    std::string error;
    bool success = loadIntrinsics(left_file.toStdString(), left, error) &&
                   loadIntrinsics(right_file.toStdString(), right, error) &&
                   calibrateStereoAruco(left_images.toStdString(), right_images.toStdString(),
                                        left, right, {}, result, error);
    if (success) {
      success = saveStereoResult(output.toStdString(), result, error);
    }
    const QString message = success
                                ? QString("Stereo calibration saved: RMS=%1 px, baseline=%2 m, pairs=%3")
                                      .arg(result.rms_px, 0, 'f', 4)
                                      .arg(result.baseline_m, 0, 'f', 6)
                                      .arg(result.used_pairs)
                                : QString::fromStdString(error);
    if (self) {
      QMetaObject::invokeMethod(self, [self, success, message] {
        if (!self) return;
        self->stereo_run_->setEnabled(true);
        self->showStatus(message, !success);
      });
    }
  });
  connect(thread, &QThread::finished, thread, &QObject::deleteLater);
  thread->start();
}

void MainWindow::setCaptureControlsEnabled(const bool enabled) {
  capture_left_->setEnabled(enabled);
  capture_right_->setEnabled(enabled && right_camera_->running());
  capture_pair_->setEnabled(enabled && right_camera_->running());
  left_device_->setEnabled(!enabled);
  right_device_->setEnabled(!enabled);
  exposure_->setEnabled(!enabled);
  gain_->setEnabled(!enabled);
  frame_rate_->setEnabled(!enabled);
  buffer_count_->setEnabled(!enabled);
}

void MainWindow::showStatus(const QString& message, const bool error) {
  statusBar()->showMessage(message, 15000);
  if (error) {
    QMessageBox::warning(this, "Camera Calibration Studio", message);
  }
}

}  // namespace calibration_studio
