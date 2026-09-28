#include "camera_calibration_studio/workflow_pages.hpp"

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
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMetaObject>
#include <QMouseEvent>
#include <QPainter>
#include <QPointer>
#include <QPushButton>
#include <QSpinBox>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>

namespace calibration_studio {
namespace {

QString defaultSession() {
  return QString::fromStdString((std::filesystem::current_path() / "sessions").string()) +
         "/" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
}

QWidget* pathRow(QLineEdit*& edit, const QString& button_text,
                 const std::function<void()>& callback) {
  auto* widget = new QWidget;
  auto* layout = new QHBoxLayout(widget);
  layout->setContentsMargins(0, 0, 0, 0);
  edit = new QLineEdit;
  auto* button = new QPushButton(button_text);
  layout->addWidget(edit, 1);
  layout->addWidget(button);
  QObject::connect(button, &QPushButton::clicked, widget,
                   [callback](bool) { callback(); });
  return widget;
}

void configurePreview(QLabel* label, const QString& text) {
  label->setText(text);
  label->setAlignment(Qt::AlignCenter);
  label->setMinimumSize(440, 330);
  label->setStyleSheet(
      "QLabel { background: #202020; color: #dddddd; border: 1px solid #555; }");
}

void fillDeviceCombo(QComboBox* combo, const std::vector<CameraDescriptor>& devices,
                     const bool include_none) {
  combo->clear();
  if (include_none) {
    combo->addItem("None", QString{});
  }
  for (const auto& device : devices) {
    const QString label = QString::fromStdString(
        device.serial_number + "  " + device.model_name + "  [" + device.transport + "]");
    combo->addItem(label, QString::fromStdString(device.serial_number));
  }
}

QImage toQImage(const cv::Mat& image) {
  if (image.empty()) return {};
  if (image.channels() == 1) {
    return QImage(image.data, image.cols, image.rows, static_cast<int>(image.step),
                  QImage::Format_Grayscale8).copy();
  }
  cv::Mat rgb;
  cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
  return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
                QImage::Format_RGB888).copy();
}

}  // namespace

PreviewLabel::PreviewLabel(QWidget* parent) : QLabel(parent) {
  configurePreview(this, "Camera is stopped");
  setMouseTracking(true);
}

void PreviewLabel::showFrame(const cv::Mat& image) {
  if (image.empty()) return;
  last_image_ = image.clone();
  setPixmap(QPixmap::fromImage(toQImage(last_image_))
                .scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  update();
}

void PreviewLabel::setImageRoi(const cv::Rect& roi) {
  image_roi_ = roi;
  update();
}

QRect PreviewLabel::imageDisplayRect() const {
  if (last_image_.empty()) return {};
  const QSize scaled = QSize(last_image_.cols, last_image_.rows)
                           .scaled(contentsRect().size(), Qt::KeepAspectRatio);
  return QRect(contentsRect().center() - QPoint(scaled.width() / 2, scaled.height() / 2),
               scaled);
}

cv::Rect PreviewLabel::widgetRectToImage(const QRect& rectangle) const {
  const QRect display = imageDisplayRect();
  const QRect clipped = rectangle.normalized().intersected(display);
  if (clipped.width() < 3 || clipped.height() < 3 || last_image_.empty()) return {};
  const double scale_x = static_cast<double>(last_image_.cols) / display.width();
  const double scale_y = static_cast<double>(last_image_.rows) / display.height();
  return cv::Rect(static_cast<int>((clipped.left() - display.left()) * scale_x),
                  static_cast<int>((clipped.top() - display.top()) * scale_y),
                  std::max(1, static_cast<int>(clipped.width() * scale_x)),
                  std::max(1, static_cast<int>(clipped.height() * scale_y)));
}

void PreviewLabel::mousePressEvent(QMouseEvent* event) {
  if (selection_enabled_ && event->button() == Qt::LeftButton) {
    dragging_ = true;
    drag_start_ = event->pos();
    drag_end_ = drag_start_;
    update();
  }
  QLabel::mousePressEvent(event);
}

void PreviewLabel::mouseMoveEvent(QMouseEvent* event) {
  if (dragging_) {
    drag_end_ = event->pos();
    update();
  }
  QLabel::mouseMoveEvent(event);
}

void PreviewLabel::mouseReleaseEvent(QMouseEvent* event) {
  if (dragging_ && event->button() == Qt::LeftButton) {
    drag_end_ = event->pos();
    dragging_ = false;
    const cv::Rect selected = widgetRectToImage(QRect(drag_start_, drag_end_));
    if (selected.area() > 0 && roi_selected) roi_selected(selected);
    update();
  }
  QLabel::mouseReleaseEvent(event);
}

void PreviewLabel::paintEvent(QPaintEvent* event) {
  QLabel::paintEvent(event);
  QPainter painter(this);
  painter.setPen(QPen(QColor(0, 255, 80), 2));
  const QRect display = imageDisplayRect();
  if (!last_image_.empty() && image_roi_.area() > 0 && !display.isEmpty()) {
    const double sx = static_cast<double>(display.width()) / last_image_.cols;
    const double sy = static_cast<double>(display.height()) / last_image_.rows;
    painter.drawRect(QRect(display.left() + static_cast<int>(image_roi_.x * sx),
                           display.top() + static_cast<int>(image_roi_.y * sy),
                           static_cast<int>(image_roi_.width * sx),
                           static_cast<int>(image_roi_.height * sy)));
  }
  if (dragging_) {
    painter.setPen(QPen(QColor(255, 220, 0), 2, Qt::DashLine));
    painter.drawRect(QRect(drag_start_, drag_end_).normalized());
  }
}

MonoCameraPage::MonoCameraPage(QWidget* parent)
    : QWidget(parent), camera_(std::make_unique<HikCamera>()) {
  auto* root = new QVBoxLayout(this);
  auto* controls = new QHBoxLayout;
  device_ = new QComboBox;
  auto* refresh = new QPushButton("Refresh");
  start_stop_ = new QPushButton("Start camera");
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
  controls->addWidget(new QLabel("Camera:"));
  controls->addWidget(device_, 1);
  controls->addWidget(new QLabel("Exposure:"));
  controls->addWidget(exposure_);
  controls->addWidget(new QLabel("Gain:"));
  controls->addWidget(gain_);
  controls->addWidget(new QLabel("Rate:"));
  controls->addWidget(frame_rate_);
  controls->addWidget(new QLabel("Buffers:"));
  controls->addWidget(buffer_count_);
  controls->addWidget(refresh);
  controls->addWidget(start_stop_);
  root->addLayout(controls);
  connect(refresh, &QPushButton::clicked, this, &MonoCameraPage::refreshDevices);
  connect(start_stop_, &QPushButton::clicked, this, &MonoCameraPage::toggleCamera);

  auto* center = new QHBoxLayout;
  preview_ = new PreviewLabel;
  preview_->setSelectionEnabled(true);
  center->addWidget(preview_, 3);
  auto* focus_group = new QGroupBox("Manual focus");
  auto* focus_layout = new QVBoxLayout(focus_group);
  focus_values_ = new QLabel("Drag an ROI on the image to begin focus measurement.");
  focus_values_->setMinimumWidth(260);
  focus_values_->setWordWrap(true);
  roi_preview_ = new QLabel("Original ROI");
  configurePreview(roi_preview_, "Original ROI");
  roi_preview_->setMinimumSize(260, 200);
  roi_preview_->setMaximumSize(420, 360);
  auto* reset = new QPushButton("Reset focus peak");
  focus_layout->addWidget(focus_values_);
  focus_layout->addWidget(roi_preview_, 1);
  focus_layout->addWidget(reset);
  center->addWidget(focus_group, 1);
  root->addLayout(center, 1);
  connect(reset, &QPushButton::clicked, this, &MonoCameraPage::resetFocus);
  preview_->roi_selected = [this](const cv::Rect& roi) {
    focus_roi_ = roi;
    preview_->setImageRoi(roi);
    resetFocus();
  };

  auto* capture_group = new QGroupBox("Mono image capture");
  auto* capture_layout = new QVBoxLayout(capture_group);
  auto* session_row = new QHBoxLayout;
  session_ = new QLineEdit(defaultSession());
  auto* choose_session = new QPushButton("Choose session");
  auto* capture = new QPushButton("Capture mono image");
  session_row->addWidget(new QLabel("Session:"));
  session_row->addWidget(session_, 1);
  session_row->addWidget(choose_session);
  session_row->addWidget(capture);
  captures_ = new QListWidget;
  captures_->setMaximumHeight(90);
  capture_layout->addLayout(session_row);
  capture_layout->addWidget(captures_);
  root->addWidget(capture_group);
  connect(choose_session, &QPushButton::clicked, this, &MonoCameraPage::chooseSession);
  connect(capture, &QPushButton::clicked, this, &MonoCameraPage::captureImage);
  capture->setObjectName("captureButton");

  auto* calibration_group = new QGroupBox("7 x 7 circles mono calibration (30 mm spacing)");
  auto* form = new QFormLayout(calibration_group);
  form->addRow("Images:", pathRow(images_, "Browse", [this] { chooseImages(); }));
  camera_name_ = new QLineEdit("camera");
  form->addRow("Camera name:", camera_name_);
  form->addRow("Output YAML:", pathRow(output_, "Browse", [this] { chooseOutput(); }));
  calibrate_button_ = new QPushButton("Calibrate selected camera");
  form->addRow(calibrate_button_);
  root->addWidget(calibration_group);
  connect(calibrate_button_, &QPushButton::clicked, this, &MonoCameraPage::calibrate);

  status_ = new QLabel("Ready");
  root->addWidget(status_);
  timer_ = new QTimer(this);
  timer_->setInterval(100);
  connect(timer_, &QTimer::timeout, this, &MonoCameraPage::updateFrame);
  timer_->start();
  refreshDevices();
  setStreamingUi(false);
}

MonoCameraPage::~MonoCameraPage() { deactivate(); }

void MonoCameraPage::deactivate() {
  camera_->close();
  setStreamingUi(false);
}

CameraSettings MonoCameraPage::settings() const {
  CameraSettings value;
  value.exposure_time_us = exposure_->value();
  value.gain = gain_->value();
  value.frame_rate = frame_rate_->value();
  value.sdk_buffer_count = static_cast<unsigned int>(buffer_count_->value());
  return value;
}

void MonoCameraPage::refreshDevices() {
  if (camera_->running()) return;
  std::string error;
  const auto devices = HikCamera::enumerate(error);
  fillDeviceCombo(device_, devices, false);
  status(error.empty() ? QString("Found %1 camera(s)").arg(devices.size())
                       : QString::fromStdString(error), !error.empty());
}

void MonoCameraPage::toggleCamera() {
  if (camera_->running()) {
    deactivate();
    status("Camera stopped");
    return;
  }
  const QString serial = device_->currentData().toString();
  if (serial.isEmpty()) return status("Select a camera", true);
  std::string error;
  if (!camera_->open(serial.toStdString(), settings(), error)) {
    return status(QString::fromStdString(error), true);
  }
  focus_roi_ = {};
  focus_analyzer_.reset();
  setStreamingUi(true);
  status("Camera started; drag a green focus ROI on the image");
}

void MonoCameraPage::updateFrame() {
  CapturedFrame frame;
  if (!camera_->latestFrame(frame)) return;
  preview_->showFrame(frame.image);
  try {
    last_focus_ = focus_analyzer_.process(frame.image, focus_roi_);
    if (focus_roi_.area() == 0) {
      focus_roi_ = last_focus_.roi;
      preview_->setImageRoi(focus_roi_);
    }
    const bool exposure_warning = last_focus_.dark_ratio > 0.05 ||
                                  last_focus_.bright_ratio > 0.05;
    focus_values_->setText(
        QString("Current: %1\nSmoothed: %2\nPeak: %3\nRelative: %4 %\n"
                "Brightness: %5\nDark: %6 %\nBright: %7 %")
            .arg(last_focus_.current_score, 0, 'f', 1)
            .arg(last_focus_.smoothed_score, 0, 'f', 1)
            .arg(last_focus_.peak_score, 0, 'f', 1)
            .arg(last_focus_.relative_score * 100.0, 0, 'f', 1)
            .arg(last_focus_.mean_brightness, 0, 'f', 1)
            .arg(last_focus_.dark_ratio * 100.0, 0, 'f', 1)
            .arg(last_focus_.bright_ratio * 100.0, 0, 'f', 1));
    focus_values_->setStyleSheet(exposure_warning ? "color: #d08000; font-weight: bold;" : "");
    const cv::Mat roi = frame.image(last_focus_.roi);
    roi_preview_->setPixmap(QPixmap::fromImage(toQImage(roi)).scaled(
        roi_preview_->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
  } catch (const std::exception& exception) {
    status(QString::fromStdString(exception.what()), true);
  }
}

void MonoCameraPage::chooseSession() {
  const QString path = QFileDialog::getExistingDirectory(this, "Choose session", session_->text());
  if (!path.isEmpty()) session_->setText(path);
}

void MonoCameraPage::captureImage() {
  CapturedFrame frame;
  if (!camera_->latestFrame(frame)) return status("No current image", true);
  DatasetWriter writer(session_->text().toStdString());
  std::filesystem::path path;
  std::string error;
  if (!writer.saveMono(camera_->serialNumber(), frame.image, path, error)) {
    return status(QString::fromStdString(error), true);
  }
  captures_->addItem(QString("%1  focus=%2  relative=%3 %")
                         .arg(QString::fromStdString(path.string()))
                         .arg(last_focus_.smoothed_score, 0, 'f', 1)
                         .arg(last_focus_.relative_score * 100.0, 0, 'f', 1));
  images_->setText(QString::fromStdString(path.parent_path().string()));
  status("Saved " + QString::fromStdString(path.string()));
}

void MonoCameraPage::resetFocus() {
  focus_analyzer_.reset();
  status("Focus peak reset");
}

void MonoCameraPage::chooseImages() {
  const QString path = QFileDialog::getExistingDirectory(this, "Choose mono images");
  if (!path.isEmpty()) images_->setText(path);
}

void MonoCameraPage::chooseOutput() {
  const QString path = QFileDialog::getSaveFileName(this, "Save intrinsics", {}, "YAML (*.yaml)");
  if (!path.isEmpty()) output_->setText(path);
}

void MonoCameraPage::calibrate() {
  if (images_->text().isEmpty() || output_->text().isEmpty()) {
    return status("Choose images and output YAML", true);
  }
  calibrate_button_->setEnabled(false);
  status("Running mono calibration...");
  const QString images = images_->text();
  const QString output = output_->text();
  const QString name = camera_name_->text();
  QPointer<MonoCameraPage> self(this);
  auto* thread = QThread::create([self, images, output, name] {
    Intrinsics result;
    std::string error;
    bool ok = calibrateMono(listImages(images.toStdString()), {}, result, error);
    if (ok) ok = saveIntrinsics(output.toStdString(), name.toStdString(), result, error);
    const QString message = ok
        ? QString("Saved intrinsics: RMS=%1 px, used=%2, rejected=%3")
              .arg(result.rms_px, 0, 'f', 4).arg(result.used_images.size())
              .arg(result.rejected_images.size())
        : QString::fromStdString(error);
    if (self) QMetaObject::invokeMethod(self, [self, ok, message] {
      if (!self) return;
      self->calibrate_button_->setEnabled(true);
      self->status(message, !ok);
    });
  });
  connect(thread, &QThread::finished, thread, &QObject::deleteLater);
  thread->start();
}

void MonoCameraPage::setStreamingUi(const bool streaming) {
  start_stop_->setText(streaming ? "Stop camera" : "Start camera");
  device_->setEnabled(!streaming);
  exposure_->setEnabled(!streaming);
  gain_->setEnabled(!streaming);
  frame_rate_->setEnabled(!streaming);
  buffer_count_->setEnabled(!streaming);
  if (!streaming) preview_->setText("Camera is stopped");
}

void MonoCameraPage::status(const QString& text, const bool error) {
  status_->setText(text);
  status_->setStyleSheet(error ? "color: #c03030; font-weight: bold;" : "");
  if (error) QMessageBox::warning(this, "Mono camera", text);
}

StereoCameraPage::StereoCameraPage(QWidget* parent)
    : QWidget(parent), left_camera_(std::make_unique<HikCamera>()),
      right_camera_(std::make_unique<HikCamera>()) {
  auto* root = new QVBoxLayout(this);
  auto* controls = new QHBoxLayout;
  left_device_ = new QComboBox;
  right_device_ = new QComboBox;
  auto* refresh = new QPushButton("Refresh");
  start_stop_ = new QPushButton("Start stereo cameras");
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
  controls->addWidget(new QLabel("Left:"));
  controls->addWidget(left_device_, 1);
  controls->addWidget(new QLabel("Right:"));
  controls->addWidget(right_device_, 1);
  controls->addWidget(new QLabel("Exposure:"));
  controls->addWidget(exposure_);
  controls->addWidget(new QLabel("Gain:"));
  controls->addWidget(gain_);
  controls->addWidget(new QLabel("Rate:"));
  controls->addWidget(frame_rate_);
  controls->addWidget(refresh);
  controls->addWidget(start_stop_);
  root->addLayout(controls);
  connect(refresh, &QPushButton::clicked, this, &StereoCameraPage::refreshDevices);
  connect(start_stop_, &QPushButton::clicked, this, &StereoCameraPage::toggleCameras);

  auto* previews = new QHBoxLayout;
  left_preview_ = new PreviewLabel;
  right_preview_ = new PreviewLabel;
  previews->addWidget(left_preview_, 1);
  previews->addWidget(right_preview_, 1);
  root->addLayout(previews, 1);
  pair_status_ = new QLabel("Waiting for stereo frames");
  pair_status_->setAlignment(Qt::AlignCenter);
  root->addWidget(pair_status_);

  auto* capture_group = new QGroupBox("Stereo pair capture");
  auto* capture_layout = new QVBoxLayout(capture_group);
  auto* session_row = new QHBoxLayout;
  session_ = new QLineEdit(defaultSession());
  auto* choose_session = new QPushButton("Choose session");
  auto* capture = new QPushButton("Capture stereo pair");
  session_row->addWidget(new QLabel("Session:"));
  session_row->addWidget(session_, 1);
  session_row->addWidget(choose_session);
  session_row->addWidget(capture);
  captures_ = new QListWidget;
  captures_->setMaximumHeight(90);
  capture_layout->addLayout(session_row);
  capture_layout->addWidget(captures_);
  root->addWidget(capture_group);
  connect(choose_session, &QPushButton::clicked, this, &StereoCameraPage::chooseSession);
  connect(capture, &QPushButton::clicked, this, &StereoCameraPage::capturePair);

  auto* calibration_group = new QGroupBox("Four-ArUco stereo calibration");
  auto* form = new QFormLayout(calibration_group);
  form->addRow("Left images:", pathRow(left_images_, "Browse", [this] { chooseLeftImages(); }));
  form->addRow("Right images:", pathRow(right_images_, "Browse", [this] { chooseRightImages(); }));
  form->addRow("Left intrinsics:",
               pathRow(left_intrinsics_, "Browse", [this] { chooseLeftIntrinsics(); }));
  form->addRow("Right intrinsics:",
               pathRow(right_intrinsics_, "Browse", [this] { chooseRightIntrinsics(); }));
  form->addRow("Output YAML:", pathRow(output_, "Browse", [this] { chooseOutput(); }));
  calibrate_button_ = new QPushButton("Calibrate stereo cameras");
  form->addRow(calibrate_button_);
  root->addWidget(calibration_group);
  connect(calibrate_button_, &QPushButton::clicked, this, &StereoCameraPage::calibrate);

  status_ = new QLabel("Ready");
  root->addWidget(status_);
  timer_ = new QTimer(this);
  timer_->setInterval(100);
  connect(timer_, &QTimer::timeout, this, &StereoCameraPage::updateFrames);
  timer_->start();
  refreshDevices();
  setStreamingUi(false);
}

StereoCameraPage::~StereoCameraPage() { deactivate(); }

void StereoCameraPage::deactivate() {
  left_camera_->close();
  right_camera_->close();
  setStreamingUi(false);
}

CameraSettings StereoCameraPage::settings() const {
  CameraSettings value;
  value.exposure_time_us = exposure_->value();
  value.gain = gain_->value();
  value.frame_rate = frame_rate_->value();
  value.sdk_buffer_count = static_cast<unsigned int>(buffer_count_->value());
  return value;
}

void StereoCameraPage::refreshDevices() {
  if (left_camera_->running() || right_camera_->running()) return;
  std::string error;
  const auto devices = HikCamera::enumerate(error);
  fillDeviceCombo(left_device_, devices, false);
  fillDeviceCombo(right_device_, devices, true);
  if (right_device_->count() > 2) right_device_->setCurrentIndex(2);
  status(error.empty() ? QString("Found %1 camera(s)").arg(devices.size())
                       : QString::fromStdString(error), !error.empty());
}

void StereoCameraPage::toggleCameras() {
  if (left_camera_->running() || right_camera_->running()) {
    deactivate();
    status("Stereo cameras stopped");
    return;
  }
  const QString left = left_device_->currentData().toString();
  const QString right = right_device_->currentData().toString();
  if (left.isEmpty() || right.isEmpty()) return status("Select both cameras", true);
  if (left == right) return status("Left and right cameras must differ", true);
  std::string error;
  const auto camera_settings = settings();
  if (!left_camera_->open(left.toStdString(), camera_settings, error)) {
    return status(QString::fromStdString(error), true);
  }
  if (!right_camera_->open(right.toStdString(), camera_settings, error)) {
    left_camera_->close();
    return status(QString::fromStdString(error), true);
  }
  setStreamingUi(true);
  status("Stereo cameras started; keep the calibration target stationary before capture");
}

void StereoCameraPage::updateFrames() {
  CapturedFrame left;
  CapturedFrame right;
  const bool have_left = left_camera_->latestFrame(left);
  const bool have_right = right_camera_->latestFrame(right);
  if (have_left) left_preview_->showFrame(left.image);
  if (have_right) right_preview_->showFrame(right.image);
  if (have_left && have_right) {
    const double delta_ms = std::abs(
        std::chrono::duration<double>(left.received_at - right.received_at).count()) * 1000.0;
    pair_status_->setText(QString("Latest receive-time delta: %1 ms").arg(delta_ms, 0, 'f', 1));
    pair_status_->setStyleSheet(delta_ms > 150.0 ? "color: #c03030; font-weight: bold;" : "");
  }
}

void StereoCameraPage::chooseSession() {
  const QString path = QFileDialog::getExistingDirectory(this, "Choose session", session_->text());
  if (!path.isEmpty()) session_->setText(path);
}

void StereoCameraPage::capturePair() {
  CapturedFrame left;
  CapturedFrame right;
  if (!left_camera_->latestFrame(left) || !right_camera_->latestFrame(right)) {
    return status("Both cameras need a current frame", true);
  }
  const auto now = std::chrono::steady_clock::now();
  const double left_age = std::chrono::duration<double>(now - left.received_at).count();
  const double right_age = std::chrono::duration<double>(now - right.received_at).count();
  const double delta = std::abs(
      std::chrono::duration<double>(left.received_at - right.received_at).count());
  if (left_age > 0.5 || right_age > 0.5 || delta > 0.15) {
    return status(QString("Frames are stale or unsynchronized: %1 ms").arg(delta * 1000.0, 0, 'f', 1),
                  true);
  }
  DatasetWriter writer(session_->text().toStdString());
  std::filesystem::path left_path;
  std::filesystem::path right_path;
  std::string error;
  if (!writer.saveStereoPair(left.image, right.image, left_path, right_path, error)) {
    return status(QString::fromStdString(error), true);
  }
  captures_->addItem(QString("%1  delta=%2 ms")
                         .arg(QString::fromStdString(left_path.filename().string()))
                         .arg(delta * 1000.0, 0, 'f', 1));
  left_images_->setText(QString::fromStdString(left_path.parent_path().string()));
  right_images_->setText(QString::fromStdString(right_path.parent_path().string()));
  status("Saved stereo pair " + QString::fromStdString(left_path.filename().string()));
}

void StereoCameraPage::chooseLeftImages() {
  const QString path = QFileDialog::getExistingDirectory(this, "Choose left images");
  if (!path.isEmpty()) left_images_->setText(path);
}
void StereoCameraPage::chooseRightImages() {
  const QString path = QFileDialog::getExistingDirectory(this, "Choose right images");
  if (!path.isEmpty()) right_images_->setText(path);
}
void StereoCameraPage::chooseLeftIntrinsics() {
  const QString path = QFileDialog::getOpenFileName(this, "Choose left intrinsics", {}, "YAML (*.yaml)");
  if (!path.isEmpty()) left_intrinsics_->setText(path);
}
void StereoCameraPage::chooseRightIntrinsics() {
  const QString path = QFileDialog::getOpenFileName(this, "Choose right intrinsics", {}, "YAML (*.yaml)");
  if (!path.isEmpty()) right_intrinsics_->setText(path);
}
void StereoCameraPage::chooseOutput() {
  const QString path = QFileDialog::getSaveFileName(this, "Save stereo calibration", {}, "YAML (*.yaml)");
  if (!path.isEmpty()) output_->setText(path);
}

void StereoCameraPage::calibrate() {
  if (left_images_->text().isEmpty() || right_images_->text().isEmpty() ||
      left_intrinsics_->text().isEmpty() || right_intrinsics_->text().isEmpty() ||
      output_->text().isEmpty()) {
    return status("Choose image directories, both intrinsics, and output YAML", true);
  }
  calibrate_button_->setEnabled(false);
  status("Running stereo calibration...");
  const QString left_images = left_images_->text();
  const QString right_images = right_images_->text();
  const QString left_file = left_intrinsics_->text();
  const QString right_file = right_intrinsics_->text();
  const QString output = output_->text();
  QPointer<StereoCameraPage> self(this);
  auto* thread = QThread::create([self, left_images, right_images, left_file, right_file, output] {
    Intrinsics left;
    Intrinsics right;
    StereoResult result;
    std::string error;
    bool ok = loadIntrinsics(left_file.toStdString(), left, error) &&
              loadIntrinsics(right_file.toStdString(), right, error) &&
              calibrateStereoAruco(left_images.toStdString(), right_images.toStdString(),
                                   left, right, {}, result, error);
    if (ok) ok = saveStereoResult(output.toStdString(), result, error);
    const QString message = ok
        ? QString("Saved stereo result: RMS=%1 px, baseline=%2 m, pairs=%3")
              .arg(result.rms_px, 0, 'f', 4).arg(result.baseline_m, 0, 'f', 6)
              .arg(result.used_pairs)
        : QString::fromStdString(error);
    if (self) QMetaObject::invokeMethod(self, [self, ok, message] {
      if (!self) return;
      self->calibrate_button_->setEnabled(true);
      self->status(message, !ok);
    });
  });
  connect(thread, &QThread::finished, thread, &QObject::deleteLater);
  thread->start();
}

void StereoCameraPage::setStreamingUi(const bool streaming) {
  start_stop_->setText(streaming ? "Stop stereo cameras" : "Start stereo cameras");
  left_device_->setEnabled(!streaming);
  right_device_->setEnabled(!streaming);
  exposure_->setEnabled(!streaming);
  gain_->setEnabled(!streaming);
  frame_rate_->setEnabled(!streaming);
  buffer_count_->setEnabled(!streaming);
  if (!streaming) {
    left_preview_->setText("Left camera is stopped");
    right_preview_->setText("Right camera is stopped");
    pair_status_->setText("Waiting for stereo frames");
  }
}

void StereoCameraPage::status(const QString& text, const bool error) {
  status_->setText(text);
  status_->setStyleSheet(error ? "color: #c03030; font-weight: bold;" : "");
  if (error) QMessageBox::warning(this, "Stereo cameras", text);
}

}  // namespace calibration_studio

