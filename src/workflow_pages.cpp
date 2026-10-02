#include "camera_calibration_studio/workflow_pages.hpp"

#include "camera_calibration_studio/calibration.hpp"
#include "camera_calibration_studio/dataset.hpp"

#include <QComboBox>
#include <QBrush>
#include <QCheckBox>
#include <QColor>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QListView>
#include <QMessageBox>
#include <QMetaObject>
#include <QMouseEvent>
#include <QPainter>
#include <QPointer>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStringList>
#include <QThread>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>

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

void configureReviewList(QListWidget* list) {
  list->setViewMode(QListView::ListMode);
  list->setMovement(QListView::Static);
  list->setAlternatingRowColors(true);
  list->setUniformItemSizes(true);
}

QString monoDecisionText(const MonoFrameAnalysis& result) {
  switch (result.decision) {
    case MonoFrameDecision::keep:
      return result.low_sharpness || result.exposure_warning ? "KEEP/WARN" : "KEEP";
    case MonoFrameDecision::duplicate:
      return "SKIP/DUPLICATE";
    case MonoFrameDecision::redundant:
      return "SKIP/REDUNDANT";
    case MonoFrameDecision::unreadable:
      return "SKIP/UNREADABLE";
    case MonoFrameDecision::size_mismatch:
      return "SKIP/SIZE";
    case MonoFrameDecision::pattern_not_found:
      return "SKIP/NO GRID";
  }
  return "SKIP";
}

QString monoAnalysisText(const MonoFrameAnalysis& result) {
  QString text = QString("%1  [%2]  grid %3/49  sharp %4  mean %5")
                     .arg(QString::fromStdString(result.path.filename().string()))
                     .arg(monoDecisionText(result))
                     .arg(result.detected_points)
                     .arg(result.sharpness, 0, 'f', 0)
                     .arg(result.mean_brightness, 0, 'f', 1);
  if (result.decision == MonoFrameDecision::keep) {
    text += QString("  center (%1, %2)  coverage %3%  roll %4 deg")
                .arg(result.normalized_center.x, 0, 'f', 2)
                .arg(result.normalized_center.y, 0, 'f', 2)
                .arg(result.image_coverage * 100.0, 0, 'f', 1)
                .arg(result.board_angle_deg, 0, 'f', 1);
    if (result.pose_estimated) {
      text += QString("  tilt x %1 / y %2 / total %3 deg  pose err %4 px")
                  .arg(result.tilt_x_deg, 0, 'f', 1)
                  .arg(result.tilt_y_deg, 0, 'f', 1)
                  .arg(result.total_tilt_deg, 0, 'f', 1)
                  .arg(result.pose_reprojection_error_px, 0, 'f', 2);
    } else {
      text += "  tilt unavailable";
    }
    if (result.low_sharpness) text += "  low sharpness";
    if (result.exposure_warning) text += "  exposure warning";
  } else if (result.decision == MonoFrameDecision::duplicate ||
             result.decision == MonoFrameDecision::redundant) {
    text += QString("  nearest selected %1 (distance %2)")
                .arg(QString::fromStdString(result.duplicate_of.filename().string()))
                .arg(result.duplicate_distance, 0, 'f', 4);
    if (result.pose_estimated) {
      text += QString("  tilt x %1 / y %2 / total %3 deg")
                  .arg(result.tilt_x_deg, 0, 'f', 1)
                  .arg(result.tilt_y_deg, 0, 'f', 1)
                  .arg(result.total_tilt_deg, 0, 'f', 1);
    }
  } else if (!result.image_size.empty()) {
    text += QString("  size %1x%2").arg(result.image_size.width).arg(result.image_size.height);
  }
  return text;
}

void showReviewImage(PreviewLabel* preview, const QString& path,
                     const bool draw_circle_grid) {
  if (preview == nullptr || path.isEmpty()) return;
  const cv::Mat gray = cv::imread(path.toStdString(), cv::IMREAD_GRAYSCALE);
  if (gray.empty()) {
    preview->clear();
    preview->setText("Unable to read image");
    return;
  }
  cv::Mat display;
  cv::cvtColor(gray, display, cv::COLOR_GRAY2BGR);
  if (draw_circle_grid) {
    std::vector<cv::Point2f> centers;
    const bool found = cv::findCirclesGrid(
        gray, cv::Size(7, 7), centers,
        cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
    cv::drawChessboardCorners(display, cv::Size(7, 7), centers, found);
  }
  preview->showFrame(display);
}

QString recordingName(const QString& prefix) {
  return prefix + '_' + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
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
  stages_ = new QTabWidget;
  auto* live_page = new QWidget;
  auto* live_root = new QVBoxLayout(live_page);
  auto* review_page = new QWidget;
  auto* review_root = new QVBoxLayout(review_page);
  auto* calibration_page = new QWidget;
  auto* calibration_root = new QVBoxLayout(calibration_page);
  stages_->addTab(live_page, "1  Live capture");
  stages_->addTab(review_page, "2  Review frames");
  stages_->addTab(calibration_page, "3  Calibration");
  root->addWidget(stages_, 1);
  connect(stages_, &QTabWidget::currentChanged, this, [this](const int index) {
    if (index != 0 && recorder_.recording()) stopRecording(index == 1);
  });
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
  live_root->addLayout(controls);
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
  live_root->addLayout(center, 1);
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
  live_root->addWidget(capture_group);
  connect(choose_session, &QPushButton::clicked, this, &MonoCameraPage::chooseSession);
  connect(capture, &QPushButton::clicked, this, &MonoCameraPage::captureImage);
  capture->setObjectName("captureButton");
  auto* recording_row = new QHBoxLayout;
  recording_rate_ = new QDoubleSpinBox;
  recording_rate_->setRange(0.2, 10.0);
  recording_rate_->setValue(3.0);
  recording_rate_->setSuffix(" FPS");
  record_button_ = new QPushButton("Start recording");
  recording_status_ = new QLabel("Not recording");
  recording_row->addWidget(new QLabel("Recording rate:"));
  recording_row->addWidget(recording_rate_);
  recording_row->addWidget(record_button_);
  recording_row->addWidget(recording_status_, 1);
  capture_layout->addLayout(recording_row);
  connect(record_button_, &QPushButton::clicked, this, &MonoCameraPage::toggleRecording);

  auto* recording_select = new QHBoxLayout;
  recording_path_ = new QLineEdit;
  recording_path_->setReadOnly(true);
  auto* browse_recording = new QPushButton("Open recording");
  auto* reload_recording = new QPushButton("Reload");
  recording_select->addWidget(new QLabel("Recording:"));
  recording_select->addWidget(recording_path_, 1);
  recording_select->addWidget(browse_recording);
  recording_select->addWidget(reload_recording);
  review_root->addLayout(recording_select);
  review_list_ = new QListWidget;
  configureReviewList(review_list_);
  auto* review_content = new QHBoxLayout;
  review_content->addWidget(review_list_, 2);
  review_preview_ = new PreviewLabel;
  review_preview_->setText("Select a frame to view it");
  review_content->addWidget(review_preview_, 1);
  review_root->addLayout(review_content, 1);
  review_summary_ = new QLabel("Load a recording, then analyze quality and pose diversity.");
  review_summary_->setWordWrap(true);
  review_root->addWidget(review_summary_);
  auto* review_actions = new QHBoxLayout;
  auto* select_all = new QPushButton("Select all");
  auto* select_none = new QPushButton("Select none");
  target_frames_ = new QSpinBox;
  target_frames_->setRange(8, 100);
  target_frames_->setValue(30);
  target_frames_->setSuffix(" frames");
  analyze_review_button_ = new QPushButton("Analyze and recommend");
  auto* import = new QPushButton("Add selected frames to dataset");
  review_actions->addWidget(select_all);
  review_actions->addWidget(select_none);
  review_actions->addWidget(new QLabel("Target:"));
  review_actions->addWidget(target_frames_);
  review_actions->addWidget(analyze_review_button_);
  review_actions->addStretch();
  review_actions->addWidget(import);
  review_root->addLayout(review_actions);
  connect(browse_recording, &QPushButton::clicked, this, &MonoCameraPage::chooseRecording);
  connect(reload_recording, &QPushButton::clicked, this, &MonoCameraPage::loadRecording);
  connect(review_list_, &QListWidget::currentItemChanged, this,
          [this](QListWidgetItem* current, QListWidgetItem*) {
    if (current == nullptr) {
      review_preview_->clear();
      review_preview_->setText("Select a frame to view it");
      return;
    }
    showReviewImage(review_preview_, current->data(Qt::UserRole).toString(), true);
  });
  connect(select_all, &QPushButton::clicked, this, [this] { setReviewSelection(true); });
  connect(select_none, &QPushButton::clicked, this, [this] { setReviewSelection(false); });
  connect(analyze_review_button_, &QPushButton::clicked,
          this, &MonoCameraPage::analyzeReviewFrames);
  connect(import, &QPushButton::clicked, this, &MonoCameraPage::importSelectedFrames);

  auto* calibration_group = new QGroupBox("7 x 7 circles mono calibration (30 mm spacing)");
  auto* form = new QFormLayout(calibration_group);
  form->addRow("Images:", pathRow(images_, "Browse", [this] { chooseImages(); }));
  camera_name_ = new QLineEdit("camera");
  form->addRow("Camera name:", camera_name_);
  form->addRow("Output YAML:", pathRow(output_, "Browse", [this] { chooseOutput(); }));
  calibrate_button_ = new QPushButton("Calibrate selected camera");
  form->addRow(calibrate_button_);
  calibration_root->addWidget(calibration_group);

  calibration_details_ = new QTabWidget;
  auto* calibration_input_page = new QWidget;
  auto* calibration_input_root = new QVBoxLayout(calibration_input_page);
  calibration_selection_summary_ = new QLabel("Choose an image directory to load candidates.");
  calibration_input_root->addWidget(calibration_selection_summary_);
  auto* calibration_input_content = new QHBoxLayout;
  calibration_images_list_ = new QListWidget;
  configureReviewList(calibration_images_list_);
  calibration_input_content->addWidget(calibration_images_list_, 2);
  calibration_image_preview_ = new PreviewLabel;
  calibration_image_preview_->setText("Select a calibration image to view it");
  calibration_input_content->addWidget(calibration_image_preview_, 1);
  calibration_input_root->addLayout(calibration_input_content, 1);
  auto* calibration_input_actions = new QHBoxLayout;
  auto* select_all_calibration = new QPushButton("Select all");
  auto* select_no_calibration = new QPushButton("Select none");
  calibration_input_actions->addWidget(select_all_calibration);
  calibration_input_actions->addWidget(select_no_calibration);
  calibration_input_actions->addStretch();
  calibration_input_root->addLayout(calibration_input_actions);

  auto* calibration_result_page = new QWidget;
  auto* calibration_result_root = new QVBoxLayout(calibration_result_page);
  calibration_result_summary_ = new QLabel("Run calibration to view the result.");
  calibration_result_summary_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  calibration_result_summary_->setWordWrap(true);
  calibration_result_root->addWidget(calibration_result_summary_);
  auto* calibration_result_content = new QHBoxLayout;
  calibration_result_list_ = new QListWidget;
  configureReviewList(calibration_result_list_);
  calibration_result_content->addWidget(calibration_result_list_, 2);
  calibration_result_preview_ = new PreviewLabel;
  calibration_result_preview_->setText("Select a result row to view it");
  calibration_result_content->addWidget(calibration_result_preview_, 1);
  calibration_result_root->addLayout(calibration_result_content, 1);
  show_undistorted_ = new QCheckBox("Show undistorted preview");
  show_undistorted_->setEnabled(false);
  calibration_result_root->addWidget(show_undistorted_);

  calibration_details_->addTab(calibration_input_page, "Input selection");
  calibration_details_->addTab(calibration_result_page, "Calibration result");
  calibration_root->addWidget(calibration_details_, 1);
  connect(calibrate_button_, &QPushButton::clicked, this, &MonoCameraPage::calibrate);
  connect(images_, &QLineEdit::editingFinished, this, [this] {
    loadCalibrationImages(images_->text());
  });
  connect(select_all_calibration, &QPushButton::clicked, this,
          [this] { setCalibrationSelection(true); });
  connect(select_no_calibration, &QPushButton::clicked, this,
          [this] { setCalibrationSelection(false); });
  connect(calibration_images_list_, &QListWidget::itemChanged, this,
          [this](QListWidgetItem*) { updateCalibrationSelectionSummary(); });
  connect(calibration_images_list_, &QListWidget::currentItemChanged, this,
          [this](QListWidgetItem* current, QListWidgetItem*) {
    if (current == nullptr) return;
    showReviewImage(calibration_image_preview_, current->data(Qt::UserRole).toString(), true);
  });
  connect(calibration_result_list_, &QListWidget::currentItemChanged, this,
          [this](QListWidgetItem*, QListWidgetItem*) { updateCalibrationResultPreview(); });
  connect(show_undistorted_, &QCheckBox::toggled, this,
          [this](bool) { updateCalibrationResultPreview(); });

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
  stopRecording(false);
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
    stages_->setCurrentIndex(0);
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
  if (recorder_.recording()) {
    recorder_.submitMono(frame);
    const auto stats = recorder_.stats();
    recording_status_->setText(
        QString("Written %1  queued %2  dropped %3")
            .arg(stats.written).arg(stats.pending).arg(stats.dropped));
  }
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

void MonoCameraPage::toggleRecording() {
  if (recorder_.recording()) {
    stopRecording(true);
    return;
  }
  if (!camera_->running()) return status("Start the camera before recording", true);
  const auto directory = std::filesystem::path(session_->text().toStdString()) / "recordings" /
                         recordingName("mono").toStdString();
  std::string error;
  if (!recorder_.startMono(directory, camera_->serialNumber(), recording_rate_->value(), error)) {
    return status(QString::fromStdString(error), true);
  }
  recording_path_->setText(QString::fromStdString(directory.string()));
  record_button_->setText("Stop recording");
  recording_rate_->setEnabled(false);
  recording_status_->setText("Recording...");
  status("Recording lossless PNG frames in the background");
}

void MonoCameraPage::stopRecording(const bool open_review) {
  if (!recorder_.recording()) return;
  const auto directory = recorder_.directory();
  recorder_.stop();
  record_button_->setText("Start recording");
  recording_rate_->setEnabled(true);
  const auto stats = recorder_.stats();
  recording_status_->setText(
      QString("Stopped: %1 frames, %2 dropped").arg(stats.written).arg(stats.dropped));
  recording_path_->setText(QString::fromStdString(directory.string()));
  if (open_review) {
    loadRecording();
    stages_->setCurrentIndex(1);
  }
}

void MonoCameraPage::chooseRecording() {
  const QString path = QFileDialog::getExistingDirectory(
      this, "Choose mono recording", recording_path_->text());
  if (!path.isEmpty()) {
    recording_path_->setText(path);
    loadRecording();
  }
}

void MonoCameraPage::loadRecording() {
  ++review_generation_;
  review_list_->clear();
  const auto frames = listImages(
      std::filesystem::path(recording_path_->text().toStdString()) / "frames");
  for (const auto& path : frames) {
    auto* item = new QListWidgetItem(QString::fromStdString(path.filename().string()),
                                     review_list_);
    item->setData(Qt::UserRole, QString::fromStdString(path.string()));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Checked);
  }
  if (!frames.empty()) review_list_->setCurrentRow(0);
  analyze_review_button_->setEnabled(!frames.empty() && !review_analysis_running_);
  target_frames_->setEnabled(!review_analysis_running_);
  review_summary_->setText(
      QString("Loaded %1 frame(s). Analysis has not been run; all frames are selected.")
          .arg(frames.size()));
  status(QString("Loaded %1 recorded frame(s)").arg(frames.size()));
}

void MonoCameraPage::analyzeReviewFrames() {
  if (review_analysis_running_) return;
  std::vector<std::filesystem::path> paths;
  paths.reserve(static_cast<std::size_t>(review_list_->count()));
  for (int index = 0; index < review_list_->count(); ++index) {
    paths.emplace_back(review_list_->item(index)->data(Qt::UserRole).toString().toStdString());
  }
  if (paths.empty()) return status("Load a recording before analysis", true);

  const std::size_t generation = review_generation_;
  MonoReviewOptions options;
  options.target_frame_count = static_cast<std::size_t>(target_frames_->value());
  analyze_review_button_->setEnabled(false);
  target_frames_->setEnabled(false);
  review_analysis_running_ = true;
  review_summary_->setText(QString("Analyzing %1 frame(s) in the background...")
                               .arg(paths.size()));
  status("Detecting circle grids and comparing pose diversity...");
  QPointer<MonoCameraPage> self(this);
  auto* thread = QThread::create([self, paths = std::move(paths), options, generation] {
    const auto progress = [self, generation](const std::string& stage,
                                             const std::size_t current,
                                             const std::size_t total) {
      if (self) QMetaObject::invokeMethod(self, [self, generation, stage, current, total] {
        if (!self || generation != self->review_generation_) return;
        const QString message = current == 0
                                    ? QString::fromStdString(stage)
                                    : QString("%1: %2 / %3")
                                          .arg(QString::fromStdString(stage))
                                          .arg(current)
                                          .arg(total);
        self->review_summary_->setText(message);
        self->status(message);
      });
    };
    const auto results = analyzeMonoFrames(paths, options, progress);
    if (self) QMetaObject::invokeMethod(self, [self, results, generation] {
      if (self) self->applyReviewAnalysis(results, generation);
    });
  });
  connect(thread, &QThread::finished, thread, &QObject::deleteLater);
  thread->start();
}

void MonoCameraPage::applyReviewAnalysis(const std::vector<MonoFrameAnalysis>& results,
                                         const std::size_t generation) {
  review_analysis_running_ = false;
  analyze_review_button_->setEnabled(review_list_->count() > 0);
  target_frames_->setEnabled(true);
  if (generation != review_generation_) return;
  std::size_t kept = 0;
  std::size_t duplicates = 0;
  std::size_t redundant = 0;
  std::size_t failed = 0;
  std::size_t warnings = 0;
  std::size_t poses = 0;
  const std::size_t count =
      std::min(results.size(), static_cast<std::size_t>(review_list_->count()));
  for (std::size_t index = 0; index < count; ++index) {
    const auto& result = results[index];
    if (result.pose_estimated) ++poses;
    auto* item = review_list_->item(static_cast<int>(index));
    item->setText(monoAnalysisText(result));
    item->setToolTip(item->text());
    item->setCheckState(result.recommended ? Qt::Checked : Qt::Unchecked);
    if (result.recommended) {
      ++kept;
      if (result.low_sharpness || result.exposure_warning) {
        ++warnings;
        item->setForeground(QBrush(QColor(170, 110, 0)));
      } else {
        item->setForeground(QBrush(QColor(0, 125, 55)));
      }
    } else if (result.decision == MonoFrameDecision::duplicate) {
      ++duplicates;
      item->setForeground(QBrush(QColor(115, 115, 115)));
    } else if (result.decision == MonoFrameDecision::redundant) {
      ++redundant;
      item->setForeground(QBrush(QColor(90, 90, 140)));
    } else {
      ++failed;
      item->setForeground(QBrush(QColor(185, 40, 40)));
    }
  }
  review_summary_->setText(
      QString("Recommended %1 of %2: %3 duplicate, %4 redundant beyond target, "
              "%5 invalid, %6 warning, %7 poses estimated. "
              "The checkboxes are editable; no source files were deleted.")
          .arg(kept)
          .arg(results.size())
          .arg(duplicates)
          .arg(redundant)
          .arg(failed)
          .arg(warnings)
          .arg(poses));
  status(QString("Analysis complete: %1 recommended frame(s)").arg(kept), kept < 8);
}

void MonoCameraPage::setReviewSelection(const bool selected) {
  for (int index = 0; index < review_list_->count(); ++index) {
    review_list_->item(index)->setCheckState(selected ? Qt::Checked : Qt::Unchecked);
  }
}

void MonoCameraPage::importSelectedFrames() {
  const QString label = device_->currentData().toString().isEmpty()
                            ? QString("camera") : device_->currentData().toString();
  DatasetWriter writer(session_->text().toStdString());
  std::size_t imported = 0;
  std::filesystem::path last_path;
  for (int index = 0; index < review_list_->count(); ++index) {
    const auto* item = review_list_->item(index);
    if (item->checkState() != Qt::Checked) continue;
    const cv::Mat image = cv::imread(item->data(Qt::UserRole).toString().toStdString(),
                                     cv::IMREAD_UNCHANGED);
    std::string error;
    if (!writer.saveMono(label.toStdString(), image, last_path, error)) {
      return status(QString::fromStdString(error), true);
    }
    ++imported;
  }
  if (imported == 0) return status("Select at least one frame", true);
  images_->setText(QString::fromStdString(last_path.parent_path().string()));
  loadCalibrationImages(images_->text());
  status(QString("Added %1 frame(s) to the mono dataset").arg(imported));
}
void MonoCameraPage::resetFocus() {
  focus_analyzer_.reset();
  status("Focus peak reset");
}

void MonoCameraPage::chooseImages() {
  const QString path = QFileDialog::getExistingDirectory(this, "Choose mono images");
  if (!path.isEmpty()) {
    images_->setText(path);
    loadCalibrationImages(path);
  }
}

void MonoCameraPage::loadCalibrationImages(const QString& directory) {
  const auto paths = listImages(directory.toStdString());
  {
    const QSignalBlocker blocker(calibration_images_list_);
    calibration_images_list_->clear();
    for (const auto& path : paths) {
      auto* item = new QListWidgetItem(QString::fromStdString(path.filename().string()),
                                       calibration_images_list_);
      item->setData(Qt::UserRole, QString::fromStdString(path.string()));
      item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
      item->setCheckState(Qt::Checked);
    }
  }
  if (!paths.empty()) calibration_images_list_->setCurrentRow(0);
  updateCalibrationSelectionSummary();
  calibration_details_->setCurrentIndex(0);
}

void MonoCameraPage::setCalibrationSelection(const bool selected) {
  const QSignalBlocker blocker(calibration_images_list_);
  for (int index = 0; index < calibration_images_list_->count(); ++index) {
    calibration_images_list_->item(index)->setCheckState(selected ? Qt::Checked : Qt::Unchecked);
  }
  updateCalibrationSelectionSummary();
}

void MonoCameraPage::updateCalibrationSelectionSummary() {
  int selected = 0;
  for (int index = 0; index < calibration_images_list_->count(); ++index) {
    if (calibration_images_list_->item(index)->checkState() == Qt::Checked) ++selected;
  }
  calibration_selection_summary_->setText(
      QString("Selected %1 of %2 image(s). Only checked images enter calibration; at least 8 "
              "valid circle-grid images are required.")
          .arg(selected)
          .arg(calibration_images_list_->count()));
}

void MonoCameraPage::applyCalibrationResult(const Intrinsics& result) {
  last_intrinsics_ = result;
  const cv::Mat& matrix = result.camera_matrix;
  QStringList distortion;
  const cv::Mat flattened = result.distortion.reshape(1, 1);
  for (int column = 0; column < flattened.cols; ++column) {
    distortion << QString::number(flattened.at<double>(0, column), 'g', 10);
  }
  calibration_result_summary_->setText(
      QString("RMS: %1 px    Image: %2 x %3    Used: %4    Rejected: %5\n"
              "fx=%6  fy=%7  cx=%8  cy=%9\nDistortion: [%10]")
          .arg(result.rms_px, 0, 'f', 4)
          .arg(result.image_size.width)
          .arg(result.image_size.height)
          .arg(result.used_images.size())
          .arg(result.rejected_images.size())
          .arg(matrix.at<double>(0, 0), 0, 'g', 10)
          .arg(matrix.at<double>(1, 1), 0, 'g', 10)
          .arg(matrix.at<double>(0, 2), 0, 'g', 10)
          .arg(matrix.at<double>(1, 2), 0, 'g', 10)
          .arg(distortion.join(", ")));

  calibration_result_list_->clear();
  for (std::size_t index = 0; index < result.used_images.size(); ++index) {
    const double error = index < result.per_view_error_px.size()
                             ? result.per_view_error_px[index]
                             : std::numeric_limits<double>::quiet_NaN();
    auto* item = new QListWidgetItem(
        QString("%1  [USED]  error %2 px")
            .arg(QString::fromStdString(result.used_images[index].filename().string()))
            .arg(error, 0, 'f', 3),
        calibration_result_list_);
    item->setData(Qt::UserRole, QString::fromStdString(result.used_images[index].string()));
    item->setForeground(QBrush(QColor(0, 125, 55)));
  }
  for (const auto& path : result.rejected_images) {
    auto* item = new QListWidgetItem(
        QString("%1  [REJECTED]").arg(QString::fromStdString(path.filename().string())),
        calibration_result_list_);
    item->setData(Qt::UserRole, QString::fromStdString(path.string()));
    item->setForeground(QBrush(QColor(185, 40, 40)));
  }
  show_undistorted_->setEnabled(result.valid());
  if (calibration_result_list_->count() > 0) calibration_result_list_->setCurrentRow(0);
  calibration_details_->setCurrentIndex(1);
}

void MonoCameraPage::updateCalibrationResultPreview() {
  const auto* item = calibration_result_list_->currentItem();
  if (item == nullptr) return;
  const QString path = item->data(Qt::UserRole).toString();
  if (!show_undistorted_->isChecked() || !last_intrinsics_.valid()) {
    showReviewImage(calibration_result_preview_, path, true);
    return;
  }
  const cv::Mat gray = cv::imread(path.toStdString(), cv::IMREAD_GRAYSCALE);
  if (gray.empty()) return;
  cv::Mat corrected;
  cv::undistort(gray, corrected, last_intrinsics_.camera_matrix,
                last_intrinsics_.distortion);
  calibration_result_preview_->showFrame(corrected);
}

void MonoCameraPage::chooseOutput() {
  const QString path = QFileDialog::getSaveFileName(this, "Save intrinsics", {}, "YAML (*.yaml)");
  if (!path.isEmpty()) output_->setText(path);
}

void MonoCameraPage::calibrate() {
  if (images_->text().isEmpty() || output_->text().isEmpty()) {
    return status("Choose images and output YAML", true);
  }
  if (calibration_images_list_->count() == 0) loadCalibrationImages(images_->text());
  std::vector<std::filesystem::path> selected_images;
  for (int index = 0; index < calibration_images_list_->count(); ++index) {
    const auto* item = calibration_images_list_->item(index);
    if (item->checkState() == Qt::Checked) {
      selected_images.emplace_back(item->data(Qt::UserRole).toString().toStdString());
    }
  }
  if (selected_images.size() < 8) {
    return status("Select at least 8 calibration images", true);
  }
  calibrate_button_->setEnabled(false);
  status("Running mono calibration...");
  const QString output = output_->text();
  const QString name = camera_name_->text();
  QPointer<MonoCameraPage> self(this);
  auto* thread = QThread::create([self, selected_images = std::move(selected_images),
                                  output, name] {
    Intrinsics result;
    std::string error;
    bool ok = calibrateMono(selected_images, {}, result, error);
    if (ok) ok = saveIntrinsics(output.toStdString(), name.toStdString(), result, error);
    const QString message = ok
        ? QString("Saved intrinsics: RMS=%1 px, used=%2, rejected=%3")
              .arg(result.rms_px, 0, 'f', 4).arg(result.used_images.size())
              .arg(result.rejected_images.size())
        : QString::fromStdString(error);
    if (self) QMetaObject::invokeMethod(self, [self, ok, message, result] {
      if (!self) return;
      self->calibrate_button_->setEnabled(true);
      if (ok) self->applyCalibrationResult(result);
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
  stages_ = new QTabWidget;
  auto* live_page = new QWidget;
  auto* live_root = new QVBoxLayout(live_page);
  auto* review_page = new QWidget;
  auto* review_root = new QVBoxLayout(review_page);
  auto* calibration_page = new QWidget;
  auto* calibration_root = new QVBoxLayout(calibration_page);
  stages_->addTab(live_page, "1  Live capture");
  stages_->addTab(review_page, "2  Review pairs");
  stages_->addTab(calibration_page, "3  Calibration");
  root->addWidget(stages_, 1);
  connect(stages_, &QTabWidget::currentChanged, this, [this](const int index) {
    if (index != 0 && recorder_.recording()) stopRecording(index == 1);
  });
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
  live_root->addLayout(controls);
  connect(refresh, &QPushButton::clicked, this, &StereoCameraPage::refreshDevices);
  connect(start_stop_, &QPushButton::clicked, this, &StereoCameraPage::toggleCameras);

  auto* previews = new QHBoxLayout;
  left_preview_ = new PreviewLabel;
  right_preview_ = new PreviewLabel;
  previews->addWidget(left_preview_, 1);
  previews->addWidget(right_preview_, 1);
  live_root->addLayout(previews, 1);
  pair_status_ = new QLabel("Waiting for stereo frames");
  pair_status_->setAlignment(Qt::AlignCenter);
  live_root->addWidget(pair_status_);

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
  live_root->addWidget(capture_group);
  connect(choose_session, &QPushButton::clicked, this, &StereoCameraPage::chooseSession);
  connect(capture, &QPushButton::clicked, this, &StereoCameraPage::capturePair);
  auto* recording_row = new QHBoxLayout;
  recording_rate_ = new QDoubleSpinBox;
  recording_rate_->setRange(0.2, 10.0);
  recording_rate_->setValue(3.0);
  recording_rate_->setSuffix(" FPS");
  record_button_ = new QPushButton("Start stereo recording");
  recording_status_ = new QLabel("Not recording");
  recording_row->addWidget(new QLabel("Recording rate:"));
  recording_row->addWidget(recording_rate_);
  recording_row->addWidget(record_button_);
  recording_row->addWidget(recording_status_, 1);
  capture_layout->addLayout(recording_row);
  connect(record_button_, &QPushButton::clicked, this, &StereoCameraPage::toggleRecording);

  auto* recording_select = new QHBoxLayout;
  recording_path_ = new QLineEdit;
  recording_path_->setReadOnly(true);
  auto* browse_recording = new QPushButton("Open recording");
  auto* reload_recording = new QPushButton("Reload");
  recording_select->addWidget(new QLabel("Recording:"));
  recording_select->addWidget(recording_path_, 1);
  recording_select->addWidget(browse_recording);
  recording_select->addWidget(reload_recording);
  review_root->addLayout(recording_select);
  review_list_ = new QListWidget;
  configureReviewList(review_list_);
  auto* review_content = new QHBoxLayout;
  review_content->addWidget(review_list_, 2);
  auto* preview_column = new QVBoxLayout;
  preview_column->addWidget(new QLabel("Left frame"));
  left_review_preview_ = new PreviewLabel;
  left_review_preview_->setMinimumSize(320, 220);
  left_review_preview_->setText("Select a pair to view it");
  preview_column->addWidget(left_review_preview_, 1);
  preview_column->addWidget(new QLabel("Right frame"));
  right_review_preview_ = new PreviewLabel;
  right_review_preview_->setMinimumSize(320, 220);
  right_review_preview_->setText("Select a pair to view it");
  preview_column->addWidget(right_review_preview_, 1);
  review_content->addLayout(preview_column, 1);
  review_root->addLayout(review_content, 1);
  auto* review_actions = new QHBoxLayout;
  auto* select_all = new QPushButton("Select all");
  auto* select_none = new QPushButton("Select none");
  auto* import = new QPushButton("Add selected pairs to dataset");
  review_actions->addWidget(select_all);
  review_actions->addWidget(select_none);
  review_actions->addStretch();
  review_actions->addWidget(import);
  review_root->addLayout(review_actions);
  connect(browse_recording, &QPushButton::clicked, this, &StereoCameraPage::chooseRecording);
  connect(reload_recording, &QPushButton::clicked, this, &StereoCameraPage::loadRecording);
  connect(review_list_, &QListWidget::currentItemChanged, this,
          [this](QListWidgetItem* current, QListWidgetItem*) {
    if (current == nullptr) {
      left_review_preview_->clear();
      left_review_preview_->setText("Select a pair to view it");
      right_review_preview_->clear();
      right_review_preview_->setText("Select a pair to view it");
      return;
    }
    showReviewImage(left_review_preview_, current->data(Qt::UserRole).toString(), false);
    showReviewImage(right_review_preview_, current->data(Qt::UserRole + 1).toString(), false);
  });
  connect(select_all, &QPushButton::clicked, this, [this] { setReviewSelection(true); });
  connect(select_none, &QPushButton::clicked, this, [this] { setReviewSelection(false); });
  connect(import, &QPushButton::clicked, this, &StereoCameraPage::importSelectedPairs);

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
  calibration_root->addWidget(calibration_group);
  calibration_root->addStretch();
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
    if (recorder_.recording() && delta_ms <= 150.0) {
      recorder_.submitStereo(left, right);
      const auto stats = recorder_.stats();
      recording_status_->setText(
          QString("Written %1 pairs  queued %2  dropped %3")
              .arg(stats.written).arg(stats.pending).arg(stats.dropped));
    }
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

void StereoCameraPage::toggleRecording() {
  if (recorder_.recording()) {
    stopRecording(true);
    return;
  }
  if (!left_camera_->running() || !right_camera_->running()) {
    return status("Start both cameras before recording", true);
  }
  const auto directory = std::filesystem::path(session_->text().toStdString()) / "recordings" /
                         recordingName("stereo").toStdString();
  std::string error;
  if (!recorder_.startStereo(directory, left_camera_->serialNumber(),
                             right_camera_->serialNumber(), recording_rate_->value(), error)) {
    return status(QString::fromStdString(error), true);
  }
  recording_path_->setText(QString::fromStdString(directory.string()));
  record_button_->setText("Stop stereo recording");
  recording_rate_->setEnabled(false);
  recording_status_->setText("Recording synchronized pairs...");
  status("Recording lossless stereo PNG pairs in the background");
}

void StereoCameraPage::stopRecording(const bool open_review) {
  if (!recorder_.recording()) return;
  const auto directory = recorder_.directory();
  recorder_.stop();
  record_button_->setText("Start stereo recording");
  recording_rate_->setEnabled(true);
  const auto stats = recorder_.stats();
  recording_status_->setText(
      QString("Stopped: %1 pairs, %2 dropped").arg(stats.written).arg(stats.dropped));
  recording_path_->setText(QString::fromStdString(directory.string()));
  if (open_review) {
    loadRecording();
    stages_->setCurrentIndex(1);
  }
}

void StereoCameraPage::chooseRecording() {
  const QString path = QFileDialog::getExistingDirectory(
      this, "Choose stereo recording", recording_path_->text());
  if (!path.isEmpty()) {
    recording_path_->setText(path);
    loadRecording();
  }
}

void StereoCameraPage::loadRecording() {
  review_list_->clear();
  const std::filesystem::path root(recording_path_->text().toStdString());
  const auto left_frames = listImages(root / "left");
  for (const auto& left : left_frames) {
    const auto right = root / "right" / left.filename();
    if (!std::filesystem::exists(right)) continue;
    auto* item = new QListWidgetItem(QString::fromStdString(left.filename().string()),
                                     review_list_);
    item->setData(Qt::UserRole, QString::fromStdString(left.string()));
    item->setData(Qt::UserRole + 1, QString::fromStdString(right.string()));
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Checked);
  }
  if (review_list_->count() > 0) review_list_->setCurrentRow(0);
  status(QString("Loaded %1 recorded stereo pair(s)").arg(review_list_->count()));
}

void StereoCameraPage::setReviewSelection(const bool selected) {
  for (int index = 0; index < review_list_->count(); ++index) {
    review_list_->item(index)->setCheckState(selected ? Qt::Checked : Qt::Unchecked);
  }
}

void StereoCameraPage::importSelectedPairs() {
  DatasetWriter writer(session_->text().toStdString());
  std::size_t imported = 0;
  std::filesystem::path last_left;
  std::filesystem::path last_right;
  for (int index = 0; index < review_list_->count(); ++index) {
    const auto* item = review_list_->item(index);
    if (item->checkState() != Qt::Checked) continue;
    const cv::Mat left = cv::imread(item->data(Qt::UserRole).toString().toStdString(),
                                    cv::IMREAD_UNCHANGED);
    const cv::Mat right = cv::imread(item->data(Qt::UserRole + 1).toString().toStdString(),
                                     cv::IMREAD_UNCHANGED);
    std::string error;
    if (!writer.saveStereoPair(left, right, last_left, last_right, error)) {
      return status(QString::fromStdString(error), true);
    }
    ++imported;
  }
  if (imported == 0) return status("Select at least one stereo pair", true);
  left_images_->setText(QString::fromStdString(last_left.parent_path().string()));
  right_images_->setText(QString::fromStdString(last_right.parent_path().string()));
  status(QString("Added %1 pair(s) to the stereo dataset").arg(imported));
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
