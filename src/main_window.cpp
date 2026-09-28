#include "camera_calibration_studio/main_window.hpp"

#include "camera_calibration_studio/workflow_pages.hpp"

#include <QTabWidget>

namespace calibration_studio {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle("Camera Calibration Studio");
  resize(1500, 980);
  tabs_ = new QTabWidget;
  mono_page_ = new MonoCameraPage;
  stereo_page_ = new StereoCameraPage;
  tabs_->addTab(mono_page_, "Single camera: focus and calibration");
  tabs_->addTab(stereo_page_, "Stereo cameras: pair capture and calibration");
  setCentralWidget(tabs_);
  connect(tabs_, &QTabWidget::currentChanged, this, &MainWindow::pageChanged);
}

MainWindow::~MainWindow() {
  mono_page_->deactivate();
  stereo_page_->deactivate();
}

void MainWindow::pageChanged(const int index) {
  if (previous_page_ == 0) {
    mono_page_->deactivate();
  } else {
    stereo_page_->deactivate();
  }
  previous_page_ = index;
}

}  // namespace calibration_studio
