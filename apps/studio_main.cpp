#include "camera_calibration_studio/main_window.hpp"

#include <QApplication>

int main(int argc, char** argv) {
  QApplication application(argc, argv);
  QApplication::setApplicationName("Camera Calibration Studio");
  QApplication::setOrganizationName("PNX");
  calibration_studio::MainWindow window;
  window.show();
  return application.exec();
}

