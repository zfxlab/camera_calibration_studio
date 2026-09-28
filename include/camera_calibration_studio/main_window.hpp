#pragma once

#include <QMainWindow>

class QTabWidget;

namespace calibration_studio {

class MonoCameraPage;
class StereoCameraPage;

class MainWindow final : public QMainWindow {
  Q_OBJECT
 public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

 private slots:
  void pageChanged(int index);

 private:
  QTabWidget* tabs_{nullptr};
  MonoCameraPage* mono_page_{nullptr};
  StereoCameraPage* stereo_page_{nullptr};
  int previous_page_{0};
};

}  // namespace calibration_studio

