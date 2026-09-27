# Camera Calibration Studio

不依赖 ROS 2 的海康 MVS 相机标定桌面工具。程序直接通过 MVS SDK 获取图像，使用 Qt 5
显示和管理采集过程，并使用 OpenCV 完成单目标定、双目外参和立体校正参数计算。

## 当前功能

- 枚举并按序列号打开一台或两台 MVS USB/GigE 相机；
- 设置固定曝光、增益、帧率和 SDK 缓冲数量；
- 实时灰度预览；
- 保存单目无损 PNG 或原子保存左右 PNG 图像对；
- 使用 `7 x 7` 对称圆点板和 `0.03 m` 圆心距计算单目内参；
- 使用 `DICT_6X6_50`、ID `[1,2,3,4]` 的四 ArUco 板计算双目外参；
- 双目标定固定左右内参，并输出 `R/T/E/F/R1/R2/P1/P2/Q`；
- 同时提供 GUI 和离线命令行标定程序。

默认四 ArUco 板参数为：Marker 边长 `0.20 m`，横向中心距 `1.10 m`，纵向中心距
`0.70 m`。这些数值必须与实物一致，否则平移和基线尺度会按比例出错。

## 依赖与构建

- CMake 3.16+、C++17
- Qt 5 Widgets
- OpenCV 4（包含 `calib3d` 和 `aruco`）
- 海康 MVS SDK，默认安装在 `/opt/MVS`

```bash
sudo apt install cmake g++ qtbase5-dev libopencv-dev

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

如果运行时找不到 MVS 动态库：

```bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64:${LD_LIBRARY_PATH}
```

## GUI

```bash
./build/camera_calibration_studio
```

### 图片采集

1. 点击 `Refresh` 枚举相机；
2. 选择左/单目相机，双目时再选择右相机；
3. 设置曝光、增益和帧率；
4. 点击 `Start cameras`；
5. 选择会话目录；
6. 使用单目或双目拍照按钮保存 PNG。

目录结构：

```text
session/
├── mono/
│   └── SERIAL/
│       ├── 000001.png
│       └── 000002.png
└── stereo/
    ├── left/
    │   └── 000001.png
    └── right/
        └── 000001.png
```

双目保存要求两张最新图像均不超过 `0.5 s`，接收时刻差不超过 `0.15 s`。这适用于静止
标定板；运动场景需要共同硬件触发。

### 单目标定

选择某台相机的 `mono/SERIAL` 目录及输出 YAML。程序只使用成功检测到全部 49 个圆心且
分辨率一致的图片。最低要求 8 张，建议采集 20–30 张，并覆盖画面中心、四角、远近和多种
倾角。输出采用 ROS `camera_info_manager` 字段结构，同时包含基本标定指标。

### 双目标定

先分别完成左右单目标定，然后选择左右图片目录、左右内参 YAML 和输出文件。左右图片通过
相同文件名配对。每对至少需要三个共同 ArUco Marker，最低要求 5 个有效姿态，建议采集
20–40 个多样姿态。

## 命令行离线计算

单目：

```bash
./build/calibrate_dataset --mono \
  --images sessions/example/mono/00DA1923275 \
  --output results/00DA1923275.yaml \
  --name camera
```

双目：

```bash
./build/calibrate_dataset --stereo \
  --left-images sessions/example/stereo/left \
  --right-images sessions/example/stereo/right \
  --left-intrinsics results/left.yaml \
  --right-intrinsics results/right.yaml \
  --output results/stereo.yaml
```

## 结果约定

```text
X_right = R_right_left * X_left + t_right_left
```

平移单位为米。单目和双目图像必须保持相同分辨率、ROI、镜头焦距与对焦状态。

## 已知边界

- 当前使用普通针孔 `plumb_bob` 模型，尚未提供鱼眼模型界面；
- 当前拍照使用自由运行相机的最新帧，不是硬件同步触发；
- 当前 GUI 展示采集文件列表，但尚未提供缩略图级逐张启用/禁用；
- 当前双目板参数使用默认值，配置文件将在后续版本接入 GUI；
- 正式使用前应通过校正预览和独立验证图片检查极线误差。

