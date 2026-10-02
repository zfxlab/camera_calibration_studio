cmake_minimum_required(VERSION 3.16)
project(camera_calibration_studio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTING "Build unit tests" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d imgproc imgcodecs highgui aruco)
find_package(Qt5 5.12 REQUIRED COMPONENTS Core Gui Widgets)
find_package(HikMVS REQUIRED)

add_library(calibration_core
  src/calibration.cpp
  src/dataset.cpp
  src/focus_analyzer.cpp
  src/review_analysis.cpp
  src/recording.cpp
)
target_include_directories(calibration_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(calibration_core PUBLIC ${OpenCV_LIBS})

add_library(hik_camera_backend src/hik_camera.cpp)
target_include_directories(hik_camera_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(hik_camera_backend PUBLIC HikMVS::MvCameraControl ${OpenCV_LIBS})

add_executable(camera_calibration_studio
  apps/studio_main.cpp
  include/camera_calibration_studio/main_window.hpp
  include/camera_calibration_studio/workflow_pages.hpp
  src/main_window.cpp
  src/workflow_pages.cpp
)
target_include_directories(camera_calibration_studio PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_calibration_studio PRIVATE
  calibration_core hik_camera_backend Qt5::Core Qt5::Gui Qt5::Widgets ${OpenCV_LIBS})

add_executable(calibrate_dataset apps/calibrate_dataset_main.cpp)
target_link_libraries(calibrate_dataset PRIVATE calibration_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(target calibration_core hik_camera_backend camera_calibration_studio calibrate_dataset)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endforeach()
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(calibration_core_test tests/calibration_core_test.cpp)
  target_link_libraries(calibration_core_test PRIVATE calibration_core)
  add_test(NAME calibration_core_test COMMAND calibration_core_test)
  add_executable(review_analysis_test tests/review_analysis_test.cpp)
  target_link_libraries(review_analysis_test PRIVATE calibration_core)
  add_test(NAME review_analysis_test COMMAND review_analysis_test)
endif()

install(TARGETS camera_calibration_studio calibrate_dataset RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
