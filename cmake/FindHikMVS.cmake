find_path(HIK_MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS /opt/MVS/include /usr/local/include
)

find_library(HIK_MVS_LIBRARY
  NAMES MvCameraControl
  PATHS /opt/MVS/lib/64 /opt/MVS/lib /usr/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(HikMVS
  REQUIRED_VARS HIK_MVS_INCLUDE_DIR HIK_MVS_LIBRARY)

if(HikMVS_FOUND AND NOT TARGET HikMVS::MvCameraControl)
  add_library(HikMVS::MvCameraControl UNKNOWN IMPORTED)
  set_target_properties(HikMVS::MvCameraControl PROPERTIES
    IMPORTED_LOCATION "${HIK_MVS_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${HIK_MVS_INCLUDE_DIR}")
endif()

mark_as_advanced(HIK_MVS_INCLUDE_DIR HIK_MVS_LIBRARY)

