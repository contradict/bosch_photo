rosbuild_add_library(photo photo_capture.cpp photo_config.cpp photo_image.cpp photo.cpp)
target_link_libraries(photo)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GPHOTO2 REQUIRED libgphoto2)
include_directories(${GPHOTO2_INCLUDE_DIRS})
link_directories(${GPHOTO2_LIBRARY_DIRS})
target_link_libraries(photo ${GPHOTO2_LIBRARIES})
