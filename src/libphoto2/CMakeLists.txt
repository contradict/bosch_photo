rosbuild_add_library(photo2 photo_reporter.cpp photo_camera_list.cpp photo_camera.cpp photo_image.cpp)
target_link_libraries(photo2)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GPHOTO2 REQUIRED libgphoto2)
include_directories(${GPHOTO2_INCLUDE_DIRS})
link_directories(${GPHOTO2_LIBRARY_DIRS})
target_link_libraries(photo2 ${GPHOTO2_LIBRARIES})
