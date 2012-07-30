rosbuild_add_executable(photo_node photo_node.cpp)
target_link_libraries(photo_node photo2)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GPHOTO2 REQUIRED libgphoto2)
include_directories(${GPHOTO2_INCLUDE_DIRS})
link_directories(${GPHOTO2_LIBRARY_DIRS})
target_link_libraries(photo_node ${GPHOTO2_LIBRARIES})
