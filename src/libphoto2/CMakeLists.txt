add_library(photo2 photo_reporter.cpp photo_camera_list.cpp photo_camera.cpp photo_image.cpp)
target_link_libraries(photo2
    ${OpenCV_LIBRARIES}
    )
install(
    TARGETS
      photo2
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
