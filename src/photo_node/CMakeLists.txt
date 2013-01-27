add_executable(photo_node photo_node.cpp)
target_link_libraries(photo_node
    photo2
    ${catkin_LIBRARIES}
    ${Gphoto2_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
install(
    TARGETS
        photo_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
