add_executable(photo_node photo_node.cpp)
target_link_libraries(photo_node
    photo2
    ${catkin_LIBRARIES}
    ${Gphoto2_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
add_dependencies(photo_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(photo_node ${PROJECT_NAME}_gencfg)
install(
    TARGETS
        photo_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
