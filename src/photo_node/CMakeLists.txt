add_executable(photo_node photo_node.cpp)
target_link_libraries(photo_node
    photo2
    ${catkin_LIBRARIES}
    ${Gphoto2_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
