cmake_minimum_required(VERSION 2.8.3)
project(photo)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            std_msgs
            sensor_msgs
            message_generation
            message_runtime
)

set( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules )
find_package(Gphoto2 REQUIRED)

find_package(OpenCV REQUIRED)

# services
add_service_files(
    DIRECTORY srv
    FILES
        Capture.srv
        GetConfig.srv
        SetConfig.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
        sensor_msgs
)



catkin_package(
    CATKIN_DEPENDS
            roscpp
)

set(ROS_BUILD_TYPE Debug)

include_directories(
    include
    ${Gphoto2_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_subdirectory(src)
