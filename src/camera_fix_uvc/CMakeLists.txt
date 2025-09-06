cmake_minimum_required(VERSION 3.10)
project(camera_fix_uvc)

# ROS2 setup
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUVC REQUIRED libuvc)

include_directories(
    ${LIBUVC_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include
)

link_directories(
    ${LIBUVC_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
)

set(CAMERA_FIX_UVC_SOURCES
    src/main.cpp
    src/list_devices.cpp
    src/stream.cpp
)

add_executable(camera_fix_uvc ${CAMERA_FIX_UVC_SOURCES})

ament_target_dependencies(camera_fix_uvc
    rclcpp
    sensor_msgs
    OpenCV
)

target_link_libraries(camera_fix_uvc
    ${LIBUVC_LIBRARIES}
    ${OpenCV_LIBS}
    ${cv_bridge_LIBRARIES}
)

install(TARGETS camera_fix_uvc
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
