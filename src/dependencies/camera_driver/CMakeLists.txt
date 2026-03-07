cmake_minimum_required(VERSION 3.8)
project(camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)

# Find system libraries
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-1.0 gstreamer-rtsp-server-1.0)

add_executable(camera_driver_exe src/camera_driver_node.cpp)
target_include_directories(camera_driver_exe PRIVATE
  ${GST_INCLUDE_DIRS}
)
target_link_libraries(camera_driver_exe
  ${GST_LIBRARIES}
)
ament_target_dependencies(camera_driver_exe
  rclcpp
  image_transport
  camera_info_manager
  sensor_msgs
  std_msgs
)

# Install executable
install(TARGETS camera_driver_exe
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
