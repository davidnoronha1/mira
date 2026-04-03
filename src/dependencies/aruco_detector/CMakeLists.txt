cmake_minimum_required(VERSION 3.5)
project(aruco_detector)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# find_package(rosidl_default_generators REQUIRED)

# If you have custom messages, use rosidl
# find_package(rosidl_default_generators REQUIRED)

# Message files
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/ConfidenceLevel.msg"
#   DEPENDENCIES geometry_msgs
# )

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Executables
add_executable(detector_node src/detector_node.cpp src/utils.cpp)
# add_executable(tf_node src/tf_node.cpp)

ament_target_dependencies(detector_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  custom_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  ament_index_cpp
)


# ament_export_dependencies(rosidl_default_runtime)

# ament_target_dependencies(tf_node
#   rclcpp
#   geometry_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# Link Eigen if needed
target_link_libraries(detector_node Eigen3::Eigen  ${OpenCV_LIBS})
# target_link_libraries(tf_node Eigen3::Eigen)
# ament_target_dependencies(detector_node rclcpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_EXPORTED_TARGETS)
# # Message dependencies
# rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
# target_link_libraries(detector_node "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  detector_node
  # tf_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (only if they exist)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()


# # Install calibration files
# install(DIRECTORY calibration_files/
#   DESTINATION share/${PROJECT_NAME}/calibration_files
# )

# Install headers if needed
# install(DIRECTORY include/
#   DESTINATION include
# )

ament_package()
