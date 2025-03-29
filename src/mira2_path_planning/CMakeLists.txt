cmake_minimum_required(VERSION 3.5)
project(mira2_path_planning)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(control_utils REQUIRED)

# Include directories
include_directories(
  include
)

# Declare a C++ executable
add_executable(gate_navigator_exe src/gate_navigator.cpp)

# Link libraries
ament_target_dependencies(gate_navigator_exe
  rclcpp
  geometry_msgs
  custom_msgs
  vision_msgs
  control_utils
)

# Install the executable
install(TARGETS gate_navigator_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
