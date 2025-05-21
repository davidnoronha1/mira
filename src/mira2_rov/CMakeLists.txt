cmake_minimum_required(VERSION 3.5)
project(mira2_rov)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Declare a C++ executable
add_executable(joystick_exe src/joystick_main.cpp)

# Link libraries
ament_target_dependencies(joystick_exe
  rclcpp
  std_msgs
  custom_msgs
)


# Install targets
install(TARGETS joystick_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Export package
ament_package()
