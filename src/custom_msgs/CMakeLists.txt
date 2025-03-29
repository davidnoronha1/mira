cmake_minimum_required(VERSION 3.5)
project(custom_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

################################################
## Declare ROS messages, services, and actions #
################################################

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Commands.msg"
  "msg/Telemetry.msg"
  "msg/EmergencyKill.msg"
  "msg/Depth.msg"
  "msg/Heading.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

###################################
## Build and Install Directives ##
###################################

# Include directories
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Export package
ament_export_dependencies(rosidl_default_runtime sensor_msgs std_msgs)

# Mark the package as an ament package
ament_package()
