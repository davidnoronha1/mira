cmake_minimum_required(VERSION 3.5)
project(mira2_pid_control)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_utils REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(std_srvs REQUIRED)
# Include directories
include_directories(
  include
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# Declare C++ executables
# add_executable(go_down_exe src/go_down.cpp)
add_executable(depth_tuning_exe src/depth_tuner.cpp)
add_executable(yaw_tuning_exe src/yaw_tuner.cpp)
add_executable(lateral_tuning_exe src/lateral_tuner.cpp)
add_executable(forward_tuning_exe src/forward_tuner.cpp)

# SAUVC competition-specific executables
if(DEFINED ENV{COMPETITION} AND "$ENV{COMPETITION}" STREQUAL "SAUVC")
  add_executable(video_hardcode_exe src/sauvc/video_submission_hardcode.cpp)
  add_executable(flare_imu src/sauvc/flare_imu.cpp)
  add_executable(flare_timed src/sauvc/flare_avoidance_timed.cpp)
endif()
# Link libraries
# ament_target_dependencies(go_down_exe rclcpp std_srvs std_msgs custom_msgs geometry_msgs control_utils)
ament_target_dependencies(depth_tuning_exe rclcpp std_srvs std_msgs custom_msgs geometry_msgs control_utils)
ament_target_dependencies(yaw_tuning_exe rclcpp std_srvs std_msgs custom_msgs geometry_msgs control_utils)
ament_target_dependencies(lateral_tuning_exe rclcpp std_msgs custom_msgs geometry_msgs control_utils)
ament_target_dependencies(forward_tuning_exe rclcpp std_msgs custom_msgs geometry_msgs control_utils)

# SAUVC competition-specific dependencies
if(DEFINED ENV{COMPETITION} AND "$ENV{COMPETITION}" STREQUAL "SAUVC")
  ament_target_dependencies(video_hardcode_exe rclcpp std_msgs std_srvs custom_msgs geometry_msgs control_utils)
  ament_target_dependencies(flare_imu rclcpp std_msgs std_srvs custom_msgs geometry_msgs control_utils)
  ament_target_dependencies(flare_timed rclcpp std_msgs std_srvs custom_msgs geometry_msgs control_utils)
endif()
# Install targets
install(TARGETS
  depth_tuning_exe
  yaw_tuning_exe
  lateral_tuning_exe
  forward_tuning_exe
  DESTINATION lib/${PROJECT_NAME}
)

# SAUVC competition-specific install targets
if(DEFINED ENV{COMPETITION} AND "$ENV{COMPETITION}" STREQUAL "SAUVC")
  install(TARGETS
    video_hardcode_exe
    flare_imu
    flare_timed
    DESTINATION lib/${PROJECT_NAME}
  )
endif()
# Install include directory
# install(DIRECTORY include/
#  DESTINATION include/
# )
# Install other files (e.g., launch files)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
	scripts/key_pub.py
	scripts/pid_tuner_gui.py
	DESTINATION
	lib/${PROJECT_NAME}
)

# SAUVC competition-specific scripts
if(DEFINED ENV{COMPETITION} AND "$ENV{COMPETITION}" STREQUAL "SAUVC")

  install(
    DIRECTORY ./resource
    DESTINATION share/${PROJECT_NAME}
  )

  install(PROGRAMS
    src/sauvc/bucket_p1.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()


# Export package
ament_package()
