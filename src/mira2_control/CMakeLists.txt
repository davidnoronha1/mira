cmake_minimum_required(VERSION 3.5)
project(mira2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(utils REQUIRED)

add_executable(joystick_exe src/joystick_main.cpp)
ament_target_dependencies(joystick_exe rclcpp sensor_msgs custom_msgs utils)

install(TARGETS joystick_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
