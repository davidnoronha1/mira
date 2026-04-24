cmake_minimum_required(VERSION 3.8)
project(mira2_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(control_utils REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(zed_msgs REQUIRED)

# Create executable
add_executable(mira2_actions_exe 
  src/main.cpp 
  src/behaviours/DetectObjectBoundingbox.cpp
  src/behaviours/DetectTargetPoint.cpp
  src/motions/ApproachBoundingBox.cpp
  src/motions/Approach3DBBox.cpp
  src/motions/AlignXY.cpp
  src/motions/ApproachWithDepth.cpp
  src/motions/HoldPosition.cpp
  src/motions/TimedForward.cpp
  src/motions/LateralEvasion.cpp
  src/motions/RotateToTargetHeading.cpp
  src/motions/YawSweep.cpp)
 ament_target_dependencies(mira2_actions_exe
  rclcpp
  Behaviortree_cpp
  control_utils
  custom_msgs
  vision_msgs
  geometry_msgs
  std_msgs
  ament_index_cpp
  tf2
  tf2_ros
  zed_msgs)

# Install executable
install(TARGETS mira2_actions_exe
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
