cmake_minimum_required(VERSION 3.8)
project(vision_depth)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(vision_depth_exe
  src/core.cpp
  src/util.cpp
  src/depth_anything.cpp
)

install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_target_dependencies(vision_depth_exe
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

target_include_directories(vision_depth_exe PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against OpenVINO runtime + OpenCV
target_link_libraries(vision_depth_exe
  openvino::runtime
  ${OpenCV_LIBS}
)

install(TARGETS vision_depth_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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
