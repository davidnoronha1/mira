cmake_minimum_required(VERSION 3.8)
project(control_utils)

set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Add executable for PID demo
add_executable(pid_demo src/pid_demo.cpp)
ament_target_dependencies(pid_demo rclcpp std_msgs std_srvs)
target_include_directories(pid_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pid_demo stdc++_libbacktrace)

ament_export_dependencies(std_msgs std_srvs)
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
	scripts/tuner.py
	DESTINATION
	lib/${PROJECT_NAME}
)

# Install executable
install(TARGETS pid_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
