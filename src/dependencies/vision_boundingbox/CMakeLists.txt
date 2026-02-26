cmake_minimum_required(VERSION 3.8)
project(vision_boundingbox)

# Define backend macro - change to OPENVINO, ONNX, etc.
set(BACKEND "OPENVINO" CACHE STRING "Backend to use: OPENVINO, ONNX, etc.")
add_compile_definitions(BACKEND=${BACKEND})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Backend-specific dependencies
if(BACKEND STREQUAL "OPENVINO")
  find_package(OpenVINO REQUIRED)
  set(BACKEND_SOURCES 
    src/backends/openvino/yolo_v11_openvino.cpp
  )
  set(BACKEND_LIBS openvino::runtime)
elseif(BACKEND STREQUAL "ONNX")
  # Add ONNXRuntime find_package and sources here
  message(FATAL_ERROR "ONNX backend not implemented yet")
else()
  message(FATAL_ERROR "Unsupported BACKEND: ${BACKEND}")
endif()

find_package(ament_index_cpp REQUIRED)

# Create executable
add_executable(vision_boundingbox_exe 
  src/main.cpp
  src/helpers.cpp
  ${BACKEND_SOURCES}
)

install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

target_include_directories(vision_boundingbox_exe PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(vision_boundingbox_exe
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Install executable
install(TARGETS vision_boundingbox_exe
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(vision_boundingbox_exe
  ${BACKEND_LIBS}
  stdc++_libbacktrace
  ${OpenCV_LIBS}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
