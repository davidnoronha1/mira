cmake_minimum_required(VERSION 3.8)
project(vision_boundingbox)

# Define backend macro - change to OPENVINO, TENSORRT, ONNX, etc.
set(BACKEND "OPENVINO" CACHE STRING "Backend to use: OPENVINO, TENSORRT, ONNX, etc.")
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
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)

# ─── Backend-specific dependencies ───────────────────────────────────────────

if(BACKEND STREQUAL "OPENVINO")
  find_package(OpenVINO REQUIRED)
  set(BACKEND_SOURCES
    src/backends/openvino/yolo_v11_openvino.cpp
  )
  set(BACKEND_LIBS openvino::runtime)

elseif(BACKEND STREQUAL "TENSORRT")
  # ── CUDA ──────────────────────────────────────────────────────────────────
  find_package(CUDA REQUIRED)

  # ── TensorRT ──────────────────────────────────────────────────────────────
  # Prefer explicit paths via cmake args:
  #   -DTensorRT_ROOT=/path/to/TensorRT
  # Falls back to standard system locations.
  find_path(TensorRT_INCLUDE_DIR
    NAMES NvInfer.h
    HINTS
      ${TensorRT_ROOT}/include
      /usr/include
      /usr/local/include
      /opt/TensorRT/include
  )

  find_library(TensorRT_LIBRARY
    NAMES nvinfer
    HINTS
      ${TensorRT_ROOT}/lib
      /usr/lib
      /usr/local/lib
      /opt/TensorRT/lib
  )

  find_library(TensorRT_PLUGIN_LIBRARY
    NAMES nvinfer_plugin
    HINTS
      ${TensorRT_ROOT}/lib
      /usr/lib
      /usr/local/lib
      /opt/TensorRT/lib
  )

  if(NOT TensorRT_INCLUDE_DIR OR NOT TensorRT_LIBRARY)
    message(FATAL_ERROR
      "TensorRT not found. Set TensorRT_ROOT to your TensorRT installation directory, "
      "e.g. -DTensorRT_ROOT=/opt/TensorRT-10.x")
  endif()

  message(STATUS "TensorRT include : ${TensorRT_INCLUDE_DIR}")
  message(STATUS "TensorRT library : ${TensorRT_LIBRARY}")

  set(BACKEND_SOURCES
    src/backends/tensorrt/yolo_v11_tensorrt.cpp
  )

  set(BACKEND_LIBS
    ${TensorRT_LIBRARY}
    ${TensorRT_PLUGIN_LIBRARY}
    ${CUDA_LIBRARIES}
  )

  set(BACKEND_INCLUDE_DIRS
    ${TensorRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
  )

elseif(BACKEND STREQUAL "ONNX")
  message(FATAL_ERROR "ONNX backend not implemented yet")

else()
  message(FATAL_ERROR "Unsupported BACKEND: ${BACKEND}")
endif()

# ─── Common ───────────────────────────────────────────────────────────────────

find_package(ament_index_cpp REQUIRED)

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
  $<INSTALL_INTERFACE:include>
  ${BACKEND_INCLUDE_DIRS}
)

ament_target_dependencies(vision_boundingbox_exe
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  cv_bridge
  vision_msgs
  image_transport
  std_srvs
)

install(TARGETS vision_boundingbox_exe
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(vision_boundingbox_exe
  ${BACKEND_LIBS}
  stdc++_libbacktrace
  ${OpenCV_LIBS}
)

# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()