cmake_minimum_required(VERSION 3.8)
project(utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package so other ament_python packages can import it.
ament_python_install_package(${PROJECT_NAME})

# Export C++ headers so downstream ament_cmake packages can use #include <utils/image_source.hpp>.
install(DIRECTORY include/ DESTINATION include/)
ament_export_include_directories(include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
