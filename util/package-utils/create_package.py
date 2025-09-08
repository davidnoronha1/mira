#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom


def create_package_xml(package_name, package_path, build_type):
    """Creates a package.xml file with the specified build type."""
    
    # Create the root element
    package = ET.Element("package", format="3")
    
    # Add basic package information
    name = ET.SubElement(package, "name")
    name.text = package_name
    
    version = ET.SubElement(package, "version")
    version.text = "0.0.0"
    
    description = ET.SubElement(package, "description")
    description.text = "TODO: Package description"
    
    # Add maintainer info (you can customize these)
    maintainer = ET.SubElement(package, "maintainer", email="your.email@example.com")
    maintainer.text = "Your Name"
    
    license_elem = ET.SubElement(package, "license")
    license_elem.text = "TODO: License declaration"
    
    # Add build dependencies based on type
    if build_type == "ament_cmake":
        buildtool_depend = ET.SubElement(package, "buildtool_depend")
        buildtool_depend.text = "ament_cmake"
        
        test_depend1 = ET.SubElement(package, "test_depend")
        test_depend1.text = "ament_lint_auto"
        
        test_depend2 = ET.SubElement(package, "test_depend")
        test_depend2.text = "ament_lint_common"
    
    elif build_type == "ament_python":
        buildtool_depend = ET.SubElement(package, "buildtool_depend")
        buildtool_depend.text = "ament_python"
        
        test_depend1 = ET.SubElement(package, "test_depend")
        test_depend1.text = "ament_copyright"
        
        test_depend2 = ET.SubElement(package, "test_depend")
        test_depend2.text = "ament_flake8"
        
        test_depend3 = ET.SubElement(package, "test_depend")
        test_depend3.text = "ament_pep257"
        
        test_depend4 = ET.SubElement(package, "test_depend")
        test_depend4.text = "python3-pytest"
    
    # Add export section with build type
    export = ET.SubElement(package, "export")
    build_type_elem = ET.SubElement(export, "build_type")
    build_type_elem.text = build_type
    
    # Pretty print the XML
    rough_string = ET.tostring(package, 'unicode')
    reparsed = minidom.parseString(rough_string)
    pretty_xml = reparsed.toprettyxml(indent="  ")
    
    # Remove empty lines and fix formatting
    lines = [line for line in pretty_xml.split('\n') if line.strip()]
    pretty_xml = '\n'.join(lines[1:])  # Remove XML declaration
    
    # Write to file
    package_xml_path = package_path / "package.xml"
    with open(package_xml_path, 'w') as f:
        f.write(pretty_xml + '\n')
    
    print(f"✅ Created package.xml")


def create_cmake_files(package_name, package_path, include_launch):
    """Creates CMakeLists.txt and main.cpp for C++ packages."""
    
    cmake_content = f"""cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Create executable
add_executable({package_name}_exe src/main.cpp)
ament_target_dependencies({package_name}_exe rclcpp)

# Install executable
install(TARGETS {package_name}_exe
  DESTINATION lib/${{PROJECT_NAME}}
)
"""
    
    if include_launch:
        cmake_content += """
# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
"""
    
    cmake_content += """
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
"""

    # Write CMakeLists.txt
    cmake_path = package_path / "CMakeLists.txt"
    with open(cmake_path, 'w') as f:
        f.write(cmake_content)
    
    print(f"✅ Created CMakeLists.txt")
    
    # Create src directory and main.cpp
    src_dir = package_path / "src"
    src_dir.mkdir(exist_ok=True)
    
    main_cpp_content = f"""#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{{
  public:
    MinimalPublisher()
    : Node("{package_name}_node")
    {{
      RCLCPP_INFO(this->get_logger(), "{package_name} node has been started!");
    }}
}};

int main(int argc, char * argv[])
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}}
"""
    
    main_cpp_path = src_dir / "main.cpp"
    with open(main_cpp_path, 'w') as f:
        f.write(main_cpp_content)
    
    print(f"✅ Created src/main.cpp")


def create_python_files(package_name, package_path, include_launch):
    """Creates setup.py, setup.cfg, and resource files for Python packages."""
    
    # Create setup.py
    setup_py_content = f"""from setuptools import find_packages, setup
import os
from glob import glob

package_name = '{package_name}'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),"""
    
    if include_launch:
        setup_py_content += """
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))"""
    
    setup_py_content += f"""
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': [
        ],
    }},
)
"""
    
    setup_py_path = package_path / "setup.py"
    with open(setup_py_path, 'w') as f:
        f.write(setup_py_content)
    
    print(f"✅ Created setup.py")
    
    # Create setup.cfg
    setup_cfg_content = f"""[develop]
script_dir=$base/lib/{package_name}
[install]
install_scripts=$base/lib/{package_name}
[build_scripts]
executable = /usr/bin/env python3
"""
    
    setup_cfg_path = package_path / "setup.cfg"
    with open(setup_cfg_path, 'w') as f:
        f.write(setup_cfg_content)
    
    print(f"✅ Created setup.cfg")
    
    # Create resource directory and file
    resource_dir = package_path / "resource"
    resource_dir.mkdir(exist_ok=True)
    
    resource_file = resource_dir / package_name
    resource_file.touch()
    
    print(f"✅ Created resource/{package_name}")
    
    # Create package directory for Python modules
    python_package_dir = package_path / package_name
    python_package_dir.mkdir(exist_ok=True)
    
    init_file = python_package_dir / "__init__.py"
    init_file.touch()
    
    print(f"✅ Created {package_name}/__init__.py")


def create_launch_directory(package_path, package_name):
    """Creates a launch directory with a sample launch file."""
    launch_dir = package_path / "launch"
    launch_dir.mkdir(exist_ok=True)
    
    # Create a sample launch file
    launch_content = f"""from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='{package_name}',
            executable='{package_name}_exe',  # For C++ packages
            name='{package_name}_node'
        ),
    ])
"""
    
    launch_file_path = launch_dir / f"{package_name}_launch.py"
    with open(launch_file_path, 'w') as f:
        f.write(launch_content)
    
    print(f"✅ Created launch/{package_name}_launch.py")


def create_package(package_name, build_type, include_launch):
    """Main function to create a ROS2 package."""
    
    # Ensure src directory exists in current working directory
    src_dir = Path.cwd() / "src"
    if not src_dir.exists():
        print(f"❌ Error: 'src' directory does not exist in current working directory!")
        print(f"Current working directory: {Path.cwd()}")
        print(f"Please run this script from the workspace root directory that contains the 'src' folder.")
        return False
    
    # Create package directory inside src
    package_path = src_dir / package_name
    
    if package_path.exists():
        print(f"❌ Error: Package directory 'src/{package_name}' already exists!")
        return False
    
    try:
        package_path.mkdir()
        print(f"✅ Created directory 'src/{package_name}'")
        
        # Create package.xml
        create_package_xml(package_name, package_path, build_type)
        
        # Create build-specific files
        if build_type == "ament_cmake":
            create_cmake_files(package_name, package_path, include_launch)
        elif build_type == "ament_python":
            create_python_files(package_name, package_path, include_launch)
        
        # Create launch directory if requested
        if include_launch:
            create_launch_directory(package_path, package_name)
        
        print(f"\n🎉 Successfully created ROS2 {build_type} package 'src/{package_name}'!")
        
        if build_type == "ament_cmake":
            print(f"\nTo build your package:")
            print(f"  colcon build --packages-select {package_name}")
            print(f"  source install/setup.bash")
            print(f"  ros2 run {package_name} {package_name}_exe")
        else:
            print(f"\nTo build your package:")
            print(f"  colcon build --packages-select {package_name}")
            print(f"  source install/setup.bash")
        
        if include_launch:
            print(f"\nTo run with launch file:")
            print(f"  ros2 launch {package_name} {package_name}_launch.py")
        
        return True
        
    except Exception as e:
        print(f"❌ Error creating package: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Create a new ROS2 package with proper structure in src/ directory"
    )
    parser.add_argument(
        "package_name",
        help="Name of the ROS2 package to create"
    )
    
    args = parser.parse_args()
    package_name = args.package_name
    
    # Check if we're in the right directory
    print(f"Current working directory: {Path.cwd()}")
    src_path = Path.cwd() / "src"
    if src_path.exists():
        print(f"✅ Found 'src' directory")
    else:
        print(f"❌ No 'src' directory found in current working directory")
        print(f"Please run this script from your ROS2 workspace root directory")
        return
    
    # Ask for build type
    print("\nSelect package type:")
    print("1. C++ (ament_cmake)")
    print("2. Python (ament_python)")
    
    while True:
        choice = input("Enter your choice (1 or 2): ").strip()
        if choice == "1":
            build_type = "ament_cmake"
            break
        elif choice == "2":
            build_type = "ament_python"
            break
        else:
            print("Invalid choice. Please enter 1 or 2.")
    
    # Ask for launch files
    while True:
        launch_choice = input("Do you want to include launch files? (y/n): ").strip().lower()
        if launch_choice in ['y', 'yes']:
            include_launch = True
            break
        elif launch_choice in ['n', 'no']:
            include_launch = False
            break
        else:
            print("Please enter 'y' for yes or 'n' for no.")
    
    # Create the package
    create_package(package_name, build_type, include_launch)


if __name__ == "__main__":
    main()
