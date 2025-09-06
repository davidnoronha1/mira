import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
import re

# XML snippets to be printed on successful check
AMENT_CMAKE_SNIPPET = """<!-- Snippet for ament_cmake build type -->

<export>
  <build_type>ament_cmake</build_type>
</export>
"""

AMENT_PYTHON_SNIPPET = """<!-- Snippet for ament_python build type -->

<export>
  <build_type>ament_python</build_type>
</export>
"""

CMAKE_LAUNCH_INSTALL_SNIPPET = """
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
"""

CMAKE_INCLUDE_INSTALL_SNIPPET = """
install(DIRECTORY include/
  DESTINATION include/
)
"""

SETUP_CFG_SNIPPET = """
[develop]
script_dir=$base/lib/{package_name}
[install]
install_scripts=$base/lib/{package_name}
[build_scripts]
executable = /usr/bin/env python3
"""

CMAKE_EXECUTABLE_INSTALL_SNIPPET = """
install(TARGETS {executable_name}
  DESTINATION lib/${{PROJECT_NAME}}
)
"""

def find_dependencies_xml(package_xml_path):
    """Parses package.xml and prints all dependencies."""
    try:
        tree = ET.parse(package_xml_path)
        root = tree.getroot()
        dependencies = set()
        for element in root.findall('./*'):
            if element.tag.endswith('depend'):
                dependencies.add(element.text.strip())
        
        print("\n--- Dependencies from package.xml ---")
        if dependencies:
            for dep in sorted(dependencies):
                print(f"  - {dep}")
        else:
            print("  - No dependencies found.")
        print("---")
    except ET.ParseError as e:
        print(f"❌ Error parsing '{package_xml_path}': {e}")
    except Exception as e:
        print(f"❌ An unexpected error occurred while reading dependencies from XML: {e}")

def find_dependencies_cmake(cmake_content):
    """Finds and prints all find_package() calls in CMakeLists.txt."""
    dependencies = set()
    # Regex to find find_package calls and capture the package name
    matches = re.findall(r'find_package\(([\w_]+)(?:\s+.*?)?\)', cmake_content)
    dependencies.update(matches)
    
    print("\n--- Dependencies from CMakeLists.txt ---")
    if dependencies:
        for dep in sorted(dependencies):
            print(f"  - {dep}")
    else:
        print("  - No find_package() calls found.")
    print("---")

def check_build_files(path):
    """
    Parses package.xml to find the build type, then checks for the presence of
    required build files.

    Args:
        path (Path): The Path object for the directory to check.
    """
    # Check if the provided path is a valid directory
    if not path.is_dir():
        print(f"❌ Error: The provided path '{path}' is not a valid directory.")
        return False

    package_xml_path = path / "package.xml"
    if not package_xml_path.exists():
        print(f"❌ Error: No 'package.xml' file found in '{path}'.")
        return False

    find_dependencies_xml(package_xml_path)

    try:
        tree = ET.parse(package_xml_path)
        root = tree.getroot()
        build_type_element = root.find('export/build_type')
        if build_type_element is not None and build_type_element.text:
            build_type = build_type_element.text.strip().lower()
            print(f"Found build type '{build_type}' in package.xml.")
        else:
            build_type = None

    except ET.ParseError as e:
        print(f"❌ Error parsing 'package.xml': {e}")
        return False

    if build_type == 'ament_cmake':
        cmake_file = Path('CMakeLists.txt')
        cmake_path = path / cmake_file
        if cmake_path.exists():
            print(f"✅ Success: '{cmake_file}' found.")
            
            try:
                cmake_content = cmake_path.read_text()
                find_dependencies_cmake(cmake_content)

                # Check for launch directory and snippet
                launch_dir = path / 'launch'
                if launch_dir.is_dir():
                    print("Found 'launch' subdirectory. Checking CMakeLists.txt for install rule...")
                    if 'install(DIRECTORY launch/' in cmake_content:
                        print("✅ Success: 'install(DIRECTORY launch/...' snippet found.")
                    else:
                        print(f"❌ Warning: 'launch' subdirectory found but missing install rule in '{cmake_file}'.")
                        print("\nPlease add the following snippet to your CMakeLists.txt:")
                        print(CMAKE_LAUNCH_INSTALL_SNIPPET)
                
                # Check for include directory and snippet
                include_dir = path / 'include'
                if include_dir.is_dir():
                    print("Found 'include' subdirectory. Checking CMakeLists.txt for install rule...")
                    if 'install(DIRECTORY include/' in cmake_content:
                        print("✅ Success: 'install(DIRECTORY include/...' snippet found.")
                    else:
                        print(f"❌ Warning: 'include' subdirectory found but missing install rule in '{cmake_file}'.")
                        print("\nPlease add the following snippet to your CMakeLists.txt:")
                        print(CMAKE_INCLUDE_INSTALL_SNIPPET)

                # Check for executables and their install rules
                executable_targets = re.findall(r'add_executable\(([\w_]+)\s', cmake_content)
                if not executable_targets:
                    print("❌ Warning: No executables found in CMakeLists.txt.")
                else:
                    installed_targets = re.findall(r'install\(TARGETS\s+([\w_]+)\s+DESTINATION\s+lib/\$\{PROJECT_NAME}\)', cmake_content)
                    installed_set = set(installed_targets)

                    for exe in executable_targets:
                        if exe not in installed_set:
                            print(f"❌ Warning: Executable '{exe}' is not installed correctly.")
                            print("Please ensure it has the correct install rule:")
                            print(CMAKE_EXECUTABLE_INSTALL_SNIPPET.format(executable_name=exe))
                        else:
                            print(f"✅ Success: Executable '{exe}' has a correct install rule.")

            except Exception as e:
                print(f"❌ Error reading '{cmake_file}': {e}")

            return True
        else:
            print(f"❌ Failure: '{cmake_file}' not found.")
            return False

    elif build_type == 'ament_python':
        files_to_check = [Path('setup.py'), Path('setup.cfg')]
        all_found = True
        for file in files_to_check:
            file_path = path / file
            if file_path.exists():
                print(f"✅ Success: '{file}' found.")
            else:
                print(f"❌ Failure: '{file}' not found.")
                all_found = False
        
        if all_found:
            print("\n✅ All required files found.")
            
            # Additional check for setup.cfg content
            setup_cfg_path = path / 'setup.cfg'
            try:
                setup_cfg_content = setup_cfg_path.read_text()
                package_name = path.name
                required_content = SETUP_CFG_SNIPPET.strip().format(package_name=package_name).strip()
                if setup_cfg_content.strip() == required_content:
                    print("✅ Success: 'setup.cfg' content matches required format.")
                else:
                    print(f"❌ Warning: 'setup.cfg' content does not match the required format.")
                    print("\nPlease update your setup.cfg to the following content:")
                    print(SETUP_CFG_SNIPPET.format(package_name=package_name))

            except Exception as e:
                print(f"❌ Error reading '{setup_cfg_path}': {e}")

            return True
        else:
            return False

    else:
        # Provide a more descriptive and "pretty" error message
        # For more advanced output formatting, consider using a library like 'rich'.
        print("\n--- Invalid or Missing Build Type ---")
        print("Could not find a valid build type in the package.xml's <export> tag.")
        print("Please ensure it contains either 'ament_cmake' or 'ament_python'.")
        print("\nHere are the XML snippets you can use in your package.xml file:")
        print("\n--- ament_cmake snippet ---")
        print(AMENT_CMAKE_SNIPPET)
        print("--- ament_python snippet ---")
        print(AMENT_PYTHON_SNIPPET)
        print("---")
        return False

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Check for required ROS 2 build files and provide export snippets by reading package.xml."
    )
    parser.add_argument(
        "path_to_check",
        type=Path,
        help="The path to the directory containing package.xml."
    )
    
    args = parser.parse_args()
    
    check_build_files(args.path_to_check)
