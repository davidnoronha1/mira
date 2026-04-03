<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/1f643f54-825b-4657-b129-be1897e39837" />

# Mira Firmware
The firmware for the Mira AUV
Please refer to the repository Wiki for detailed usage instructions and documentation. The Wiki can be accessed from the tab near the Security section at the top of this page.


> [!IMPORTANT]
> This software package is designed to run on ROS2 Jazzy which supports Ubuntu 24.04 LTS as Tier 1 and Ubuntu 22.04 as Tier 3

## Packages
- dependencies/
	- aruco_detector - Node to consume a ROS image feed and publish any detected aruco markers with pose
	- camera_driver_uvc - libUVC based camera node
	- control_utils - Contains utilities used in mira2_pid_control
	- custom_msgs - Contains the message definitions used throughout
	- vision_boundingbox - Contains the optimized node for Yolo11n inference on AUV hardware
	- vision_depth - Same but for depth anything (^)
- mira2_control_master - Takes input from ROS and forwards to PixHawk
- mira2_path_planning - Contains code to navigate a gate
- mira2_perception - Contains our perception stack
- mira2_pid_control - Contains depth tuner and yaw tuner for calibrating the control systems
- mira2_rov - Contains our joystick controller code

## Installation
Packages to install:
```
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | less

# Clone the repository
git clone https://github.com/davidnoronha1/mira.git
cd mira

# Install dependencies
make install-deps

# Sync virtual enviroment (python)
uv sync

# Fix for editor autocompletion
make fix-vscode
```

## Editing

### VSCode Extensions
Install the following extensions in vscode for autocompletion:
- [clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd)
- [python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)

The settings should be picked up from `.vscode/settings.json`
There should be a popup in Vscode to install the same extensions

### Package Validator
To validate that you haven't done any mistakes in a ROS package, you can run the following 
```
uv run util/validator/validate_package.py ./src/<PACKAGE NAME>
```

## Usage

> [!NOTE]
> Make sure you aren't in a python virtual env already (conda), if you encounter python packages missing its usually because the uv venv isnt sourced or you are using conda

### Sourcing
```
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```

### Building
```
make build
```
or 
```
colcon build
```

If you encounter issues when building, make sure you have _sourced_ ROS2, By doing:
```
source /opt/ros/jazzy/setup.bash
```
