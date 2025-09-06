# Mira Firmware
The firmware for the Mira AUV

> [!IMPORTANT]
> This software package is designed to run on ROS2 Jazzy which supports Ubuntu 24.04 LTS as Tier 1 and Ubuntu 22.04 as Tier 3

## Packages
- camera_fix_uvc/ - The camera reader
- control_utils/ - Contains the utilities for the AUV's Control System
- custom_msgs/ - Contains custom ROS message definitions mostly for telemetry & controls
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

# Install dependencies
sudo apt install ros-jazzy-ros-base \
ros-jazzy-vision-msgs

# Clone the repository
git clone https://github.com/davidnoronha1/mira.git
cd mira

# Sync virtual enviroment (python)
uv sync

# Fix for editor autocompletion
source mira.sh
fix_vscode_settings
```

## Editing
Install the following extensions in vscode for autocompletion:
- [clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd)
- [python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)

The settings should be picked up from `.vscode/settings.json`
There should be a popup in Vscode to install the same extensions

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