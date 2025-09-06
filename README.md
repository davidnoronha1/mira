# Mira Firmware
The firmware for the Mira AUV

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
ros-jazzy-ros-base
ros-jazzy-vision-msgs
```

Install:
```
uv init
source mira.sh
fix_vscode_settings
```

## Editing
Install clangd + python extensions in Vscode for the smoothest experience

## Usage

Make sure you aren't in a python virtual env already (conda), if you encounter python packages missing its usually because the uv venv isnt sourced or you are using conda

Sourcing:
```
source .venv/bin/activate
source install/setup.bash
```

Building:
```
make build
```

Normal usage:
```
source mira.sh
bs_ws
```
