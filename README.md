<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/1f643f54-825b-4657-b129-be1897e39837" />

# Mira Firmware
The firmware for the Mira AUV
Please refer to the repository Wiki for detailed usage instructions and documentation. The Wiki can be accessed from the tab near the Security section at the top of this page.


> [!IMPORTANT]
> This software package is designed to run on ROS2 Jazzy which supports Ubuntu 24.04 LTS as Tier 1 and Ubuntu 22.04 as Tier 3

## Packages
- dependencies/
	- camera_driver - libUVC based camera node (Git submodule)
	- control_utils - Contains control utilities
	- custom_msgs - Contains the message definitions used throughout
	- mira2_vision - Shared YOLO, ArUco, docking-board and pipeline perception
- mira2_hardware - Takes input from ROS and forwards to PixHawk
- mira2_path_planning - Contains code to navigate a gate
- mira2_perception - Contains our perception stack
- mira2_control - Joystick teleoperation and four-axis PID tuning GUI

## Installation
Packages to install:
```
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | less

# Clone the repository
git clone https://github.com/Dreadnought-Robotics/mira.git
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

Middleware profiles and the ongoing stack cleanup are documented
in [Stack refactor foundations](misc/doc/STACK_REFACTOR.md).

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

If you encounter issues when building, make sure you have _sourced_ ROS2, By doing:
```
source /opt/ros/jazzy/setup.bash
```


Run the integrated sibling simulator with `make sim-build` then `make sim`.
`make sim HEADLESS=1 RUN_ACTIONS=0` starts navigation without the demo;
`make sim-actions TARGET_CLASS=dock` runs it after a localized target is visible.
Use `make sim-check` to inspect configuration and `make sim-down` to stop it.
See [simulator integration](../mira_sim/docs/MIRA_STACK.md).

`camera_driver` is tracked on `new_arch` (the exact revision is pinned by the
submodule). Its ROS entry point accepts a `pipeline_config` YAML path; see
`camera_driver/launch/pipeline.launch.py` and `config/ros2_test.yaml` for a
synthetic-camera example. The simulator uses Gazebo RGB-D instead and
builds only its required Mira packages. `control_utils` and the unused
`zed-ros2-interfaces` dependency have been removed; shared control helpers remain
in `utils` and ROS `cv_bridge` remains a system dependency.
