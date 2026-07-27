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

## Developing in the Container

If you don't want to install ROS2 Jazzy and all its dependencies directly on your machine, you can develop entirely inside the `mira` Docker container instead.

```
make docker
```

This starts (or attaches to) the container and drops you into a root shell inside `/workspace`, which is bind-mounted to your repo checkout — edits made on the host or in the container show up in both instantly. It picks the `mira` service (GPU passthrough) if CUDA is available on the host, otherwise `mira-nogpu`.

`docker-compose.yml` prefers pulling the pre-built image from `ghcr.io/dreadnought-robotics/mira:latest` (published by CI on every push/PR) and only falls back to building the `Dockerfile` locally if that image can't be pulled. To force a local rebuild instead of pulling:
```
make build-docker-container
```

Since the container mounts your repo directly, `make build`, `make b P=<package>` and `colcon test` all work the same as a native install once you're inside — the container already has ROS, `uv`, and the workspace deps installed.

> [!NOTE]
> If you run commands as root inside the container (e.g. installing an apt package), files it creates may end up owned by root on the host. Run `make docker-fix-perms` from the host afterwards to restore ownership.

### Dev Container (VSCode)
The repo also ships a [Dev Container](https://containers.dev/) config at `.devcontainer/devcontainer.json`, which reuses the same `docker-compose.yml`. Open the repo in VSCode and choose **"Reopen in Container"** to get a fully configured environment (ROS2 sourced, clangd, Python, and all the extensions from the Installation section above) without any host setup. It defaults to the `mira-nogpu` service — edit the `service` field in `devcontainer.json` to `mira` if you need GPU passthrough.

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

If you encounter issues when building, make sure you have _sourced_ ROS2, By doing:
```
source /opt/ros/jazzy/setup.bash
```
