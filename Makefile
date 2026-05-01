.PHONY: master alt_master build source install-deps submodules update install-udev bs fix-vscode dashboard telemetry-viz

export FORCE_COLOR=1
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT={severity} {message}
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MACHINE_IP=$(shell hostname -I | awk '{print $$1}')
export OPENCV_FFMPEG_CAPTURE_OPTIONS="fifo_size;500000|overrun_nonfatal;1|fflags;nobuffer|flags;low_delay|framedrop;1|vf;setpts=0"
export _UID=$(shell id -u)
export _GID=$(shell id -g)

ifeq ($(MACHINE_IP),192.168.2.6)
export MACHINE_NAME=ORIN
else ifeq ($(MACHINE_IP),192.168.2.4)
export MACHINE_NAME=RPI4
endif

SHELL := /bin/bash

# Check if commands/directories exist at parse time
UV_EXISTS := $(shell command -v uv 2>/dev/null)
VENV_EXISTS := $(wildcard .venv)
ROS_JAZZY_EXISTS := $(wildcard /opt/ros/jazzy)
MAVPROXY_EXISTS := $(wildcard .venv/bin/mavproxy*) $(shell command -v mavproxy 2>/dev/null)

all: build

check-uv:
ifndef UV_EXISTS
	$(error ❌ uv is not installed. Install it with: curl -LsSf https://astral.sh/uv/install.sh | sh)
endif

check-ros: check-uv
ifndef ROS_JAZZY_EXISTS
	$(error ❌ ROS Jazzy not found at /opt/ros/jazzy. Only ROS Jazzy is supported by this workspace.)
endif

build:
	@python3 mira.py build $(P)

b:
	@python3 mira.py b $(P)

repoversion:
	@python3 mira.py repoversion

docker-ensure:
	@python3 mira.py docker-ensure

docker-x11:
	xhost +local:docker || true

build-docker-container: docker-ensure
	$(info Building Docker container...)
	@docker compose build mira

docker-fix-perms:
	@python3 mira.py docker-fix-perms

docker: docker-x11
	@python3 mira.py docker

# Install dependencies
install-deps:
	@python3 mira.py install-deps

PYTHON_VERSION ?= python3.12
install-mavproxy:
	@python3 mira.py install-mavproxy $(PYTHON_VERSION)

shell:
	@python3 mira.py shell

proxy-pixhawk:
	@python3 mira.py proxy-pixhawk $(LAPTOP_IP)

# Get submodules
get-submodules:
	@python3 mira.py get-submodules

# Get latest from remote
force-update:
	@python3 mira.py force-update

# Install udev rules
install-udev:
	@python3 mira.py install-udev

# Fix VSCode settings paths
fix-vscode:
	@python3 mira.py fix-vscode

validate-all:
	@python3 mira.py validate-all

camera_zed:
	@python3 mira.py camera zed

camera_bottomcam:
	@python3 mira.py camera bottomcam

camera_auto:
	@python3 mira.py camera auto

camera_frontcam:
	@python3 mira.py camera frontcam

PIXHAWK_PORT ?= /dev/Pixhawk
alt_master:
	@python3 mira.py alt-master $(PIXHAWK_PORT)

alt_master_sitl:
	@python3 mira.py alt-master-sitl

teleop:
	@python3 mira.py teleop

ARUCO_ID ?= 5
VIDEO ?= rtsp://192.168.2.6:2000/image_rtsp
tune-lateral: check-ros
	source .venv/bin/activate && source install/setup.bash && ros2 launch mira2_pid_control aruco_tuner.launch.py target_id:=${ARUCO_ID} axis:=lateral rtsp_url:=${VIDEO}

tune-forward: check-ros
	source .venv/bin/activate && source install/setup.bash && ros2 launch mira2_pid_control aruco_tuner.launch.py target_id:=${ARUCO_ID} axis:=forward rtsp_url:=${VIDEO}

tune-gui: check-ros
	source .venv/bin/activate && source install/setup.bash && ros2 launch mira2_pid_control pid_tuner_gui.launch.py target_id:=${ARUCO_ID} rtsp_url:=${VIDEO}

# Development setup
setup:
	@python3 mira.py setup

# Clean build artifacts
clean:
	@python3 mira.py clean

# Help target
help:
	$(info Available targets:)
	$(info   build         - Build the ROS workspace)
	$(info   source        - Source the workspace environment)
	$(info   install-deps  - Install ROS dependencies with rosdep)
	$(info   submodules    - Update git submodules)
	$(info   proxy-pixhawk - Download and run mavp2p for Pixhawk telemetry proxying)
	$(info                  Use DEVPATH=/dev/ttyACM0 to specify device path if needed)
	$(info   update        - Get latest changes from remote)
	$(info   install-udev  - Install udev rules)
	$(info   b 		   - Build specific package (set P=package_name))
	$(info   bs            - Build and source workspace)
	$(info   fix-vscode    - Fix VSCode settings paths)
	$(info   setup         - Complete workspace setup)
	$(info   clean         - Clean build artifacts)
	$(info )
	$(info ROS Launch targets:)
	$(info   master        - Launch master control)
	$(info   alt_master    - Launch alternative master control)
	$(info   teleop        - Launch teleoperation)
	$(info )
	$(info Dashboard applications:)
	$(info   dashboard     - Launch main dashboard)
	$(info   telemetry-viz - Launch telemetry visualization)
	$(info )
	$(info   help          - Show this help message)


# Delegate any unrecognised target to mira.py
# Usage: make <task> [P=pkg] [LAPTOP_IP=x.x.x.x] [ARGS="extra args"]
%:
	python3 mira.py $@ $(P) $(LAPTOP_IP) $(ARGS)
