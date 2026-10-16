.PHONY: pixhawk-controller async-master build source install-deps submodules update install-udev bs fix-vscode dashboard telemetry-viz

export FORCE_COLOR=1
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT={severity} {message}
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MACHINE_IP=$(shell hostname -I | awk '{print $$1}')
export OPENCV_FFMPEG_CAPTURE_OPTIONS="fifo_size;500000|overrun_nonfatal;1|fflags;nobuffer|flags;low_delay|framedrop;1|vf;setpts=0"
export _UID=$(shell id -u)
export _GID=$(shell id -g)

# Applied by mira.py before ROS processes start; no node chooses its own RMW.
ifneq ($(strip $(MIDDLEWARE)),)
export MIRA_MIDDLEWARE=$(MIDDLEWARE)
endif

ifeq ($(MACHINE_IP),192.168.2.6)
export MACHINE_NAME=ORIN
else ifeq ($(MACHINE_IP),192.168.2.4)
export MACHINE_NAME=RPI4
endif

SHELL := /bin/bash
# mira.py needs Python >= 3.11 (tomllib); hosts may ship an older default python3.
PYTHON ?= $(shell command -v python3.12 2>/dev/null || command -v python3)
WS = source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && source install/setup.bash && MIDDLEWARE_ENV="$$($(PYTHON) mira.py middleware-env)" && eval "$$MIDDLEWARE_ENV"

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
	@$(PYTHON) mira.py build $(P)

b:
	@$(PYTHON) mira.py b $(P)

repoversion:
	@$(PYTHON) mira.py repoversion

docker-ensure:
	@$(PYTHON) mira.py docker-ensure

docker-x11:
	xhost +local:docker || true

build-docker-container: docker-ensure
	$(info Building Docker container...)
	@docker compose build mira

docker-fix-perms:
	@$(PYTHON) mira.py docker-fix-perms

docker: docker-x11
	@$(PYTHON) mira.py docker

# Install dependencies
install-deps:
	@$(PYTHON) mira.py install-deps

PYTHON_VERSION ?= python3.12
install-mavproxy:
	@$(PYTHON) mira.py install-mavproxy $(PYTHON_VERSION)

shell:
	@$(PYTHON) mira.py shell

proxy-pixhawk:
	@$(PYTHON) mira.py proxy-pixhawk $(LAPTOP_IP)

# Get submodules
get-submodules:
	@$(PYTHON) mira.py get-submodules

# Get latest from remote
force-update:
	@$(PYTHON) mira.py force-update

# Install udev rules
install-udev:
	@$(PYTHON) mira.py install-udev

# Fix VSCode settings paths
fix-vscode:
	@$(PYTHON) mira.py fix-vscode

validate-all:
	@$(PYTHON) mira.py validate-all

camera_zed:
	@$(PYTHON) mira.py camera zed

camera_bottomcam:
	@$(PYTHON) mira.py camera bottomcam

camera_auto:
	@$(PYTHON) mira.py camera auto

camera_frontcam:
	@$(PYTHON) mira.py camera frontcam

PIXHAWK_PORT ?= /dev/Pixhawk
pixhawk-controller async-master:
	@$(PYTHON) mira.py pixhawk-controller $(PIXHAWK_PORT)

teleop:
	@$(PYTHON) mira.py teleop

# Development setup
setup:
	@$(PYTHON) mira.py setup

# Clean build artifacts
clean:
	@$(PYTHON) mira.py clean

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
	$(info   pixhawk-controller - Launch Pixhawk control)
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
	$(PYTHON) mira.py $@ $(P) $(LAPTOP_IP) $(ARGS)

.PHONY: config-check lint
config-check:
	python3 misc/tools/check_config.py

# Explicit developer commands; neither target executes tests.
lint:
	git diff --check
	python3 -m ruff check --select E9,F63,F7,F82 misc/infra src/mira2_hardware/mira2_hardware src/dependencies/utils/utils src/mira2_vision/mira2_vision

# Connect the normal guided stack to an independently running Gazebo/SITL.
# Run inside the original Mira container (or a configured host ROS workspace).
# Example: make guided-sim ARGS="pose_source:=optical_flow flow_scene_config:=/path/to/optical_flow.yaml"
# ARGS applies to both; package-specific overrides go in HARDWARE_ARGS / VISION_ARGS.
.PHONY: guided guided-sim
guided guided-sim:
	@$(WS) || exit $$?; \
	cleanup() { kill -TERM $$hardware_pid $$vision_pid 2>/dev/null || true; wait $$hardware_pid $$vision_pid 2>/dev/null || true; }; \
	trap cleanup EXIT; trap 'exit 130' INT; trap 'exit 143' TERM; \
	ros2 launch mira2_hardware $(if $(filter guided-sim,$@),guided_sim,guided).launch.py $(ARGS) $(HARDWARE_ARGS) & hardware_pid=$$!; \
	ros2 launch mira2_vision $(if $(filter guided-sim,$@),guided_sim,navigation).launch.py $(ARGS) $(VISION_ARGS) & vision_pid=$$!; \
	wait -n $$hardware_pid $$vision_pid
