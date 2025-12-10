.PHONY: master alt_master build source install-deps submodules update install-udev bs fix-vscode dashboard telemetry-viz
SHELL := /bin/bash

WS := source .venv/bin/activate && source install/setup.bash

all: build

# Check if ROS Jazzy is available

check-uv:
	@if ! command -v uv >/dev/null 2>&1; then \
		echo "❌ Error: uv is not installed. Install it with:"; \
		echo "curl -LsSf https://astral.sh/uv/install.sh | sh"; \
		exit 1; \
	fi
	@if [ ! -d ".venv" ]; then \
		echo "⚠️  Python virtual environment not found at .venv. Creating one..."; \
		uv sync; \
		echo "✅ Virtual environment created and dependencies synced."; \
	else \
		echo "✅ Virtual environment found at .venv."; \
	fi


check-ros: check-uv
	@if [ ! -d "/opt/ros/jazzy" ]; then \
		echo "❌ Error: ROS Jazzy not found at /opt/ros/jazzy."; \
		echo "Only ROS Jazzy is supported by this workspace."; \
		exit 1; \
	fi
	@echo "✅ ROS Jazzy found."

# Build the workspace

CMAKE_ARGS:= -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
			 -DCMAKE_COLOR_DIAGNOSTICS=ON

#COLCON_ARGS:= --parallel-workers 4 \
			  #--cmake-args $(CMAKE_ARGS)
			  # --symlink-install \
			  # --merge-install


SKIP_PACKAGES:=--packages-skip vision_boundingbox vision_depth

build: check-ros
	@echo "If you built in docker last - you'll need to clean and rebuild"
	@echo "Building workspace..."
	@source /opt/ros/jazzy/setup.bash && \
	source .venv/bin/activate && \
	colcon build ${SKIP_PACKAGES}

repoversion:
	@git config --global --add safe.directory /workspace
	@echo "Last commit in repository:"
	@git log -1 --oneline

build-docker-container:
	@echo "Building Docker container..."
	@docker build -t mira .

UID := $(shell id -u)
GID := $(shell id -g)
build-in-docker:
	@echo "Building workspace inside Docker..."
	@docker run \
		--rm \
		-v $(PWD):/workspace \
		-u $(UID):$(GID) \
		-w /workspace mira \
		bash -c "make repoversion && \
		make clean && \
		source /opt/ros/jazzy/setup.bash && \
		source .venv/bin/activate && \
		colcon build ${COLCON_ARGS} ${SKIP_PACKAGES}"

docker:
	docker run -it --rm \
		-v $(PWD):/workspace \
		-u $(UID):$(GID) \
		-w /workspace mira \
		bash

b: check-ros
	@source /opt/ros/jazzy/setup.bash && \
	source .venv/bin/activate && \
	colcon build ${COLCON_ARGS} --packages-select ${P}

# Install dependencies
install-deps: check-ros check-uv
	@echo "Installing Python dependencies..."
	uv sync
	@echo "Installing ROS dependencies..."
	@source /opt/ros/jazzy/setup.bash && \
	rosdep install --from-paths src --ignore-src -r -y

PYTHON_VERSION ?= python3.12
install-mavproxy: check-uv
	@echo "Installing maxproxy"
	uv tool install mavproxy
	
	@echo "Applying patch for mavproxy"
	patch /home/$(USER)/.local/share/uv/tools/mavproxy/lib/$(PYTHON_VERSION)/site-packages/MAVProxy/modules/lib/rline.py < ./misc/patches/mavproxy_rline_fix.patch

proxy-pixhawk:
	@if ! command -v mavproxy.py >/dev/null 2>&1 && ! command -v mavproxy >/dev/null 2>&1; then \
		echo "❌ Error: mavproxy not found in PATH. Install with 'make install-mavproxy' or run 'uv tool install mavproxy'."; \
		exit 1; \
	fi
	uv run mavproxy.py --master=/dev/Pixhawk --baudrate 57600 --out udp:$(LAPTOP_IP):14550


# Get submodules
get-submodules:
	@echo "Updating git submodules..."
	@git submodule update --init --recursive

# Get latest from remote
force-update:
	@echo "Fetching latest changes from remote..."
	@git fetch origin
	@git reset --hard origin/$$(git rev-parse --abbrev-ref HEAD)

# Install udev rules
install-udev:
	@echo "Installing udev rules..."
	@sudo cp misc/udev/96-mira.rules /etc/udev/rules.d/
	@sudo udevadm control --reload-rules
	@sudo udevadm trigger

# Fix VSCode settings paths
fix-vscode:
	@echo "Fixing VSCode settings paths..."
	@current_dir=$$(realpath .); \
	settings_file=".vscode/settings.json"; \
	if [ -f "$$settings_file" ]; then \
		sed -i "s|/home/david/mira|$$current_dir|g" "$$settings_file"; \
		echo "Updated paths in $$settings_file"; \
	else \
		echo "settings.json not found in .vscode directory."; \
	fi

# ROS Launch targets
master: check-ros
	${WS} && ros2 launch mira2_control_master master.launch

PIXHAWK_PORT ?= /dev/Pixhawk
alt_master: check-ros
	${WS} && \
	ros2 launch mira2_control_master alt_master.launch pixhawk_address:=${PIXHAWK_PORT}

alt_master_sitl:
	make alt_master PIXHAWK_PORT=tcp:127.0.0.1:5760

teleop: check-ros
	${WS} && ros2 launch mira2_rov teleop.launch

# Dashboard applications
dashboard: check-ros
	${WS} && ros2 run mira2_dashboard mira2_dashboard_exe

telemetry-viz: check-ros
	${WS} && ros2 run mira2_dashboard telemetry_viz

# Development setup
setup: check-ros install-deps submodules build install-udev fix-vscode
	@echo "🚀 Complete workspace setup finished!"

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	@rm -rf build/ install/ log/
	@echo "Clean completed."

# Help target
help:
	@echo "Available targets:"
	@echo "  build         - Build the ROS workspace"
	@echo "  source        - Source the workspace environment"
	@echo "  install-deps  - Install ROS dependencies with rosdep"
	@echo "  submodules    - Update git submodules"
	@echo "  proxy-pixhawk - Download and run mavp2p for Pixhawk telemetry proxying"
	@echo "                 Use DEVPATH=/dev/ttyACM0 to specify device path if needed"
	@echo "  update        - Get latest changes from remote"
	@echo "  install-udev  - Install udev rules"
	@echo "  b 		   - Build specific package (set P=package_name)"
	@echo "  bs            - Build and source workspace"
	@echo "  fix-vscode    - Fix VSCode settings paths"
	@echo "  setup         - Complete workspace setup"
	@echo "  clean         - Clean build artifacts"
	@echo ""
	@echo "ROS Launch targets:"
	@echo "  master        - Launch master control"
	@echo "  alt_master    - Launch alternative master control"
	@echo "  teleop        - Launch teleoperation"
	@echo ""
	@echo "Dashboard applications:"
	@echo "  dashboard     - Launch main dashboard"
	@echo "  telemetry-viz - Launch telemetry visualization"
	@echo ""
	@echo "  help          - Show this help message"
