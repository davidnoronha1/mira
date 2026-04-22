# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Mira is an **Autonomous Underwater Vehicle (AUV)** firmware for the Mira2 platform by Dreadnought Robotics. Built on **ROS2 Jazzy** (Ubuntu 24.04 LTS), it provides autonomous navigation, perception, and control for competition tasks (gate traversal, docking, bucket retrieval). The primary flight controller is a **Pixhawk** communicating via **MAVLink**.

## Build & Development Commands

```bash
# Full workspace setup (first time)
make install-deps          # Install system + ROS dependencies
make setup                 # Complete setup: deps + submodules + build + udev + vscode

# Building
make build                 # Full colcon workspace build
make b P=<package_name>    # Build specific package
make clean                 # Clean build artifacts

# Development shell (sources ROS2 + workspace environment)
make shell

# Validation
make validate-all          # Validate all ROS packages

# Hardware / camera
make install-udev          # Install udev rules for device access

# VSCode
make fix-vscode            # Fix VSCode settings paths after setup
```

**Run tests:**
```bash
colcon test                             # All tests
colcon test --packages-select <pkg>     # Specific package
```

**Build a single package:**
```bash
make b P=mira2_actions
# or directly:
colcon build --packages-select <pkg> --cmake-args -GNinja -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
```

The workspace uses `uv` for Python dependency management (`.venv/`), `ccache` + `lld` for fast C++ builds, and `Ninja` as the CMake generator. Python 3.12 is required.

## Architecture

### Control Flow

```
Joystick (mira2_rov) ──────────────────────────────────────┐
                                                            ▼
Behavior Tree (mira2_actions) → Path Planning nodes → mira2_control_master → Pixhawk (MAVLink)
                                                            ▲
Perception (vision_boundingbox, aruco_detector) ───────────┘
```

### Packages

**Core packages (`src/`):**

| Package | Language | Purpose |
|---|---|---|
| `mira2_actions` | C++ | Behavior tree execution engine; leaf nodes for all autonomous motions |
| `mira2_control_master` | Python | MAVLink bridge to Pixhawk; telemetry publisher; command arbitration |
| `mira2_rov` | C++ | Gamepad → 8-channel PWM for teleoperation |
| `mira2_path_planning` | C++/Python | Gate navigator (4x PID), ArUco docking state machine, bucket task |
| `mira2_pid_control` | C++/Python | Interactive PID gain tuning nodes; hardcoded mission sequences |
| `mira2_perception` | Python | Camera launch coordination and RTSP streaming |

**Dependencies (`src/dependencies/`):**

| Package | Language | Purpose |
|---|---|---|
| `custom_msgs` | ROS IDL | Shared message types: `Commands` (8× int16 PWM), `Telemetry`, `ArucoPose`, `Object2D`, etc. |
| `camera_driver` | C++ | libUVC-based USB camera driver with crop support |
| `vision_boundingbox` | Python | YOLOv11n via ONNX Runtime; publishes `BoundingBox2DArray` to `/vision/detections/bounding_box` |
| `aruco_detector` | C++ | ArUco marker detection; publishes pose for docking |
| `cv_bridge` | C++ | Custom ROS2 `sensor_msgs::Image` ↔ `cv::Mat` bridge |
| `control_utils` | C++ | Reusable PID controller library |

### Key Topics / Interfaces

- `/rov/commands` — Teleoperation PWM commands (`custom_msgs/Commands`)
- `/master/commands` — Autonomous control PWM commands (`custom_msgs/Commands`)
- `/master/telemetry` — Vehicle state from Pixhawk (`custom_msgs/Telemetry`): IMU, quaternion, depth, battery, thruster PWMs
- `/vision/detections/bounding_box` — YOLO detections (`vision_msgs/BoundingBox2DArray`)
- `/emergency_kill` — Kill-switch signal (`custom_msgs/EmergencyKill`)

### Behavior Tree (mira2_actions)

Behavior trees are defined in XML under `src/mira2_actions/config/` (e.g., `default.xml`, `docking.xml`, `bucket_task.xml`). Leaf nodes are C++ classes registered via BehaviorTree.CPP factory in `src/main.cpp`. Real-time visualization via **Groot2** on port `1337`.

Motion nodes in `src/motions/`: `AlignXY`, `ApproachBoundingBox`, `ApproachWithDepth`, `HoldPosition`, `LateralEvasion`, `RotateToTargetHeading`, `YawSweep`.

### Docking State Machine (mira2_path_planning)

`dock_controller` (Python, 20 Hz): `SEARCH → ALIGN_XY → ALIGN_YAW → APPROACH → BLIND_LATCH → DOCKED`

Uses ArUco marker detection for pose feedback.

### Command Arbitration

`mira2_control_master` subscribes to both `/rov/commands` and `/master/commands` — **last received command wins**. Emergency kill signal locks arming.

## Code Conventions

- **C++:** clang-format enforced (`.clang-format`); clangd LSP (`.clangd`). Use `make b P=<pkg>` to catch build errors quickly.
- **Python:** flake8 + pep257 enforced via ament_lint tests. Python 3.12 required.
- **PID gains** are hardcoded in source files for some nodes (e.g., `video_hardcode_exe`) but configurable via YAML in `config/` for others — check the node's source before tuning.
- The `vision_depth` package is intentionally skipped in builds (`SKIP_PACKAGES` in Makefile).

## Hardware Targets

- **Jetson Orin** (aarch64): CUDA 12.6 wheels from Jetson AI Lab index
- **Raspberry Pi 4**: CPU-only fallback
- **SITL** (software-in-the-loop): Set `pixhawk_address` to `tcp:127.0.0.1:5760` in `mira2_control_master`

## CI/CD

GitHub Actions (`.github/workflows/build-check.yml`) runs a Docker build validation on every PR to `master`. No deployment step — the Docker image is built but not pushed.
