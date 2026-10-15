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
Joystick (mira2_control) ──────────────────────────────────┐
                                                            ▼
Behavior Tree (mira2_actions) → Path Planning nodes → mira2_hardware → Pixhawk (MAVLink)
                                                            ▲
Perception (mira2_vision plugins) ───────────┘
```

### Packages

**Core packages (`src/`):**

| Package | Language | Purpose |
|---|---|---|
| `mira2_actions` | C++ | Behavior tree execution engine; leaf nodes for all autonomous motions |
| `mira2_hardware` | Python | MAVLink bridge to Pixhawk; telemetry publisher; command arbitration |
| `mira2_path_planning` | C++/Python | Gate navigator (4x PID), ArUco docking state machine, bucket task |
| `mira2_control` | C++/Python | Gamepad teleoperation and four-axis PID GUI with YOLO/pose/optional-marker feedback |
| `mira2_perception` | Python | Camera launch coordination and RTSP streaming |

**Dependencies (`src/dependencies/`):**

| Package | Language | Purpose |
|---|---|---|
| `custom_msgs` | ROS IDL | Shared message types: `CommandsNormalized` (6-axis FLU effort), `Telemetry`, `ArucoPose`, `Object2D`, etc. |
| `camera_driver` | C++ | libUVC-based USB camera driver with crop support |
| `mira2_vision` | Python | In-process detector and consumer plugins per camera; shared YOLO/ArUco results, independent pipeline detector |
| `cv_bridge` | ROS dependency | System ROS image conversion package |
| `utils` | C++/Python | Shared normalized PID, calibration and perception geometry |

### Key Topics / Interfaces

- `/mira/control/{autonomy,teleop,tuning}/command` — Stamped normalized body-FLU effort (`custom_msgs/CommandsNormalized`)
- `/mira/control/command` — Selected normalized command, published by the master for recording
- `/mira/state/telemetry` — Pixhawk state (`custom_msgs/Telemetry`); depth feedback is external pressure in hPa
- `/mira/perception/front/objects` — YOLO (`vision_msgs/Detection2DArray`), normalized bounding boxes
- `/mira/perception/bottom/markers` — Marker ID and optical-frame pose (`custom_msgs/ArucoPose`)
- `/mira/safety/kill` — Emergency signal (`custom_msgs/EmergencyKill`)

### Behavior Tree (mira2_actions)

The single demo is `src/mira2_actions/config/demo.xml`. `SelectObject` selects a
fresh depth-localized YOLO Detection2D of a requested class, then `NavigateToPose`
takes that typed object, resolves its associated pose through TF, and sends a
GUIDED goal. The shared controllers live in Pixhawk. No per-task PID YAML or
raw waypoint actions remain. Groot2 visualization uses port 1337.

### Docking State Machine (mira2_path_planning)

`dock_controller` (Python, 20 Hz): `SEARCH → ALIGN_XY → ALIGN_YAW → APPROACH → BLIND_LATCH → DOCKED`

Uses ArUco marker detection for pose feedback.

### Command Arbitration

One `master` selects `control_source=autonomy|teleop|tuning`. Only that source is
accepted; changing sources requires disarming. Normalized input is stamped and
validated. The default master uses bounded synchronous telemetry reads; async_master offers
nonblocking receive polling with shared vehicle behavior. Both handle heartbeat loss.
After a failsafe the source must issue a disarmed request before rearming.
Missions publish normalized body-effort commands.
The original /master/commands endpoint and alt_master launch name remain supported
for the operational alt_master baseline. See the control-master README.

## Code Conventions

- **C++:** clang-format enforced (`.clang-format`); clangd LSP (`.clangd`). Use `make b P=<pkg>` to catch build errors quickly.
- **Python:** flake8 + pep257 enforced via ament_lint tests. Python 3.12 required.
- Middleware belongs to deployment profiles, never application nodes. See
  `config/middleware/profiles.toml` and `misc/doc/STACK_REFACTOR.md`.
- Keep configuration schemas and YAML with their owning component. Python nodes
  use `utils.parameters.declare_config` for mappings, `dataclass_config` for flat
  dataclasses, and `Setting(default, validator)` for startup constraints. C++ nodes
  use `utils::parameter` / `utils::checked_parameter` from `utils/parameters.hpp`.
  ROS loads parameter files and CLI overrides; do not extract `/**` in launches.
  Missing values use declared defaults; invalid overrides fail startup. Mapping
  and dataclass loaders default to read-only parameters; use `mutable` or
  `Setting(..., read_only=False)` with a node update callback for live settings.
  Returned config values are startup snapshots, not automatically updated views.
- Local validation under `tests/` is intentionally excluded from commits for now.
- **PID gains** are hardcoded in source files for some nodes (e.g., `video_hardcode_exe`) but configurable via YAML in `config/` for others — check the node's source before tuning.
- The `vision_depth` package is intentionally skipped in builds (`SKIP_PACKAGES` in Makefile).

## Hardware Targets

- **Jetson Orin** (aarch64): CUDA 12.6 wheels from Jetson AI Lab index
- **Raspberry Pi 4**: CPU-only fallback
- **SITL** (software-in-the-loop): Set `pixhawk_address` to `tcp:127.0.0.1:5760` in `mira2_hardware`

## CI/CD

GitHub Actions (`.github/workflows/build-check.yml`) runs a Docker build validation on every PR to `master`. No deployment step — the Docker image is built but not pushed.
