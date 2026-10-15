# Continue Mira stack refactor

## Latest requested changes

- Replaced vision_boundingbox with mira2_vision: explicit launch-selected,
  same-process detector/consumer plugins, shared YOLO/ArUco products, independent
  HSV pipeline detector, and external module/entry-point extension support.
  All five model assets retain their original Git hashes.
- Renamed qauli.launch to qualification.launch. Path-planning controllers remain:
  BTs still lack equivalent qualification, pipeline, recovery, and terminal behavior.
- PID tuner defaults to automatic visual feedback: fresh configured ArUco marker
  first, YOLO boxes otherwise. Metric and box PID profiles are separate; effective
  source changes disarm. Explicit source modes remain; depth/yaw are telemetry-only.
- Static Python/YAML/XML parsing and whitespace checks passed. No runtime tests,
  ROS builds, simulator or hardware validation ran for these changes.

## Current status — 2026-10-14

Implementation continued without running tests or builds, as requested. Existing
uncommitted changes were preserved. `mira_sim` and local test files were not changed.
Historical 53-test results do not validate the changes below.

## Implemented in this continuation

- Gate, docking, bucket, flare, timed submission and BT motion commands now publish
  stamped CommandsNormalized. Mission geometry is mapped to body FLU in shared
  publication helpers. The original /master/commands PWM endpoint is retained
  for alt_master compatibility (see the update below).
- C++ `utils/control.hpp` mirrors Python PID sample intervals, bounds and conditional
  integration. The old control_utils implementation is retired behind an include shim.
- BT effort/gain ports, editor defaults and stage YAML use normalized values. Gate,
  docking and bucket settings have installed YAML and a generic mission launch.
  Docking's timed handover surge now uses HoldPosition.forward_effort.
- The separate named-actuator interface was removed at the user's request.
  Normalized body commands retain neutral spare RC channels.
- GUIDED targets resolve map/odom TF at the requested stamp into the configured local
  frame before ENU-to-NED conversion. Missing transforms reject the target.
- Shared registered-depth association, target pose/BT point/color outputs, and robust
  SVO point-cloud helpers are implemented. Bucket phase 1 consumes shared detections
  and calibrated pressure feedback, preserving vision steering without a GUI/inference copy.
- Mission topics are relative/remappable; retained ZED/SVO topics use canonical names.
  Camera acquisition limits have YAML; non-ROS image sources can publish canonical raw
  frames. ArUco dictionary/size and preview limits remain in per-camera configuration.
- Added local config/lint commands and fail-fast configuration gates before CI image
  builds/publication. Integration/build/HIL instructions are in STACK_REFACTOR.md.

## Deferred until the user requests validation

- Compile/regenerate changed ROS messages and all C++/Python consumers.
- Update local excluded tests for the six-axis command contracts and normalized
  mission settings; then run them only when requested.
- ROS smoke across FastDDS, CycloneDDS, Zenoh and Zenoh-client.
- Check actual camera-driver submodule output/remappings and RGB/depth registration
  on retained hardware; verify model labels and vehicle channel calibration.
- Retune PID gains: unit conversion is implemented, but corrected sample-time and
  anti-windup behavior changes controller dynamics.
- Simulator integration remains deferred; do not modify mira_sim.

## Configuration entry points

- `src/mira2_hardware/config/master.yaml`: master runtime, transport and
  compatibility settings.
- `src/mira2_hardware/config/command_calibration.yaml`: six-axis hardware
  calibration. Both master launches load these separate files together.
- `src/mira2_path_planning/config/{gate,docking,bucket,pipeline}.yaml`
- `src/mira2_control/config/`: PID tuner configuration.
- `src/mira2_vision/config/`: per-camera workers; bucket_front
  selects the retained bucket model. Set the correct target_class for model labels.
- `ros2 launch mira2_path_planning mission.launch.py mission:=gate` (or docking,
  bucket, pipeline). Start exactly one autonomy source at a time.

Tests remain local-only under `/tests` and excluded from commits. No tests, builds,
lint command or simulator runs were performed this continuation;
source inspection, Python syntax parsing, configuration validation and whitespace
review were used.

## User correction: alt_master is the operational baseline

The default `master` now preserves alt_master command ordering, refused-mode-change
behavior, eight direct legacy RC channels, telemetry sources/field semantics,
100 Hz stream requests, per-channel RC sends, and legacy toggle/endpoints.
`alt_master` executable/import/launch names are compatibility aliases.

A separate `async_master` uses bounded nonblocking receive polling. The default
master uses synchronous telemetry reads with a 0.1 s cycle deadline. Both inherit
one shared `master_core` implementation so the experimental variant changes IO
scheduling without duplicating vehicle behavior. Both retain the newer watchdog,
normalized-command validation and stamped TF guidance safeguards. Legacy command
expiry is disabled by default to preserve hold-last-command behavior; canonical
normalized commands still expire. No tests were run for this change.

Use `make master` or `make async-master`, or their respective ROS launches.
Do not run both against the vehicle simultaneously. See the control-master README
for exact compatibility semantics and deliberate differences from old alt_master.


## Latest: GUIDED pose tasks and VO integration

The current actions package replaces the old effort/detection leaves with one
NavigateToPose leaf. Shared controllers are Pixhawk GUIDED's PIDs; task XML
only overrides goal geometry, completion tolerances and timeout. Old custom BT
node names are removed. Dock/bucket examples are approach-and-settle templates,
not mechanical manipulation. See mira2_actions/README.md.

mira2_hardware now installs the mira_sim vision/goal/obstacle bridges plus a
GUIDED launch with RTAB-Map. This launch replaces async_master for navigation;
never run both command owners. Separate MAVLink router endpoints are required.
mira_sim itself was not edited. Its patched ArduSub OA firmware remains a
prerequisite. Perception target_pose now excludes non-requested classes.

Validation: ten transport-free bridge regression tests pass; Python/XML/YAML
parsing and both launch-description construction checks pass. C++ compilation
is blocked on missing behaviortree_cpp in the host's Humble installation;
the project targets Jazzy. No SITL or physical vehicle run performed.

Hardware Python package colcon build also passed after filtering launch directories from setup data files.


## Latest: camera-owned depth/navigation and mira2_vision rename

The perception package now lives at src/mira2_vision, with package/executable/
Python namespace/plugin entry points renamed to mira2_vision. The camera worker
owns depth decode, freshness, bounded RGB-D matching and depth-event dispatch.
The vision_bridge and obstacle_distance modules are in-process consumers here,
not hardware executables. RTAB-Map reads matched outputs from the VO consumer.
The goal bridge stays in hardware; guided.launch.py composes both packages.

simulation=false is the default. Hardware disables GT, spawn offsets, implicit
sim origin writes, Gazebo display calls and auto-arming. MAVLink links are
explicit on hardware; empty links leave ROS-only processing running. Simulation
must be selected explicitly, including for the clock. See mira2_vision/README.md.

Validation for this move: both mira2_vision and mira2_hardware colcon builds
passed; 19 regression checks passed, including shared decode, frame/time
matching, hardware simulation gates, and bridge delivery/hold behavior.
Hardware/simulation launch construction and syntax/YAML/XML checks passed.
Model asset hashes still match the original tracked assets. Tests used the
system NumPy ahead of user-site NumPy to match this host's ROS Humble OpenCV
ABI. No SITL or physical vehicle validation was performed.


## Latest: object input and one actions demo

Removed mira2_actions/config/navigation.yaml, all old task XMLs, the separate
Groot model XML and test launch. Only config/demo.xml remains as a BT example.
SelectObject selects a depth-localized YOLO Detection2D by class and passes the
actual detection through the blackboard to NavigateToPose. Navigation extracts
that bounding box's associated metric pose from mira2_vision, transforms it at
its timestamp, and sends a GUIDED goal facing the object. No anonymous pose-topic
or raw waypoint coordinate inputs remain. Defaults are on the action, not YAML.

Validation: two bbox-to-pose regression tests pass (each class retains its own
metric position; missing depth produces no navigation range). The single demo
XML and launch parse/construct successfully. C++ build still stops at missing
behaviortree_cpp in the host environment; no action runtime validation ran.

## Latest: submodules, dependency cleanup and sibling simulator integration

Initialized camera_driver and switched it to origin/new_arch at f3736a6.
.gitmodules tracks new_arch; the submodule pointer and removal of the unused
zed-ros2-interfaces gitlink are staged. Removed the unreferenced control_utils
package. custom_msgs/utils remain required; image conversion uses system ROS
cv_bridge. The camera_driver branch's ROS CMake option references absent
src/ros2_node.cpp; no physical-camera ROS runtime validation is claimed.

Updated ../mira_sim with an opt-in docker-compose.mira.yml overlay and build/
bring-up orchestration. Real Mira sources mount read-only; colcon artifacts live
in mira_sim/.mira-stack. Standalone VO testing remains available. The integrated
stack starts only the Gazebo sensor/TF bridge plus Mira vision/hardware nodes,
and gates the one-shot actions demo on a fresh localized target and EKF pose.
Defaults now use dock, matching the supplied model label; target_class is a
launch parameter bound to the demo's blackboard. There is still one demo XML.

From Mira: make sim-build, make sim, make sim-actions, make sim-down and make
sim-check. RUN_ACTIONS=0 starts navigation without the demo; HEADLESS=1 selects
server rendering. See ../mira_sim/docs/MIRA_STACK.md.

Validation: 27 local regression checks passed, plus CPU/GPU Compose mount
validation, Make dry runs, launch/script syntax and whitespace checks in both
repos. No integrated image build or live SITL run: the simulator base image is
not installed locally. The image supplies Jazzy BT/vision dependencies missing
from this host, so compiling the C++ action still requires make sim-build.

## Camera ROS fix and dedicated simulator services (2026-10-14)

Restored camera_driver/new_arch ROS YAML entry point, node-aware publisher factory,
static element registrations and consistent ament link signatures. Added a generic
pipeline launch, synthetic-camera YAML and live ROS smoke test. Built the node
against host Humble using development packages extracted under /tmp (no system
install), received RGB frames and verified missing-config failure and SIGINT exit.
Jazzy/container and physical-camera validation remain unperformed.

The sibling simulator overlay now adds dedicated mira-stack/mira-stack-gpu services
in profile mira, extending the original simulator services without modifying their
images. The runner maps GZ_SERVICE to the dedicated service. The optional image
reuses the Gazebo/ROS/RTAB-Map base and adds application dependencies. Both CPU
and GPU variants are supported (`mira-sim-stack:cpu` using CPU PyTorch and `best.onnx`;
`mira-sim-stack:cu128` using CUDA 12.8 PyTorch 2.7.1 / torchvision 0.22.1 and `best.pt`).
28 focused tests and Compose CPU/GPU inheritance checks pass.

## GPU YOLO stack and Docker investigation update (2026-10-15)

- Validated launch parameter propagation: `guided.launch.py` correctly propagates
  `yolo_device` (mapped to `device: GPU`) and `yolo_model` (mapped to `model_name: best.pt`)
  along with `config`, `source`, `start_x`, and `start_y` into the camera-owned
  `mira2_vision` perception node.
- Confirmed Compose merging: `docker compose config` merges image tags (`mira-sim-stack:cpu`
  and `mira-sim-stack:cu128`), build args (`TORCH_VARIANT=cpu` vs `cu128`), environment
  variables, and NVIDIA runtime/device reservations cleanly without conflicts.
- Confirmed PyTorch wheel availability: official `torch==2.7.1+cu128` and
  `torchvision==0.22.1+cu128` wheels exist for Python 3.12 (`cp312`) on x86_64, matching
  Ubuntu 24.04 / ROS Jazzy.
- Confirmed detector failure semantics: `Yolo` detector maps `device: GPU` to device `"0"`.
  When CUDA is unavailable, Ultralytics YOLO raises an explicit exception rather than
  silently falling back to CPU.
- Docker containerd diagnosis:
  * Missing content blobs that previously caused `docker image ls` to fail have been
    resolved by pulling `ghcr.io/davidnoronha1/mira_sim-gazebo:latest`. Both `docker image ls`
    and `docker images -a` now list all local images without error.
  * However, starting or building containers across all images (`ubuntu:22.04`, `golang:1.24`,
    `mira_sim-gazebo:latest`) currently fails inside containerd's snapshotter with:
    `failed to create snapshot: missing parent "moby/<id>/sha256:..." bucket: not found`.
  * Simultaneously, pulling layers fails with:
    `AlreadyExists: target snapshot "sha256:...": already exists`.
  * Root cause: containerd's metadata bolt database (`/mnt/shared/DAVID/CONTAINERD/io.containerd.metadata.v1.bolt/meta.db`)
    has desynchronized parent snapshot buckets in namespace `moby`.
  * Noninteractive `sudo` is unavailable on this host (`sudo: a password is required`),
    and `/run/containerd/containerd.sock` is restricted to root (`0660`).
    Resolving the containerd metadata desynchronization requires privileged containerd
    inspection (`ctr -n moby snapshots rm ...` or daemon metadata reset).

