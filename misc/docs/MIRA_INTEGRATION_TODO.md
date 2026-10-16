# Mira integration context and pending work

Updated: 2026-10-15. GPU simulator image and Docker repair are complete; end-to-end sim run is the next step.

## User intent

- Simplify mira2_actions around Pixhawk GUIDED navigation and shared controllers, with overrides where needed. Avoid per-task motion/control duplication.
- Navigate to an already-visible YOLO object: bounding box + registered depth -> object pose -> goal pose -> Pixhawk. Keep one demo BT XML.
- Rename vision_unified to mira2_vision. Vision owns depth reception and exposes depth to independent VO and obstacle consumers. Hardware owns the goal-to-Pixhawk bridge. Simulator-only behavior must be disabled outside explicit simulation mode.
- Integrate ../mira_sim's VO testing workflow with the real Mira packages and provide Make/Compose targets.
- Initialize camera_driver on new_arch, fix its missing ROS entry point, and push that fix.
- Enable GPU YOLO in the integrated simulator container and repair Docker's missing image blobs: done 2026-10-15 (see below). A live `make sim` run has not been done yet.

## Workspace and preservation

- Mira: /mnt/shared/DAVID/PERSONAL/mira
- Simulator: /mnt/shared/DAVID/PERSONAL/mira_sim
- Both contain extensive uncommitted changes from earlier work. Do not reset, clean, or blanket-commit them.
- Sibling simulator writes, Git metadata writes and Docker/network operations require tool escalation in this environment. sudo -n reports that a password is required, even in escalated commands.
- Only the camera submodule fix has been committed and pushed. Parent Mira and simulator integration edits remain local.

## Implemented before the latest GPU request

### Actions and vision

- mira2_actions has one config/demo.xml: SelectObject -> NavigateToPose.
- SelectObject selects a fresh, localized Detection2D matching target_class from mira/perception/front/objects. Default class is dock. User asked what this helper was; it was explained but remains present.
- NavigateToPose accepts the actual Detection2D, transforms its depth-derived pose into odom, applies stand-off and sends /mira/guidance/goal. It monitors /mira/state/pose and requests /mira/guidance/hold on cancellation/failure. Old task XMLs and custom motion code were removed.
- mira2_vision owns registered depth decoding/matching and plugin depth events. depth_targets enriches YOLO detections with metric poses; vision_bridge feeds RTAB-Map and MAVLink; obstacle_distance independently consumes depth.
- Hardware goal bridge remains in mira2_hardware. Simulation flags gate ground truth, spawn offsets, auto-arm and other simulator behavior. Hardware MAVLink transports require explicit configuration.
- Shared navigation math is in src/dependencies/utils/utils/navigation.py.
- control_utils and unused zed interfaces removed; custom_msgs/utils remain; use system cv_bridge.

### Camera fix: complete and pushed

- camera_driver is on new_arch, tracking origin/new_arch.
- Pushed commit: 51f3691, "Restore ROS YAML pipeline node and retain element factories".
- Remote: git@github.com:davidnoronha1/camera_driver.git.
- Parent submodule pointer was staged to this commit; parent repository was not committed/pushed.
- Added src/ros2_node.cpp: YAML pipeline_config parameter, node-aware ROS2Publisher factory, ROS executor + nonblocking GStreamer tick loop, graceful shutdown.
- Fixed static factory retention and mixed ament/CMake link signatures. Added dependency declarations, launch/pipeline.launch.py, config/ros2_test.yaml and test/ros2_smoke.py.
- Built successfully against host ROS Humble. Live smoke test received RGB frames, checked invalid config failure and clean SIGINT shutdown.
- Temporary build: /tmp/camera-ros-build; dependencies extracted under /tmp/camera-build-deps/sysroot. No system package installation succeeded or was needed.
- Camera Jazzy and physical-device validation remain unperformed. Existing legacy launch/test_video_file.launch.py was not migrated to the new YAML API.

### Simulator integration

New sibling files:

- docker-compose.mira.yml
- docker/mira-stack.Dockerfile
- docker/mira-stack.py
- docker/mira-stack-runner.sh
- docker/wait-mira-target.py
- docs/MIRA_STACK.md

Also edited sibling Makefile/README/.gitignore and parent Makefile/README.

- Dedicated Compose services mira-stack and mira-stack-gpu, profile mira, extend original simulator services through explicit file: docker-compose.yml inheritance. Original simulator services remain unchanged by the overlay.
- Mira sources mounted read-only at /mira; build/install/logs in sibling .mira-stack mounted at /mira_build.
- Selected colcon packages: custom_msgs, utils, mira2_vision, mira2_hardware, mira2_actions. Physical camera packages excluded for simulation.
- Make targets in mira_sim: mira-stack-build, bringup-mira, mira-actions, mira-stack-down, mira-stack-check.
- Parent wrappers: sim-build, sim, sim-actions, sim-down, sim-check.
- Runner maps GZ_SERVICE=mira_sim to mira-stack, and mira-sim-gpu to mira-stack-gpu. Existing Makefile selects NVIDIA rendering when available.
- Bringup uses tmux windows for SITL, Gazebo, sensor/TF bridge, real Mira GUIDED stack and optional actions demo. Only the old sensor bridge is reused; duplicate sim VO/goal/obstacle scripts are not launched.
- Demo waits for fresh localized target and EKF feedback before starting. TARGET_CLASS=dock; ACTION_WAIT=120. RUN_ACTIONS=0 disables demo. Simulation AUTO_ARM=true can be overridden false.
- Last completed checks BEFORE GPU edits: 28 focused tests passed (7 sim stack, 2 object pose, 8 depth, 11 guided navigation). CPU/GPU Compose inheritance, shell/Python syntax and make sim-check passed. Full Jazzy container build/run has never completed.

## GPU edits validation: complete

Host nvidia-smi reports NVIDIA RTX PRO 6000 Blackwell, driver 570.211.01, CUDA capability 12.8, approximately 96 GB VRAM. Other host GPU workloads were running; do not disrupt them.

Chosen approach: CUDA-enabled PyTorch with existing best.pt, avoiding a separate GPU ONNX/TensorRT runtime. Official references consulted:

- https://pytorch.org/get-started/previous-versions/
- https://pytorch.org/blog/pytorch-2-7/

Files edited:

1. ../mira_sim/docker/mira-stack.Dockerfile
   - ARG TORCH_VARIANT=cu128.
   - Install torch==2.7.1 and torchvision==0.22.1 from the selected cpu/cu128 wheel index.
   - Existing pip step still installs numpy<2, opencv-python<4.12, ultralytics and CPU onnxruntime. GPU mode uses .pt, so GPU ONNX is unnecessary.
   - Reuses simulator base with ROS/RTAB-Map. CUDA libraries come from PyTorch wheels, not an added CUDA base image.
2. ../mira_sim/docker-compose.mira.yml
   - CPU service image mira-sim-stack:cpu, build TORCH_VARIANT=cpu, env YOLO_DEVICE=CPU and YOLO_MODEL=best.onnx.
   - GPU service image mira-sim-stack:cu128, build TORCH_VARIANT=cu128, env YOLO_DEVICE=GPU and YOLO_MODEL=best.pt.
   - Distinct tags prevent CPU/GPU builds overwriting each other.
3. ../mira_sim/docker/mira-stack-runner.sh
   - Passes yolo_device and yolo_model launch arguments from those environment variables.
4. src/mira2_vision/launch/navigation.launch.py
   - Declares optional yolo_device/yolo_model args and applies nonempty values as device/model_name parameter overrides.
5. ../mira_sim/docs/MIRA_STACK.md
   - Updated GPU/CPU behavior and GPU build/run commands.

Completed checks:

- [x] **Parameter propagation through `guided.launch.py` into `navigation.launch.py`**:
  Validated through ROS 2 Launch execution. The camera-owned `perception` node (`mira2_vision`)
  successfully receives and resolves all overrides:
  `device: GPU`, `model_name: best.pt`, `vo.source: vo`, `vo.start_x: 7.0`, `vo.start_y: -10.0`,
  and `--params-file /mira/src/mira2_vision/config/guided_front.yaml`.
- [x] **Compose merging**:
  Validated via `docker compose config`. Tag separation (`mira-sim-stack:cpu` vs `mira-sim-stack:cu128`),
  build args (`TORCH_VARIANT`), environment variables (`YOLO_DEVICE: GPU`, `YOLO_MODEL: best.pt`),
  and NVIDIA device reservations (`capabilities: [gpu, compute, utility, graphics, video, display]`,
  `driver: nvidia`, `count: -1`, `runtime: nvidia`) merge cleanly without conflicts.
- [x] **PyTorch wheel availability**:
  Verified official index `https://download.pytorch.org/whl/cu128/`.
  Confirmed that `torch==2.7.1+cu128` and `torchvision==0.22.1+cu128` have prebuilt wheels for
  Python 3.12 (`cp312-manylinux_2_28_x86_64`), matching the Ubuntu 24.04 / ROS Jazzy container.
- [x] **Detector GPU failure semantics**:
  Confirmed in `src/mira2_vision/mira2_vision/detectors/yolo.py`. When `device: GPU` is specified,
  it maps to device `"0"`. In Ultralytics YOLO, passing a specific GPU device index when CUDA
  is unavailable raises an explicit exception, preventing silent unintended fallback to CPU.
- [x] **Documentation check**:
  Checked all workspace markdown files. Stale CPU-only wording in `CONTINUE.md` was updated.
- [x] **Focused tests & syntax**:
  All 28 focused regression tests passed (7 sim stack, 2 object pose, 8 depth, 11 guided navigation).
  `git diff --check` reported zero whitespace errors. All Python files compile cleanly.

## Docker repair and GPU image build: complete (2026-10-15)

Docker 29.7.1, containerd v2.2.6, containerd image store, root `/mnt/shared/DAVID/DOCKER`.

What was wrong and how it was fixed:

1. Containerd snapshot errors (`missing parent ... bucket: not found`) were gone by 2026-10-15
   (the Docker root had been moved/reset); `docker run` worked again.
2. Dockerd's embedded BuildKit still had stale state: builds failed with
   `parent snapshot ... does not exist` and `lease "ony19103bglv0hu3pyjg0f726": not found`,
   even after `docker builder prune -a -f` and fresh `docker pull`s. Fixed by the user with sudo:
   ```sh
   sudo systemctl stop docker docker.socket
   sudo mv /mnt/shared/DAVID/DOCKER/buildkit /mnt/shared/DAVID/DOCKER/buildkit.broken-2026-10-15
   sudo systemctl start docker
   ```
   `buildkit.broken-2026-10-15` can be deleted once builds keep working.
3. The published `ghcr.io/davidnoronha1/mira_sim-gazebo:latest` (built 2026-10-06) predates the
   2026-10-07 `gazebo.Dockerfile` change that added `/opt/mavenv`. `docker/mira-stack.Dockerfile`
   now installs `python3-venv` and creates `/opt/mavenv` (+ pymavlink) when it is missing.
   Republishing the gazebo image would make this a no-op.
4. Compose runs the stack as `${UID:-1000}:${GID:-1000}`; bash does not export UID and GID is unset,
   so colcon could not write `/mira_build` (owned by 1001:1002). `docker/mira-stack.py` now sets
   UID/GID from the invoking user.
5. `custom_msgs/CMakeLists.txt` still listed the deleted `msg/Commands.msg`; entry removed
   (no code references the old message).

Verified:

- [x] `make sim-build GZ_SERVICE=mira-sim-gpu` succeeds: image `mira-sim-stack:cu128` built, colcon
  builds custom_msgs, utils, mira2_vision, mira2_actions, mira2_hardware (5/5).
- [x] Inside `mira-stack-gpu`: torch 2.7.1+cu128, `torch.cuda.is_available()` is True on the
  RTX PRO 6000 Blackwell; ultralytics imports; all five packages visible to `ros2 pkg list`.
- [x] 28 focused host tests still pass.
- Note: pip installs protobuf 7.36.2 into `/opt/mavenv` (system has 4.21.12). The image has no `gz`
  Python bindings, so nothing known breaks, but keep this in mind if protobuf errors appear.

## Remaining work

- [ ] Run the integrated stack end to end:
  `make sim GZ_SERVICE=mira-sim-gpu HEADLESS=1 RUN_ACTIONS=0`, then with actions enabled.
  Confirm YOLO runs on GPU (`device: GPU`, `best.pt`) and the demo reaches the dock target.
- [ ] Build/verify the CPU variant: `make sim-build` (image `mira-sim-stack:cpu`).
- [ ] Camera driver: Jazzy build and physical-device validation; migrate
  `launch/test_video_file.launch.py` to the YAML API.
- [ ] Commit the parent Mira and mira_sim integration changes (nothing beyond the camera submodule is
  committed). Make sure the untracked `utils` files (`include/utils/control.hpp`,
  `include/utils/parameters.hpp`, `utils/control.py`, `utils/depth.py`, `utils/navigation.py`) are
  added, since the deleted `control_utils` PID now lives there.
- [ ] Optionally republish `mira_sim-gazebo` so the published base includes `/opt/mavenv`.

## Useful validation commands

From Mira:

```sh
make sim-check
make sim-build GZ_SERVICE=mira-sim-gpu
make sim GZ_SERVICE=mira-sim-gpu HEADLESS=1 RUN_ACTIONS=0
```

Focused host tests (Humble installed; apt NumPy must precede incompatible user NumPy):

```sh
. /opt/ros/humble/setup.bash
PYTHONPATH=/usr/lib/python3/dist-packages:src/dependencies/utils:$PYTHONPATH python3 -m unittest discover -s tests -p test_sim_stack.py
PYTHONPATH=/usr/lib/python3/dist-packages:src/dependencies/utils:$PYTHONPATH python3 -m unittest discover -s tests -p test_object_pose.py
PYTHONPATH=/usr/lib/python3/dist-packages:src/dependencies/utils:$PYTHONPATH python3 -m unittest discover -s tests -p test_vision_depth.py
PYTHONPATH=/usr/lib/python3/dist-packages:src/dependencies/utils:$PYTHONPATH python3 -m unittest discover -s tests -p test_guided_navigation.py
```

Compose inspection does not need a working daemon:

```sh
docker compose -f ../mira_sim/docker-compose.yml -f ../mira_sim/docker-compose.mira.yml --profile mira config --format json
```

