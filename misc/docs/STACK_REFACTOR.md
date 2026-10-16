# Stack refactor: implemented foundations

Simulator integration is deliberately deferred. No changes to `mira_sim` are
required for these features.

## Middleware selection

Node code must not set `RMW_IMPLEMENTATION`. Select a deployment profile before
starting any ROS processes:

```bash
python mira.py middleware-check --middleware cyclonedds
make teleop MIDDLEWARE=zenoh
python mira.py run --middleware fastdds
```

Profiles live in `config/middleware/profiles.toml`. Precedence is explicit
`--middleware`, then `MIRA_MIDDLEWARE`, then an existing `RMW_IMPLEMENTATION`,
then CycloneDDS. The GUI no longer forces a particular RMW. CycloneDDS is the
recommended GUI baseline because of its existing GStreamer/FastDDS startup
issue; GUI startup under other RMWs still needs separate validation.

For direct ROS commands, apply the same settings to the current shell:

```bash
eval "$(python mira.py middleware-env --middleware cyclonedds)"
```

Switching requires restarting existing nodes, and `ros2 daemon stop` to clear
the CLI daemon started under the previous middleware. Selection is not a live
switch. Both ends must use matching middleware and ROS domain settings.

Install a selected middleware with `python mira.py install-middleware
--middleware zenoh`. The Dockerfile includes CycloneDDS and Zenoh; FastDDS is
provided by the ROS base image. Docker CLI passthrough applies profile values
at exec time so persistent containers do not retain an old selection.
For direct `ros2` commands inside a container shell, apply `middleware-env`
there too; Compose environment passthrough alone does not resolve profiles.

### Zenoh onboard / ground-control topology

On the onboard machine, start a router in a dedicated terminal:

```bash
python mira.py zenoh-router --middleware zenoh
```

Use the `zenoh` profile for onboard nodes. For ground-control nodes connecting
directly to that router:

```bash
export MIRA_ZENOH_ENDPOINT=tcp/ROBOT_IP:7447
eval "$(python mira.py middleware-env --middleware zenoh-client)"
ros2 topic list
```

The client profile explicitly sets Zenoh client mode and the endpoint rather
than assuming peer discovery across hosts. Native `ZENOH_SESSION_CONFIG_URI`,
`ZENOH_ROUTER_CONFIG_URI`, `ZENOH_CONFIG_OVERRIDE`, `CYCLONEDDS_URI` and DDS
profile paths are preserved. Provide router authentication, access control and
TLS configuration before exposing the router outside a trusted network.
The router is not automatically started or stopped by application nodes.
Shared memory is not automatically enabled; configure and benchmark it with
the installed Zenoh version and container IPC topology first.

`rmw_zenoh_cpp` is not interoperable with `zenoh-bridge-ros2dds`. These profiles
select native ROS RMWs, not a DDS-to-Zenoh bridge.

Reference: https://github.com/ros2/rmw_zenoh/tree/jazzy

## Normalized control boundary

`utils.control` now provides ROS-independent saturation and per-channel PWM
calibration. NaN/infinity are rejected. Calibration supports asymmetric limits,
inversion and deadbands. Physical PWM settings belong at the vehicle boundary.

The single master now performs encoding directly: the intermediate normaliser
and duplicate master implementations are gone. It selects one source, rejects
stale/nonfinite commands, watches heartbeat/command loss nonblockingly, and
requires a fresh disarmed request before rearming after a failsafe. Calibration
is in `mira2_hardware/config/command_calibration.yaml`; runtime, transport,
watchdogs and compatibility settings are in
`mira2_hardware/config/master.yaml`.
Both master launches load the two files together. Calibration groups settings
under each axis; ROS expands them to names such as `forward.minimum`.

All mission, joystick and GUI requests now use stamped normalized commands.
The master retains the original `/master/commands` PWM endpoint for alt_master
compatibility; normalized inputs remain the preferred interface. The separate
actuator message and named actuator topics have been removed.
GUIDED targets are transformed at their acquisition stamp into `local_frame`
(default `odom`) before ENU-to-NED conversion. A missing TF rejects the target.

## Perception and tuning consolidation

The mira2_vision package runs explicitly selected detector and consumer plugins
in one process per camera. YOLO results feed object/depth consumers; ArUco results
feed marker/board consumers; HSV pipeline detection is independent. Plugins share
one decoded frame and acquisition header; only optional previews copy the frame. See
`misc/docs/mira2_vision.md` for bringup and configuration.

The PID GUI is the only axis-tuning application and consumes that shared output.
It covers lateral, forward, pressure/depth, and heading. ArUco is optional:
automatic visual feedback prefers a fresh ArUco marker, falling back to shared
YOLO boxes with a separate PID profile. Effective-source transitions disarm.
Explicit marker, YOLO-box and registered-depth target-pose modes remain available;
pressure/depth and heading use telemetry alone. Its conventional PID
is in the existing `utils.control` module. Old PWM gains do not transfer; zero
gains/disarmed are the defaults. C++ missions now use the same sample-interval and anti-windup contract in
`utils/control.hpp`. Existing gain units were scaled to normalized effort;
the corrected derivative/integral semantics require retuning before hardware use.
The GUI no longer contains camera acquisition, ArUco inference, or a GStreamer
pipeline. Its launch does not start another perception worker.

## Validation in a ROS Jazzy container

Validation scripts are local-only and excluded in `.git/info/exclude` at the
user's request; they are not included in commits. Validation is performed in an
isolated Jazzy container, without starting the simulator or using hardware.

The ROS smoke suite exercises normalized conversion, invalid commands, input
timeout, best-effort images, services, late-subscriber static TF and MCAP
record/readback under FastDDS, CycloneDDS, Zenoh, and a loopback Zenoh client
connected to a router. It does not operate vehicle
hardware, start the simulator, or benchmark network performance. Synthetic
ArUco and offscreen four-axis GUI checks are included. It manages only the
Zenoh router process it starts itself.

## Implementation status and deferred validation

Mission commands, BT effort ports, stage YAML and shared PID utilities are migrated.
`control_utils` is a compatibility include; its previous PID implementation is retired.
The legacy Commands message is accepted at `/master/commands` when compatibility
is enabled. Legacy RC release values are preserved at that boundary.

Registered RGB/depth association runs inside the existing perception worker. Enable
`depth_topic` only for depth registered to the RGB optical frame and dimensions.
16UC1 depth is millimeters; 32FC1 depth is meters. Skewed or invalid samples produce
no target. Target poses, inverse-distance BT points and bucket color share one selection.
SVO tools share robust point-cloud geometry with this utility layer. SDK-native ZED
ObjectsStamped remains an independently remappable input to Approach3DBBox.

No tests or builds were run for this continuation, at the user's request. Historical
validation results above predate these changes. Next validation must cover message
regeneration, C++ compilation, changed PID dynamics, TF offsets,
registered-depth timing and all four middleware profiles. `mira_sim` remains untouched.

## Colcon / Docker integration

Build after sourcing ROS Jazzy using `colcon build --symlink-install` (or `make build`).
Rebuild custom_msgs and its consumers together because CommandsNormalized now has six
body-effort fields. Source `install/setup.bash`.
Use the existing Docker/deployment middleware profile selection; nodes never select RMW.

`ros2 launch mira2_path_planning mission.launch.py mission:=gate` loads gate YAML;
`docking`, `bucket` and `pipeline` select their respective installed settings.
BT graphs remain XML configuration with normalized effort/gain ports. Timed hold now
accepts `forward_effort` for the docking handover surge. Mission source geometry uses
right-positive lateral/clockwise heading; the publish helper maps it to FLU.

Use `make config-check` for fail-fast YAML/TOML/BT/calibration checks and `make lint`
for source lint (requires PyYAML and Ruff respectively). These commands were added
but not executed in this session. Image build/publish workflows run configuration
validation before packaging; no images were published by this continuation.

For future HIL, first establish pressure calibration and hardware axis inversion. Supply map-to-odom TF from localization; do not assume identity.
Camera driver submodules/hardware must be checked for actual topic publication when
integration resumes. Simulator integration stays deferred until explicitly requested.

## Operational master baseline

The user's operational baseline is alt_master. `master` now preserves its arm/mode
ordering, telemetry field semantics and legacy endpoints. `pixhawk_controller` now owns bounded receive polling, with RC and GUIDED
behavior selected through controller plugins; see `misc/docs/mira2_hardware.md`.
The synchronous implementation uses bounded waits; neither variant reproduces the
old indefinite waits. Read `misc/docs/mira2_hardware.md` before comparing
variants; it lists retained safety changes and exact telemetry semantics.
