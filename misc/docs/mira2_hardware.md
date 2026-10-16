# Mira Pixhawk controller

`pixhawk_controller` is the single Pixhawk command node. It owns one MAVLink
connection and a bounded, nonblocking receive loop. Capabilities are selected
with the read-only `plugins` ROS parameter at startup.

```bash
ros2 launch mira2_hardware pixhawk_controller.launch.py

# Deployment helper
make pixhawk-controller
```

The launch loads two configuration files from the package's `config/` directory:

| File | Settings |
|---|---|
| `master.yaml` | MAVLink transport, plugins and RC startup mode |
| `command_calibration.yaml` | Six body-FLU effort axes: minimum/neutral/maximum PWM, inversion, and deadband |

Calibration uses nested axis mappings. ROS expands them into the same
parameter names used by the master, such as `forward.minimum`.
The Pixhawk device launch argument overrides `master.yaml`.

Direct `ros2 run` uses parameter defaults unless files are passed explicitly.
For the same configuration and canonical topic namespace as the launch:

```bash
ros2 run mira2_hardware pixhawk_controller --ros-args -r __ns:=/mira \
  --params-file src/mira2_hardware/config/master.yaml \
  --params-file src/mira2_hardware/config/command_calibration.yaml
```

Run this example from the repository root, or use the installed package's config
paths.

## Receive behavior and safety changes

The master drains at most 64 immediately available packets
per timer callback and never waits for a telemetry packet or ACK. This is a
nonblocking receive experiment; MAVLink writes still use the transport's normal
send calls, so it is not a guarantee of nonblocking serial/socket writes.

It retains heartbeat-loss disarm, neutral shutdown, stale-field detection,
normalized-command freshness/range checks, and a disarmed-request latch after
watchdog failures. A vehicle found armed on connection is disarmed. Cached stale
pressure/battery fields and heading are NaN. Internal pressure is taken
from `SCALED_PRESSURE`.

## Interfaces (launch namespace `/mira`)

| Topic | Type | Direction |
|---|---|---|
| `command` | `CommandsNormalized` | Command input |
| `telemetry` | `Telemetry` | Pixhawk telemetry, including IMU and heading |

Telemetry uses Pixhawk's native conventions: IMU and magnetic-field vectors use
the `base_link_frd` body frame (forward-right-down), while attitude and heading
are referenced to NED. Battery and pressure are frame-independent.

Normalized commands contain six body-FLU efforts and require a fresh stamp and
`base_link` frame. The vehicle boundary encodes these into a single eight-channel
PWM override.

The `emergency_stop` (`SetBool`) service engages or clears the emergency stop.
The `arm` (`SetBool`) service requests arm (`true`) or disarm (`false`).
The `change_mode` (`custom_msgs/srv/ChangeMode`) service changes the flight mode
only while disarmed and the emergency state is clear.

## Controller plugins

| Plugin | Capability |
|---|---|
| `rc` | Calibrated RC overrides from normalized commands; arm/mode services |
| `guided` | Position/yaw goals, direct-line motion and hold; requires `navigation` |
| `navigation` | EKF pose/TF, raw attitude, pressure depth, IMU and optional paths |
| `telemetry` | Existing `custom_msgs/Telemetry` publisher from fresh cached packets |
| `external_navigation` | ROS odometry, flow and heading to MAVLink; EKF source selection and origin |
| `obstacle_distance` | Body-frame ROS LaserScan to MAVLink obstacle sectors |

The default selection is `[rc, telemetry]`. GUIDED launch selects
`[rc, navigation, guided, telemetry, external_navigation, obstacle_distance]`. Read-only feedback plugins can also run alone.
`rc` and `guided` may be enabled together. RC rejects normalized commands,
suspends its command watchdog, and releases existing overrides on entering
GUIDED. It sends no RC overrides while GUIDED is active. Leaving GUIDED clears
active goals and direct-line motion; returning to RC requires a new command.
Plugin selection happens at startup: stop the controller before changing plugins; runtime parameter changes cannot swap them.

```sh
ros2 launch mira2_hardware pixhawk_controller.launch.py plugins:=rc,telemetry
```

A custom plugin subclasses `ControllerPlugin` and is selected as
`your_package.module:YourPlugin`. Implement `on_connect()`, `on_message(msg)`,
`tick(now)` and `stop(reason)` as needed. Declare `message_rates` as a map of
MAVLink message IDs to Hz; the controller merges requests using the highest
rate and reasserts them every two seconds. Declare `requires` for dependencies.
Command plugins must yield to the reported autopilot mode. Invalid names and
missing dependencies are rejected before opening the connection. Dependencies
initialize and receive packets before dependents.

Plugins use the controller's ROS interfaces and shared `master` connection.
They must not read or close that connection. Arm/mode requests go through the
controller. The controller owns emergency stop, heartbeat-loss disarm, message
freshness, packet-source filtering and disarm/close on shutdown. Plugin state
is cleared on emergency stop and reconnect; old goals are not resumed.

## GUIDED navigation

GUIDED functionality formerly implemented by `guided_master` / `goal_bridge`
now lives in the `guided` and `navigation` plugins of `pixhawk_controller`.
`ros2 launch mira2_hardware guided.launch.py` starts only the hardware controller.
[mira2_vision navigation](mira2_vision.md) runs perception and odometry separately.
`make guided` starts both; `make guided-sim` starts both with simulation defaults.

Hardware mode is the default. `ARGS` supplies shared launch arguments;
`HARDWARE_ARGS` and `VISION_ARGS` supply package-specific overrides. Set `pixhawk_address` (or the legacy `goal_link`)
to connect the controller. Commands, external navigation and obstacles all share
that one connection. `vision_link` and `obstacle_link` are removed; vision has
no MAVLink readers. Never launch multiple controllers against the same vehicle.

```sh
make guided-sim HARDWARE_ARGS="start_x:=11 start_y:=-11.5 auto_arm:=true"
```

Use `simulation:=true` explicitly for mira_sim, rather than setting only
`use_sim_time`. It enables the simulator clock, endpoint defaults and spawn
translation. Outside simulation, the goal bridge disables automatic arming,
Gazebo marker commands and ground-truth subscriptions/publications. ROS EKF
path visualization remains available with `visualize:=true`. Direct invocation uses ROS parameters: set `pixhawk_address`,
`plugins`, and (for SITL) `simulation` and `use_sim_time` explicitly.

`/goal_pose` is a metric PoseStamped, transformed at its acquisition stamp into
ENU `odom`, then converted to MAVLink NED/FRD. Height is literal, including
z=0 (surface). Targets are sent three times to avoid restarting the autopilot's
avoidance path continuously; final heading follows arrival. The bridge publishes
`/vision/ekf_pose` and `odom -> base_link` TF. It exposes `/mira/guidance/hold`
(Trigger) and `/mira/emergency_stop` (SetBool). A hold never arms a disarmed
vehicle. Feedback loss clears goals and requests disarming.

All tasks use the autopilot's shared GUIDED controllers. Configure firmware
external-navigation fusion and proximity/avoidance independently; GUIDED alone
does not enable mira_sim's patched ArduSub avoidance. See the vision README for
RGB-D configuration, hardware origin setup and simulator migration instructions.

## Autopilot parameters

`config/ardusub/external_nav.parm` holds the ArduSub settings Mira's vision
navigation needs on the Pixhawk: no GPS, MAVLink external navigation, and two
EKF3 source sets that the `external_navigation` plugin switches between at runtime (set 1 SLAM,
set 2 optical flow). Load it once with a ground station. The simulator loads
the same file and then its own overrides (`mira_sim/config/ardupilot/`).

Check before use on the vehicle:

- `VISO_DELAY_MS` is the simulator's latency; measure the real one.
- `EK3_SRC2_YAW` is `1` (compass). Set `6` only together with the
  scene-specific floor-line heading (`optical_flow.floor_heading`, validated
  only in the simulated SAUVC pool).

The `navigation` plugin also publishes the raw autopilot IMU (`SCALED_IMU`, requested at
100 Hz) on `/mira/state/imu`; optical flow removes rotation with it.

