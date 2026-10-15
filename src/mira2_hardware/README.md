# Mira hardware master

`async_master` is the sole supported MAVLink master. It uses bounded,
nonblocking MAVLink receive polling.

```bash
ros2 launch mira2_hardware async_master.launch

# Deployment helper
make async-master
```

The launch loads two configuration files from the package's `config/` directory:

| File | Settings |
|---|---|
| `master.yaml` | MAVLink transport and startup mode |
| `command_calibration.yaml` | Six body-FLU effort axes: minimum/neutral/maximum PWM, inversion, and deadband |

Calibration uses nested axis mappings. ROS expands them into the same
parameter names used by the master, such as `forward.minimum`.
The Pixhawk device launch argument overrides `master.yaml`.

Direct `ros2 run` uses parameter defaults unless files are passed explicitly.
For the same configuration and canonical topic namespace as the launch:

```bash
ros2 run mira2_hardware async_master --ros-args -r __ns:=/mira \
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
pressure/battery fields are NaN and heading is -1. Internal pressure is taken
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

No tests, builds or vehicle runs were performed for this split.

## GUIDED navigation

`goal_bridge` remains the hardware-owned PoseStamped-to-Pixhawk bridge.
`ros2 launch mira2_hardware guided.launch.py` composes it with
[mira2_vision navigation](../mira2_vision/README.md), which now owns depth,
RTAB-Map inputs, external-navigation forwarding and obstacle-distance consumers.

Hardware mode is the default. Set distinct `goal_link`, `vision_link` and
`obstacle_link` MAVLink-router endpoints to enable forwarding. Empty hardware
links disable their transports; no goal bridge is launched without `goal_link`.
Never point independent readers at the same serial device or run `async_master`
alongside this navigation command owner.

```sh
ros2 launch mira2_hardware guided.launch.py simulation:=true \
  start_x:=11 start_y:=-11.5 auto_arm:=true
```

Use `simulation:=true` explicitly for mira_sim, rather than setting only
`use_sim_time`. It enables the simulator clock, endpoint defaults and spawn
translation. Outside simulation, the goal bridge disables automatic arming,
Gazebo marker commands and ground-truth subscriptions/publications. ROS EKF
path visualization remains available with `visualize:=true`. Direct invocation
requires `--mavlink` on hardware; `--simulation true` opts into SITL defaults.

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
