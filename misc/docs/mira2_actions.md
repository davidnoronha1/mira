# Navigate to a detected object

The single demo is `config/demo.xml`:

```xml
<Sequence>
  <SelectObject class_id="{target_class}" object="{target}"/>
  <NavigateToPose object="{target}" stand_off="0.5"/>
</Sequence>
```

`SelectObject` selects the highest-confidence fresh, localized YOLO bounding box
of the requested class from `mira/perception/front/objects`. It passes the actual
`vision_msgs/Detection2D` through the blackboard: box, class, confidence, camera
frame, acquisition stamp and the associated metric pose. It assumes the object
is visible and allows one second to receive a sample; it does not search or move.

`mira2_vision` must run `yolo` with `depth_targets` and `objects` consumers.
`depth_targets` samples registered depth inside each bounding box and projects
its center using CameraInfo, enriching that same detection's hypothesis pose.
This runs for every class, independent of the legacy `target_class`/`target_pose`
output. A 2D box without valid matched depth cannot become a navigation goal.

`NavigateToPose` takes that **object**, extracts its associated pose, transforms
it into `odom` at the detection timestamp, and sends the resulting goal to the
hardware GUIDED bridge. It faces the object; YOLO boxes do not supply orientation.
The goal is latched so other detections cannot replace it during travel.
`stand_off` defaults to zero (navigate to the object position); the demo stops
0.5 m short along the 3D approach ray.

There are no raw waypoint coordinates, anonymous target-pose inputs, separate
navigation YAML or task-specific PID settings. Pixhawk owns the shared GUIDED
controllers. The action defaults to a 0.3 m arrival radius and 60 s timeout,
overridable with `tolerance` and `timeout`. Arrival must hold for one second with
heading within 0.15 radians. Feedback must be no more than 0.5 s old; timeout,
lost feedback or cancellation request a hold.

Run the vision/hardware stack first, with registered RGB-D and camera-to-base TF:

```sh
# In one terminal (starts hardware and vision):
make guided-sim
# In another terminal:
ros2 launch mira2_actions mira2_actions_launch.py use_sim_time:=true
```

The demo defaults to `dock`, matching the supplied model labels. Override with
`target_class:=your_label` when using another model. Set `objects_topic` on
`SelectObject` for another camera. Simulation requires explicit arming or the
hardware launch's simulator-only `auto_arm:=true`. The demo executes once and
exits; it does not claim to implement a complete docking or retrieval mission.

The simulator runner uses `gate` and `config/gate.xml` by default with Gazebo
boxes. Set `ACTION_TREE=demo TARGET_CLASS=flare` for the simple object approach.
The executable waits for simulated time
and `odom <- base_link` TF before acquiring its short-lived target. SIGINT and
SIGTERM halt the running tree while ROS is still active so navigation can request
the hold service before exiting.

Validated on 2026-10-15 in the sibling simulator: Gazebo boxes → registered
depth → this tree → GUIDED → ArduSub EKF, with cuVSLAM 17.0.1 RGB-D and no
RTAB-Map node. The `flare` approach sent `(7.285, -3.573, -1.364)` in `odom`,
travelled about 6.5 m and returned SUCCESS after position and heading settled.
This validates the supplied two-leaf demo, not the legacy path-planning missions.

`gate.xml` now starts with a configurable `ScanArea` task, then acquires the
gate and executes `TraverseGate`. Each task owns its behavior independently;
recording remains external. The earlier stop-and-reacquire implementation has
been replaced with continuous visual approach updates.

Earlier verified gate/flare run on 2026-10-15: actual truth crossed the gate plane at
`(6.580, 0, -1.350)` with the gate centered at `(7, 0)` and posts at `x=6.25/7.75`.
Minimum vehicle-center distance to the flare center was 1.121 m. The two-stage
tree returned SUCCESS; the ArduPilot log contained 48 OABR samples, 11 active,
with `OA_TYPE=1` and `PRX1_TYPE=2`. MAVProxy also recorded actual depth-derived
OBSTACLE_DISTANCE packets. The corrected video is `recordings/gate_avoidance.mp4`.

Contract tests use synthetic feedback in an isolated domain and command no
vehicle. They cover success, stand-off, latched goals, missing targets,
unlocalized boxes, timeout, stale feedback, cancellation with a hold request,
two-stage generic navigation, continuous gate switching with bounded mapped
gate updates and rejection of foreground depth jumps, scan radius/sweep/return, invalid scan parameters, scan and gate
cancellation/hold, and conflicting-offset rejection:

```sh
# Inside mira-stack-gpu, with ROS and /mira_build/install/setup.bash sourced:
ROS_DOMAIN_ID=79 python3 /workspace/tools/test_actions.py
```

The current gate routine runs three separate BT tasks:

```xml
<Sequence>
  <ScanArea radius="{scan_radius}" sweep_degrees="{scan_sweep}"
            step_degrees="{scan_step}" depth="{scan_depth}"
            face_class="{target_class}" timeout="150"/>
  <SelectObject class_id="{target_class}" object="{target}"/>
  <TraverseGate object="{target}" class_id="{target_class}"
                through_distance="1.5" timeout="90"/>
</Sequence>
```

`ScanArea` first descends vertically at the starting XY position, then
surveys about that actual starting EKF pose and returns to its survey origin.
It refuses scan depths above the configured `minimum_depth` (default 0.6 m),
so sideways movement starts below the surface starting frame. The simulator
frame has physical collisions matching its visual bars. `radius` controls the circle in metres; zero rotates in place.
`sweep_degrees` controls angular coverage (0 skips the survey after the clearance descent), and `step_degrees`
controls viewpoint spacing (up to 90 degrees). `depth` is metres below the
surface. It waits for position and heading at each viewpoint so sensing has
stable coverage. Its optional `face_class` records sensor detections during
the survey and faces that class after returning to spawn. It uses no simulator
object coordinates. Invalid parameters, stale feedback, timeout and
cancellation fail or request hold. The task can be reused without `face_class`.

`TraverseGate` refines depth-derived gate geometry while approaching.
Corrections are bounded around the first observed stationary gate so a
foreground occluder or a single visible post cannot move the target plane.
It retains that mapped geometry within 4 m, before posts become cropped. Once lateral position, depth and heading align within the
crossing corridor, it latches the short crossing goal while still moving.
There is no stand-off arrival or one-second settling pause in front of the
gate. Only completion beyond the plane settles. It requires a fresh initial gate observation and stops on stale EKF
feedback or cancellation. Near the gate it navigates using the mapped landmark,
continuous cuVSLAM odometry and ArduPilot avoidance. A frontal gate is assumed; arbitrary gate-plane orientation is
not inferred from a box.

```sh
ros2 launch mira2_actions gate.launch.py use_sim_time:=true \
  scan_radius:=0.75 scan_sweep:=360.0 scan_step:=15.0 scan_depth:=1.0
```

A fresh integrated bringup starts the routine automatically:

```sh
make sim HEADLESS=1 SCAN_RADIUS=0.75 SCAN_SWEEP=360 SCAN_STEP=15 SCAN_DEPTH=1.0
```

The GPU simulator selects cuVSLAM only. Its native SLAM runs alongside its
continuous odometry; loop-closure poses are published separately from the
odometry consumed by the vehicle. No recording logic belongs to the BT tasks.
Do not launch a second action owner while the routine is running.

For an external demo, start a fresh simulator with `RUN_ACTIONS=0`, then run
inside the GPU container with ROS and the Mira install sourced:

```sh
GZ_PARTITION=mira_stack python3 /workspace/tools/record_gate_slam.py \
  /tmp/scan_gate_demo --run-gate --scan-radius .75 --scan-sweep 360
```

The tool records an actual Gazebo third-person camera, onboard RGB and
registered depth from before the survey through the gate crossing. It then
replays only cuVSLAM, saves its native SLAM map, and projects registered depth
using the optimized cuVSLAM poses for a colored 3D reconstruction. This dense
visualization is not a second SLAM backend. It includes starting-area frames.
No RTAB-Map process is launched. The map panel is explicitly labelled as a
reconstruction from replay. `--replay-only` regenerates the map/video from the
saved trusted local `sequence.pkl` without starting a vehicle action.

Outputs include `gate_demo.mp4`, `gate_demo.jpg`, `depth.png`,
`third_person.png`, `map.png`, `map.ply`, `cuvslam_map/`, `accuracy.json`,
trajectories, the raw RGB-D sequence and logs. Ground truth is used only for
verification and external camera framing.

The simulator uses cuVSLAM's depth-constrained Multisensor solver with one
RGB-D camera, which tracked the rotation sequence accurately where legacy RGBD
drifted. Hardware retains RGBD mode unless configured otherwise through
`vo.cuvslam_tracking_mode`; calibration and rectification must match the mode.
Native cuVSLAM and FFmpeg are needed for the demo; RTAB-Map is not needed for this workflow.

The demo verifies starting-frame clearance against an enclosing 0.45 m hull
sphere, the 0.04 m frame bars, and an allowance for motion between truth samples.
`starting_ring_min_clearance_m` and `starting_ring_cleared_conservative_hull`
are saved alongside gate-aperture verification. These simulator coordinates
are used only by the external verification tool, never by the BT tasks.

Current validation (2026-10-15): all 16 isolated behavior contracts pass.
A live startup survey at radius 0.75 m, sweep 360 degrees and depth 1.0 m
returned within 0.096 m of its start, with a maximum measured radius of 0.800 m.
The physical starting frame cleared the conservative 0.45 m hull check with
positive clearance. Its scan-only video and native cuVSLAM map are under
`recordings/slam_validation/scan_area/`. That recording did not complete the
gate crossing and is explicitly labelled as scan-only evidence.

The latest mapped-gate routine still requires a complete live end-to-end
validation. Subsequent runs stopped safely on missing feedback while the host
reported load averages around 235 on 24 CPUs. The validation simulator was
stopped after saving evidence. Do not treat a BT arrival log or a successful
`ros2 launch` exit alone as proof of a gate crossing; the external verifier
checks the actual aperture and starting-frame clearance.
