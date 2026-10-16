# Latest: flow-only gate pass works (2026-10-16)

POSE_SOURCE=optical_flow now passes the gate: 3/3 fresh runs crossed the
aperture (x 6.99/6.88/6.69), EKF XY error <= 0.32 m over ~12.5 m, yaw <= 1.2 deg
(Gazebo truth for verification only). Not committed. Sim tools moved from
mira/util/sim to mira_sim/tools (in-container /workspace/tools).

Three causes, fixed:
1. Heading drift: floor-line heading. optical_flow projects strong floor edges
   onto the floor with EKF attitude, takes the dominant direction mod 90 deg,
   compares with line_heading (sim 0; vehicle: from start pose) and publishes a
   corrected heading on /mira/perception/bottom/floor_heading. vision_bridge
   sends it as VISION_POSITION_ESTIMATE yaw (EK3_SRC2_YAW 6 in
   ardusub_vision.parm - the real Pixhawk needs this too). Offline 1.8 deg RMS.
2. Dummy position: that message must carry a position; (0,0) was fused while
   source set 1 (ExtNav XY) was active at boot -> tens of metres jumps. Now the
   EKF's own LOCAL_POSITION_NED is echoed.
3. Boot divergence (the old "wrong gyro bias"): flow derotated with EKF
   attitude differences, so an EKF gyro-bias error became fake flow velocity,
   fused into a bigger error (2/3 boots: tilt 20 deg, 30 m/s while still; also
   happened with the heading off). guided_master now publishes raw SCALED_IMU at
   100 Hz on /mira/state/imu and flow integrates the gyro between frames.
   3/3 boots settle in ~20 s.
Also: features on a 4x5 grid over a CLAHE image (spreads off the dark stripes;
same accuracy offline, more frames measured); attitude slerp-interpolated to
frame time; wait-mira-target.py requires a settled EKF (still 3 s); sim
LOG_DISARMED 1.
Run 1 (before fix 3) failed after the flare detour left it 0.8 m off-axis and a
close one-post gate sighting fed the latch - not seen since; watch for it.
Untested: SLAM+flow mode with gyro derotation; the per-message flow-vs-truth
comparison in diagnose telemetry looks frame/timing-skewed (EKF path is 2 %).
Topic naming: guided_master still publishes legacy /vision/ekf_attitude,
/vision/pressure_depth (and flow on /vision/flow_velocity); candidates to move
under /mira/state and /mira/perception.

# Latest: flow-only diagnosis, SLAM+flow demo video (2026-10-15, latest)

Recorded demo: recordings/slam_validation/gate_pass_slam_flow/ and
~/Videos/mira_gate_pass_slam_flow.mp4 (42.5 s, 3x2 panels incl. bottom-camera
flow tracks and flow-vs-truth plot). SLAM_OPTICAL_FLOW_ENABLED=1: crossing
x=6.968, aperture PASS, ring +0.185 m, scan drift 0.084 m, flare 0.71 m.
Flow reads ~10 % high at ~0.9 m/s cruise (see plot); not yet investigated.

Changes since the section below:
- Substitution test on recorded flow-only data (util: scratch scripts, not in
  repo): live attitude fine (0.031 vs 0.020 m/s RMS with truth); EKF depth was
  the error (0.27 m scatter, offset differs by mode). Flow height now from raw
  pressure: guided_master publishes /vision/pressure_depth (SCALED_PRESSURE,
  zeroed by BARO1_GND_PRESS), flow low-passes it (0.5 s). Floor 2.32 confirmed
  (raw pressure offset 0.119 m).
- Flow skipped while |vertical speed| > 0.15 m/s (descent zoom leak caused
  0.56 m drift in the ring). MIN_SIGMA 0.05 m/s.
- vision_bridge: with SLAM, flow goes as VISION_SPEED_ESTIMATE (horizontal
  ExtNav velocity; needs VISO_QUAL_MIN 0 because ArduSub passes quality 0 for
  it -> set in ardusub_vision.parm, also needed on the real Pixhawk). Body
  odometry (VISION_POSITION_DELTA) only for flow-only; its vertical is a 1 s
  line fit to pressure (2-sample diff corrupted EKF depth). With body odometry
  in SLAM mode EKF depth deviated up to 0.5 m; with speed estimate std 0.06 m.
- TraverseGate: gate estimate gated against the CURRENT estimate with short
  memory (weight >= 0.2) instead of the first sighting; geometry latches only
  from a fresh (<1 s) sighting within 4 m and crossing requires the latch.
  Result: flow-only run timed out instead of a false success. Tests 16/16.
- Flow-only remains NOT working: EKF heading with no yaw source rotates
  ~2-4.5 deg/s even when stationary (SIM_DRIFT_SPEED 0.002 did not help;
  reverted). Unverified hypothesis: EKF learns a wrong gyro bias. Options: a
  heading reference (pool lane-line direction from the bottom camera), or
  EK3 gyro-bias tuning. Also EKF depth in flow-only needs re-checking.

# Earlier: downward optical flow + pose-source switches (2026-10-15, late)

Switches (mira_sim Makefile, `make sim VAR=...`; launch args in
mira2_vision navigation.launch.py): POSE_SOURCE=slam|optical_flow|gt,
SLAM_ALGORITHM=auto|cuvslam|rtabmap, SLAM_OPTICAL_FLOW_ENABLED=0|1,
SLAM_IMU_ENABLED=0|1, OPTICAL_FLOW_FLOOR_DEPTH (default 2.32). Legacy
VISION_SOURCE/VO_BACKEND still map over.

Pieces: `consumers/optical_flow.py` (bottom camera, LK + fwd/back check,
rotation and depth-zoom compensation, per-point floor depth, median,
TwistWithCovarianceStamped /vision/flow_velocity); separate bottom node from
config/optical_flow.yaml. vision_bridge `source: none`, `flow_topic` (sends
VISION_POSITION_DELTA, vertical from SCALED_PRESSURE rate), `ekf_source_set`
(runtime MAV_CMD_SET_EKF_SOURCE_SET). guided_master publishes
/vision/ekf_attitude (attitude + depth, x/y NaN) even without horizontal
position. Sim: bottom camera bridged (/vision/bottom/*), gz_frame_id
bottom_camera_optical in model.sdf, static TF in bridge.sh, EK3_SRC2_* (no
POSXY, VELXY ExtNav, baro, yaw 0 = gyro only). utils declare_config treats an
unset dynamic list (YAML `[]`) as empty.

Findings:
- Offline (truth attitude/depth, 1192 recorded frames): LK flow 0.011 m/s RMS,
  slope 1.000. Phase correlation/ECC/Farneback much worse. Live scale error was
  the height reference: SITL pressure depth reads 0.124 m deeper than Gazebo
  (std 0.044) -> floor_depth must be the floor's PRESSURE depth (2.32 in sim).
- SLAM + flow (SLAM_OPTICAL_FLOW_ENABLED=1): gate PASSED x=7.06, flow bias
  -0.016 m/s, RMS 0.14 m/s live, EKF XY error max 0.12 m over 12 m. One run.
- Flow only: NOT working yet. Fixed so far: VISION_POSITION_DELTA vertical=0
  made the EKF lose depth (vehicle surfaced); EKF's own vz fed back made depth
  run away; compass yaw unusable in SITL (gyro-only now); bootstrap deadlock
  (no LOCAL_POSITION_NED before aiding). Last run: depth OK (mean -0.10 m),
  but XY drift 7.5 m over 12 m, flow over-read ~35 % in closed loop, gate not
  reached while BT claimed SUCCESS (false success from drifted odom).
  Next: why closed-loop flow differs from the 0.14 m/s SLAM+flow run (height
  now from GLOBAL_POSITION_INT relative_alt rather than local z; gyro yaw;
  flow rate), and whether TraverseGate should refuse to declare success
  without fresh gate observations near the plane.
- Recurring "EKF feedback is stale" aborts (age 1.52 s) appear in some runs;
  diagnostic subscriber saw no gaps, so the BT process stalls. Unresolved.
Defaults unchanged: POSE_SOURCE=slam, flow off. Tests 16/16. Not committed.

# Earlier: smooth motion + IMU experiment (2026-10-15, end of session)

User found the first demo jerky and asked for a faster spawn scan and smoother
waypoints, and suggested IMU in odometry. Changes:

- ScanArea: viewpoints are passed through (next heading sent when within half
  a step, no dwell); descent and return still settle. Defaults now radius 0,
  step 30 (launches, main.cpp, mira_sim Makefile/runner/mira-stack.py). The
  360 deg scan is one steady 30 deg/s turn, ~12 s (was ~45 s pulsed).
- guided_master: every target carries position AND yaw (no path-facing yaw and
  no separate arrival turn); hold keeps the current heading.
- guided_master `/goal_pose_direct`: straight-line carrot streamed at 20 Hz as
  position+velocity targets (ArduSub posvel controller, bypasses wp_nav and
  so BendyRuler), trapezoidal speed (--direct-speed 0.8, --direct-accel 0.3),
  continues from current speed, carrot capped 1 m ahead of the vehicle.
  TraverseGate sends only the visually aligned crossing on it; the approach
  (flare detour) keeps OA. Root cause it fixes: with lateral error up to 0.2
  m, BendyRuler's 0.5 m margin routed around a gate post (false success at
  x=5.49). Chosen over toggling OA_TYPE because PARAM_SET persists.
- objects.hpp `kStaleLimit` 1.5 s: Scan/Gate pause on stale EKF and only fail
  on sustained loss. SITL->MAVProxy delivers EKF with ~0.25 s stalls even
  idle; recorder load pushed one past 0.5 s and aborted a run.
- IMU: sim bridge now publishes /vision/imu (1 kHz, frame
  bluerov2_heavy/base_link/imu_sensor, static TF roll pi in bridge.sh).
  CuVslamOdometry supports `imu_topic`/`imu_frame` (Multisensor). Recorder
  captures IMU and replays with and without it. RESULT: IMU fusion is worse
  on the same 391 frames (ATE 0.83 m vs 0.088 m RGB-D only; offline variants
  noise x1/x10/x100: 2.2/14.6/1.0 m vs 0.043 m) despite a correct mount
  (gravity estimate 0,0,-9.81). Default is therefore OFF
  (`cuvslam_imu_topic: ''`). Ideas: cuVSLAM Inertial mode with the sim stereo
  pair; check the Gazebo IMU (reads 9.66 m/s^2 at rest, not 9.81).
- Results: diagnostic run crossed x=6.95; recorded run
  `recordings/slam_validation/gate_pass_smooth/` crossed x=6.947, aperture
  PASS, ring +0.161 m, flare 0.74 m, 49.7 s video (that live run had IMU fusion
  on; it was turned off afterwards). 0 yaw reversals vs several before.
  Videos copied to ~/Videos/mira_gate_pass.mp4 and mira_gate_pass_smooth.mp4.
- Remaining roughness: a slow-down (~0.25 m/s) near the stage point when the
  flare detour leaves it off-axis. Tests 16/16. Nothing committed.

# Earlier: verified gate pass recorded (2026-10-15, later session)

This supersedes the "honest current result" below. Two consecutive fresh runs
passed through the physical aperture; the second is the recorded demo.

Root causes found with `mira_sim/tools/diagnose_gate.py` (now also logs the depth
proximity scan) — not odometry drift:

1. EKF feedback arrived at 3.4 Hz, not 30 Hz. MAVProxy (inside sim_vehicle.py)
   re-requests data streams at its default 4 Hz, overriding guided_master's
   one-shot SET_MESSAGE_INTERVAL. Fix: `guided_master._request_rates` re-asserts
   the 30 Hz interval every 2 s (also guards against a GCS on hardware). This
   explains earlier "EKF feedback is stale" aborts.
2. With 30 Hz TF, the BT node's TF buffer fell ~3 s behind: the listener had no
   spin thread and the 10 Hz tick's spin_some took one /tf message per tick.
   Every gate observation then failed the TF lookup *silently* and TraverseGate
   never published. Fix in main.cpp: TransformListener spin thread + executor
   spin_all each tick. TF failures/waiting are now logged (throttled).
3. The orange flare (7,-3) lies on the spawn->gate line. BendyRuler correctly
   detours around it, which left the old routine (approach 0.6 m before the
   plane, hard fail at 0.5 m) misaligned with no room to recover; in the old
   false-success run it then routed around a gate post. Fix in gate.hpp: stage
   point `stage_distance` (default 2 m) on the gate axis, cross immediately
   when aligned anywhere from stage+0.7 m to 0.8 m before the plane, otherwise
   settle at the stage point; fail only on reaching the plane unaligned.
   Accepted gate observations are averaged before the 4 m latch; 50 ms TF wait.

World/gate geometry untouched (user: the original sauvc world is correct).

Results (Gazebo truth used only by the external verifiers):
- Diagnostic run: crossing x=6.81 (posts 6.25/7.75), BT SUCCESS, aligned with
  lateral 0.04 m from 49 observations, EKF error <=0.07 m on approach.
- Recorded run `/tmp/scan_gate_verified` (container), copied to host
  `recordings/slam_validation/gate_pass_verified/` (no 2.4 GB sequence.pkl or
  map_snapshots.pkl): crossing x=7.147, z=-1.30, aperture with hull margin
  PASSED, ring clearance +0.071 m, min flare centre distance 0.61 m, scan
  radius 0.818 m / return 0.142 m / yaw 362.7 deg. cuVSLAM replay over 21.4 m:
  VO ATE RMSE 0.861 m (endpoint 1.50 m); native SLAM ATE RMSE 0.270 m
  (endpoint 0.107 m); RPE(1 s) ~0.08 m (see accuracy.json). Much better than
  the previous 2.2 m run, but VO alone still drifts; live EKF tracked truth
  within ~0.1 m in the diagnostic run. gate_demo.mp4 1920x1080,
  77.4 s; frames inspected (flare detour, crossing between posts, map).
- `mira_sim/tools/test_actions.py`: 16/16 pass with updated staging expectations.

Remaining caveats (older section): two passes are not a success rate; the gate depth estimate
sits ~0.35 m before the true plane (success threshold still lands past it);
mira_sim compose still lacks `--streamrate` for MAVProxy (not changed to avoid
recreating containers; guided_master fix makes it unnecessary). Nothing committed.

---

# Earlier handoff: active spawn scan and gate routine (2026-10-15)

This section is the latest status and supersedes older validation/deferred-work
statements below. Historical notes are preserved after this section. The user
requested this handoff after asking why validation was taking so long.

## Overall requested task and constraints

- Work across `mira` and sibling `../mira_sim`.
- Validate cuVSLAM versus RTAB-Map accuracy. Run only ONE backend in the final
  stack; prefer cuVSLAM if accuracy is reasonably close.
- Make gate behavior a `mira2_actions` routine: user brings up sim + SITL + stack,
  and the routine handles the behavior automatically.
- The robot starts without prior scene knowledge. Add a separate configurable
  `ScanArea` BT task to actively survey around spawn; the user controls coverage.
- Clear the physical starting ring before lateral movement; do not collide.
- Find, align with, and pass through the gate. Avoid the previous unnatural stop
  in front of the gate and failure to inspect the starting area.
- Generate a new demonstration video showing the gate pass, actual third-person
  Gazebo view, depth image, and generated 3D SLAM map.
- Recording, ground-truth verification, and demo presentation are EXTERNAL to
  the behavior routine. Never steer using simulator truth or hardcoded gate
  world coordinates. Do not run two autonomy owners simultaneously.

## Honest current result

The configurable scan and gate routine are implemented and the last core build
passed 16 isolated behavior tests. Starting-ring clearance was verified in live
simulation. A fully verified gate pass with the latest routine is NOT achieved.

The latest full retry (`/tmp/scan_gate_retry_now` inside `mira-stack-gpu`) completed
the BT with SUCCESS, but the independent simulator check caught a FALSE SUCCESS:
the robot passed beside the aperture. Do not present that video as successful.
Do not equate ROS launch exit code or BT arrival with an actual gate crossing.

Latest retry measurements:

- Scan requested: radius 0.75 m, sweep 360 degrees, step 15 degrees, depth 1 m.
- Measured maximum scan radius 0.8764 m, return distance 0.2336 m, yaw range
  360.70 degrees.
- Conservative starting-ring clearance +0.02047 m: passed.
- Actual gate-plane crossing: world x=4.60645, y=0, z=-1.32435 m. Valid x with
  hull margin is 6.56..7.44 m: FAILED.
- Minimum center distance to flare 0.70065 m.
- cuVSLAM replay: 1,039/1,039 valid matched RGB-D frames, retained at 15.15 Hz.
  VO positional ATE RMSE 2.1693 m; native SLAM ATE RMSE 2.2561 m. Endpoint
  errors about 4.93/5.18 m. Native map saved, 202,884 colored map points.
- These are REPLAY accuracy measurements, not captured live-VO accuracy. Do not
  assume the live and replay trajectories are identical.
- Live logs near the gate showed roughly 6 navigation updates per second and
  an EKF lane switch. This is evidence to investigate, not a proven root cause.
- Earlier retries were blocked by host load about 235 on 24 CPUs. On this retry
  load was about 20, but another unrelated GPU workload was running. Do not stop
  unrelated containers/jobs.

## Implemented behavior

`src/mira2_actions/config/gate.xml` is a Sequence:

1. `ScanArea` (timeout 150 s, blackboard radius/sweep/step/depth, face target).
2. `SelectObject` for the configured target class (default gate).
3. `TraverseGate` (through distance 1.5 m, timeout 90 s).

`src/mira2_actions/src/main.cpp` registers the new nodes and exposes blackboard
parameters. Dedicated `launch/gate.launch.py` and existing
`launch/mira2_actions_launch.py` expose `scan_radius=0.75`, `scan_sweep=360`,
`scan_step=15`, `scan_depth=1.0`.

`src/mira2_actions/src/scan.hpp`:

- Starts at the current EKF XY pose and descends vertically to survey depth.
- No lateral motion until the descent goal is reached (0.25 m position,
  0.15 rad yaw, 0.3 s dwell).
- Visits a configurable circle in angular steps, then returns to the survey
  origin. Radius 0 rotates in place; sweep 0 does clearance descent only.
- Optionally faces an observed target after return, using detections and TF.
- Minimum survey depth defaults 0.6 m. Rejects shallow/invalid parameters.
- Requires fresh EKF feedback (0.5 s); stale feedback, timeout or cancellation
  holds. This safety limit was not relaxed to force a demo through.

`src/mira2_actions/src/gate.hpp`, latest installed implementation:

- Uses a depth-localized gate and TF into odom; derives approach direction from
  the initial robot-to-gate vector (assumes frontal gate approach).
- Accepts updated gate positions only within 0.3 m of the original reference,
  to reject floor/flare/partial-frame depth contamination.
- Latches mapped geometry at estimated distance <=4 m; stops requiring camera
  observations after that, because close-range cropping previously caused safe
  failures. THIS ASSUMPTION NEEDS REVISITING AFTER THE LATEST FALSE SUCCESS.
- Approach target is 0.6 m before the mapped plane. Bounded updates avoid
  constantly restarting BendyRuler.
- Switches while moving to a goal 1.5 m past the mapped gate when estimated
  distance <=2.3 m, lateral error <=0.20 m, vertical <=0.25 m, yaw <=0.30 rad.
- Reports success 1 m beyond mapped plane with lateral <=0.35 m, vertical
  <=0.30 m, yaw <=0.20 rad for 1 s. This is odometry-relative, and latest run
  proves it alone cannot certify passage through the physical aperture.
- Requires fresh EKF; cancellation/stale feedback/timeout invoke guidance hold.

## SLAM and navigation changes

`src/mira2_vision/mira2_vision/cuvslam_odometry.py`:

- Uses installed cuVSLAM 17.0.1+51631ba on CUDA 12.8/RTX PRO 6000 Blackwell.
- Added native cuVSLAM SLAM alongside continuous VO. Loop-closed pose is
  published separately on `odom_topic + '/slam_pose'` in `odom_frame + '_map'`;
  mapping corrections are not sent as EKF odometry jumps.
- Added explicit `rgbd` and `multisensor` tracking modes. Simulation defaults
  to multisensor in `consumers/vision_bridge.py`; hardware keeps legacy rgbd.
- Multisensor uses one RGB-D camera, depth_camera_ids=[0], scale 1000,
  enable_depth_stereo_tracking, GPU, async SBA, observation export. No second
  SLAM backend or added IMU fusion.
- Stored base<-camera extrinsics for external reconstruction. Depth limit 8 m,
  BGR->RGB conversion, meters->uint16 mm; lost covariance 9999 and minimum 15
  observations. Do not treat non-lost status as proof of metric accuracy.

Historical comparison was 563 common timestamps on an older route: cuVSLAM
position RMSE ~0.206 m versus RTAB-Map ~0.697 m; RTAB orientation ~1.86 degrees
versus cu ~2.96 degrees. That motivated cu preference, but does NOT establish
which is best on the NEW active-scan route. Legacy cu RGBD mode drifted severely
on rotations (~15.9 m RMSE). Multisensor fixed one recorded scan/approach to
~0.080 m VO and ~0.035 m SLAM, but the newest complete route drifts again as
reported above. No honest overall accuracy winner is established for this route.

`../mira_sim/docker/ardusub_vision.parm` now sets EK3_SRC1_YAW=6 (ExternalNav),
matching existing XY ExternalNav inputs. Compass yaw previously realigned
independently during scan. `docker/check-sitl-avoidance.py` checks this together
with OA_TYPE=1, PRX1_TYPE=2, WP_YAW_BEHAVIOR=1. Those startup checks passed.
Existing BendyRuler handles motion/avoidance; depth publishes obstacle distances.
OA margin 0.5 m, lookahead 3 m, yaw rate cap 30 degrees/s.

Simulator Makefile/runner default GPU to cuVSLAM and CPU to RTAB-Map, and forward
SCAN_RADIUS/SCAN_SWEEP/SCAN_STEP/SCAN_DEPTH. Runtime uses a single backend.
Optional RTAB backend support remains in source. Docker demo dependency adds
ffmpeg; extra RTAB mapping dependencies were removed from cu-only demo workflow.

## Physical starting ring

In sibling simulator submodule file
`../mira_sim/src/sauvc_sim/models/starting_zone/model.sdf`, added matching
primitive collisions for the four existing visual white bars. Original ring
was visual-only. Preserved original formatting and visuals.

Ring is 1.4 x 1.4 m at world (7,-10,-0.2), bar radius 0.04 m. External verifier
uses an enclosing 0.45 m hull sphere and subtracts a motion allowance between
truth samples. These world coordinates are verification-only. Behavior clears
the ring by descending at spawn XY before lateral movement.

## External recording and diagnostics

`mira_sim/tools/record_gate_slam.py` (external, cu-only):

- `--run-gate` launches the dedicated gate routine; `--replay-only` reuses the
  saved dataset. Start stack with RUN_ACTIONS=0 to avoid duplicate action owners.
- Spawns an actual Gazebo observer camera at 640x480, 5 Hz. Retains matched
  RGB-D at about 15 Hz to reduce recording overhead (previously 30 Hz and a
  larger observer). The retained sequence may differ from live VO inputs.
- Generates four-panel 1920x1080 video: third person, onboard RGB with gate
  boxes, registered depth, and native cuVSLAM map reconstruction over time.
- Native cu map saved through tracker.save_map; dense colored points are
  registered depth fused with optimized native cu poses, not another mapper.
- Saves gate_action.log, sequence.pkl (serialized ROS RGB/depth/info, truth,
  phases/stamps, camera extrinsics), accuracy.json, trajectories.json,
  map_snapshots.pkl, map.ply, map.png, native cuvslam_map/, depth.png,
  third_person.png, capture.mp4/composite.mp4/gate_demo.mp4 and snapshot.
- Accuracy uses fixed-scale first-pose SE(3) alignment and interpolated truth;
  reports ATE and 1-second RPE. Previously checked rigid-transform zero error
  and preservation of scale errors.
- Checks the actual gate aperture crossing and conservative ring clearance;
  raises on failed BT, ring or gate verification even when a video was written.
- Parses actual `Behavior tree finished: SUCCESS`, because ros2 launch itself
  may return 0 even when its child action fails.
- Ground truth is only for external verification and camera framing.

`mira_sim/tools/diagnose_gate.py` was added in the latest turn to isolate the issue
without video, RGB-D serialization or SLAM replay. It records live truth, VO,
EKF, commanded goals and TF-transformed gate detections into telemetry.json.
Python syntax compilation passed; it has NOT yet produced a useful moving run.

The first lightweight diagnostic accidentally launched too early: its own
goal subscription satisfied ScanArea's subscriber-count check before the real
controller had discovered the action publisher. The initial single-shot goal
was missed; robot remained stationary at spawn. No useful gate diagnosis was
obtained. The tool now waits for >=2 goal subscribers (itself + controller)
before launching. This fix has NOT been rerun. It also exposes a general
startup robustness concern with relying on any subscriber as controller
readiness and publishing a goal only once. Do not confuse this diagnostic
startup race with the earlier full recorded run, which DID move and miss gate.

## Tests and recorded evidence

- Last core build/install succeeded inside the GPU stack container.
- `mira_sim/tools/test_actions.py`: 16 isolated contracts passed, about 33.3 seconds,
  ROS_DOMAIN_ID=79. Includes original navigation contract checks and ScanArea
  descent/circle/return/config/cancel/stale checks, plus moving gate switching,
  foreground-jump rejection and holds. Tests do not validate real SLAM accuracy.
- Core tests are for the current gate logic; only comments/docs, recorder
  overhead reductions, and the new diagnostic changed since that build.
- `mira_sim/tools/test_gate_depth.py` contains synthetic hollow-frame depth checks.
  Existing frame association samples perimeter depth and a foreground
  percentile; cropped/occluded frames can contaminate it. Inspect before changing.
- Saved host evidence: `recordings/slam_validation/scan_area/` contains
  behavior_tests.log, scan_validation.json, startup_scan.mp4, map.png, map.ply,
  native cuvslam_map/. Explicitly scan-only, not a successful gate demo.
- That earlier good scan returned within 0.0955 m, radius ~0.8002 m and cleared
  the ring with ~0.0126 m conservative reserve; its subsequent gate pass failed.
- Older `recordings/slam_validation/gate_pass/` is historical mixed-backend
  evidence, not the requested final cu-only active-scan demonstration.

Important container output directories (all retained, some very large):

- `/tmp/scan_gate_retry_now`: NEWEST full recorded retry, false BT success,
  actual gate failure, native map/video and ~2.1 GB sequence.pkl.
- `/tmp/scan_gate_final`: earlier good scan/map but failed gate; source of
  saved host scan-only evidence.
- `/tmp/scan_gate_consistent_nav`: near-range observation failure.
- `/tmp/scan_gate_complete`: stopped safely on stale feedback under extreme load.
- `/tmp/gate_telemetry_retry`: stationary diagnostic described above; interrupted
  for this handoff. Not a scan/gate pass.
- Other scan_gate_* and gate_slam_demo* directories are historical failed or
  superseded runs. Read accuracy.json before labeling any artifact successful.

Latest failed-run accuracy was also copied to host
`/tmp/mira_gate_retry_handoff_accuracy.json`; interrupted diagnostic directory
copied to `/tmp/mira_gate_retry_handoff_telemetry/`. Earlier host scratch copies
include `/tmp/gate_retry_accuracy.json`, `/tmp/gate_retry_trajectories.json`,
`/tmp/gate_retry_third.png`.

## Commands and next work

From `/mnt/shared/DAVID/PERSONAL/mira`:

```bash
# Fresh stack only; no automatic action while external tool owns launch.
make sim GZ_SERVICE=mira-sim-gpu HEADLESS=1 RUN_ACTIONS=0

# Focused telemetry FIRST; source ROS and application overlay in container.
docker exec mira-stack-gpu bash -c 'source /opt/ros/jazzy/setup.bash; source /mira_build/install/setup.bash; python3 -u /workspace/tools/diagnose_gate.py /tmp/gate_telemetry_next'

# Full demo only after diagnosing navigation and verifying actual crossing.
docker exec -e GZ_PARTITION=mira_stack mira-stack-gpu bash -c 'source /opt/ros/jazzy/setup.bash; source /mira_build/install/setup.bash; python3 -u /workspace/tools/record_gate_slam.py /tmp/scan_gate_next --run-gate --scan-radius .75 --scan-sweep 360 --scan-step 15 --scan-depth 1.0'

# Normal automatic behavior once ready (recording remains external).
make sim GZ_SERVICE=mira-sim-gpu HEADLESS=1 RUN_ACTIONS=1 SCAN_RADIUS=.75 SCAN_SWEEP=360 SCAN_STEP=15 SCAN_DEPTH=1.0

# Stop only this Mira stack.
make sim-down
```

1. Rerun lightweight telemetry with the readiness fix. Compare live VO and EKF
   against truth across scan, approach and crossing. Inspect gate observations
   and goals before/after the 4 m latch and EKF lane switch. Confirm real input
   rates and distinguish replay drift from live navigation drift.
2. Fix the demonstrated error, not just BT success thresholds. The old fixed
   mapped gate plus drifting odometry can report false success; investigate
   robust gate-frame localization and observation consistency. Do not force a
   pass by using simulator truth, reducing hull margins or removing stale holds.
3. Consider robust controller readiness/goal acknowledgement or appropriate
   command retry; observer subscriptions must not falsely indicate readiness.
4. If cu accuracy remains substantially worse, compare RTAB on the same saved
   scan/approach dataset offline and choose ONE backend according to the user's
   stated preference. Historical older-route results are insufficient.
5. After a real verified pass, record the requested four-panel demo, copy
   successful artifacts into a clearly named host recordings directory, inspect
   video/map and ffprobe duration, and update README validation limitations with
   measured results. Do not package a known failed pass as final evidence.

## Workspace and shutdown state

- All implementation is uncommitted. Preserve unrelated pre-existing changes;
  do not git reset/clean. Many current dirty files came from earlier refactors.
- Mira tracked dirty files include README, actions README/launch/main/navigation/
  objects, vision README/launch/bridge/cuvslam/depth_targets/package, shared depth
  utility; new scan.hpp/gate.hpp/gate.xml/gate.launch.py and mira_sim/tools scripts.
- Sibling dirty files include Makefile, docker-compose.mira.yml, SITL parameters,
  runner, Dockerfile, Python runner, wait-mira-target.py, docs; new OA Dockerfile
  and avoidance checker. Starting ring change is INSIDE sauvc_sim submodule.
- `CONTINUE.md` already contained earlier handoff notes. They are preserved
  below; older claims that tests/sim are deferred are superseded by this section.
- On user handoff request, sent SIGINT to the active diagnostic action (PID 354
  in this run), then ran sibling `make mira-stack-down`. Mira simulator and SITL
  are stopped; unrelated workloads are untouched. Output directories persist
  in the stopped container. Do not reuse old PIDs when resuming.
- No subagents were used in this continuation. No commits or publication.

---

# Earlier handoff notes: Mira stack refactor

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
not mechanical manipulation. See misc/docs/mira2_actions.md.

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
must be selected explicitly, including for the clock. See misc/docs/mira2_vision.md.

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

