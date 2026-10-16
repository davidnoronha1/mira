# Mira vision

`mira2_vision` processes camera data and publishes ROS measurements. It opens no
MAVLink connection and imports no autopilot protocol library. Pixhawk forwarding
belongs to `mira2_hardware` controller plugins.

## Camera workers and plugin parameters

Run one worker per camera with `detection.launch.py`. Select detectors and
consumers explicitly, or use the installed `config/front.yaml`, `bottom.yaml`,
`pipeline.yaml`, `bucket_front.yaml` and `guided_front.yaml` profiles.

```sh
ros2 launch mira2_vision detection.launch.py camera:=front
ros2 launch mira2_vision detection.launch.py camera:=bottom \
  config:=$(ros2 pkg prefix --share mira2_vision)/config/bottom.yaml
```

Plugin settings use `<plugin>.<setting>` (nested YAML is flattened by ROS):

| Plugin | Settings |
|---|---|
| `yolo` | `model_name`, `device`, `rate`, `conf_threshold`, `nms_threshold`, `input_width`, `input_height` |
| `aruco` | `dictionary`, `rate`, `clahe_clip_limit`, `clahe_grid_size` |
| `markers` | `marker_size`, `publish_tf` |
| `docking_board` | `marker_size`, `ids`, `offsets`, `filter_alpha`, `filter_max_jump`, `filter_reset_after` |
| `pipeline` | `hsv_lower`, `hsv_upper`, `min_area`, `kernel_size` |
| `depth_targets` | `target_class`, `frame_classes`, `min_depth`, `max_depth` |
| `obstacle_distance` | `topic`, `band`, `min_range`, `max_range` |
| `optical_flow` | `floor_depth`, `floor_heading`, tracking and topic settings |

Each selected plugin declares and validates its own settings. Camera acquisition,
calibration, `rate`, `max_frame_age`, preview settings, `simulation`, `detectors`
and `consumers` remain worker parameters. The worker's `rate` and `yolo.rate`
control different stages. Old flat plugin parameter names are removed.

```yaml
/**:
  ros__parameters:
    detectors: [yolo]
    consumers: [objects, depth_targets]
    yolo:
      model_name: package://mira2_vision/models/best.pt
      device: CPU
      rate: 15.0
    depth_targets:
      target_class: blue_bucket
      min_depth: 0.1
      max_depth: 20.0
```

Bare model filenames resolve under the installed `mira2_vision/models` directory;
absolute paths and `package://` paths also work. `yolo_model` and `yolo_device`
launch arguments override `yolo.model_name` and `yolo.device`.

## Native cuVSLAM and RTAB-Map

`navigation.launch.py` starts one odometry backend independently of detection.
`slam_algorithm:=auto` probes native cuVSLAM on the GPU, then falls back to
RTAB-Map. Explicit selection fails with an installation error if unavailable.
`start_vo:=false` uses an independently launched estimator.

```sh
ros2 launch mira2_vision navigation.launch.py slam_algorithm:=cuvslam
ros2 launch mira2_vision navigation.launch.py slam_algorithm:=rtabmap
```

The native Python estimator publishes continuous body-FLU `nav_msgs/Odometry`
on `/vision/vo_odom`, with base-to-camera extrinsics from TF. Pixhawk's EKF owns
`odom -> base_link`; neither backend publishes navigation TF. Tracking failures,
nonfinite estimates and fewer than 15 observations produce covariance 9999 and
clear velocity history. Duplicate or backward acquisition stamps are discarded;
a simulator clock reset requires restarting the estimator to establish a new origin.

Native mapping runs alongside VO. Loop-corrected `PoseStamped` measurements use
`/vision/vo_odom/slam_pose` in `vo_odom_map`, so map corrections never jump the
EKF input. The tracker also exposes native map saving and optimized pose history
through NVIDIA's API for offline reconstruction. The restored wrapper retains
camera extrinsics for the existing replay renderer, and replays may request
IMU fusion the same way as the live node.

`config/cuvslam.yaml` owns `cuvslam.odom_topic`, `odom_frame`, `base_frame`,
`enable_slam`, and `tracking_mode`. Hardware defaults to `rgbd`; simulation uses
`multisensor` for its pinhole registered depth. Multisensor is NVIDIA's
experimental depth-constrained solver and requires an official cuNLS-enabled
wheel and rectified pinhole inputs.

Visual-inertial fusion is opt-in and off by default: `slam_imu:=true` sets
`cuvslam.imu_topic` (`imu_topic`, or `/vision/imu` in simulation; required on
hardware) and forces `multisensor`. IMU extrinsics come from TF (`imu_frame`, or
the message frame), the rate from the observed stream, and noise from
`cuvslam.imu_noise.*`. Samples are registered in time order before each frame.
On the recorded gate run it was markedly worse than RGB-D only (ATE 0.8-15 m vs
0.04-0.09 m); it is rejected for RTAB-Map.

Both backends consume registered RGB-D directly from the source camera topics.
cuVSLAM shares synchronization and validation code in `rgbd_inputs.py`;
`rgbd_inputs.max_depth`, `max_skew`, and `max_age` in `config/rgbd_inputs.yaml`
configure its input handling. RGB/depth must have matching optical frames,
dimensions, and valid camera calibration. cuVSLAM rejects calibration changes
mid-track and masks invalid or distant depth in its backend conversion.
RTAB-Map uses its own synchronization and input handling. There is no intermediate
conversion or republishing node. Override `rgb_topic`, `depth_topic`,
`camera_info_topic`, `vslam_config`, and `rgbd_config` as needed.

## GPU installation

Install an official cuVSLAM wheel matching the deployment Python, CUDA major
version and architecture in the same Python environment as the ROS executable.
The validated simulator image uses cuVSLAM 17.0.1 with CUDA 12.8. cuVSLAM stays
optional; CPU installations only need `ros-jazzy-rtabmap-odom`. The Isaac ROS APT
repository, CUDA 13 compatibility injection and component container are removed.
GPU Docker builds accept `CUVSLAM_WHEEL_URL` for a matching official wheel;
CUDA runtime libraries must already be available in the deployment image.
See [NVIDIA's installation guide](https://github.com/nvidia-isaac/cuVSLAM#install-pycuvslam)
and [Python API](https://nvidia-isaac.github.io/cuVSLAM/python/api.html).

## Depth and ROS measurement outputs

The camera worker decodes depth once into a read-only metric map and retains
an eight-frame buffer. RGB association requires matching timestamps, optical
frames and dimensions. `frame.depth` is the matching sample or `None`.
`on_depth(sample)` consumers receive fresh depth events independently of RGB
and detector rates. `obstacle_distance` projects depth through calibrated
camera-to-body TF and publishes a body-FLU `LaserScan`; the hardware plugin
converts that scan to MAVLink. A separate `depth_camera_info_topic` can describe
unregistered obstacle-camera depth; RGB-D SLAM and detection association still
require registered depth.

`optical_flow` uses the bottom camera to publish body-FLU velocity and optional
floor-line heading. Select `pose_source:=optical_flow` for flow-only navigation
or `slam_optical_flow:=true` alongside SLAM. `optical_flow.floor_depth` must be
calibrated per pool because there is no range sensor. Floor-line heading is
opt-in and assumes visible straight lines along known pool axes; it has only
been validated in the simulated SAUVC pool. The matching autopilot settings
are in `mira2_hardware/config/ardusub/external_nav.parm`.

## Vehicle integration and extensions

```sh
# Start vision and one Pixhawk controller:
make guided HARDWARE_ARGS="pixhawk_address:=/dev/Pixhawk"
# Independently running Gazebo/SITL:
make guided-sim ARGS="pose_source:=optical_flow optical_flow_floor_depth:=2.5"
```

`external_navigation.*` and hardware `obstacle_distance.*` parameters belong on
`pixhawk_controller`, using `hardware_config`. The obsolete `vo.*`,
`obstacles.*`, `vision_link` and `obstacle_link` settings are removed. Vision
never establishes an EKF origin or sends firmware commands.

Custom vision plugins subclass `Plugin`, declare parameters with their own
namespace, and publish ROS outputs. Declare `requires`/`provides` product keys;
optional `after` dependencies apply when the provider is selected. Importable
`module:Class` plugins and `mira2_vision.plugins` Python entry points are both
supported. Never mutate shared `Frame.image` or depth arrays; `close()` releases
resources. Selected plugins run in the camera worker's executor thread.
