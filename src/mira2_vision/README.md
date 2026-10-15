# Mira vision

`mira2_vision` runs one acquisition/decode loop per camera. Detectors and
consumers are Python plugins in that same process: they receive the same NumPy
image and shared per-frame products. There are no internal ROS image topics or
image serialization between plugins. Published outputs retain the existing
`/mira/perception/{front,bottom}` interfaces. Preview drawing uses a separate image
copy only when a preview subscriber or a local visualization needs it. Detector
preprocessing, inference and ROS output conversion can still allocate buffers.

## Selecting plugins

The launch file reads a camera configuration, then applies explicit plugin lists.
A blank launch argument uses the list from the configuration; `[]` attaches none.
Names can be comma-separated or a YAML list. Old `enable_*` flags are removed.

```bash
# YOLO once, publish its shared bounding boxes.
ros2 launch mira2_vision detection.launch.py detectors:=yolo consumers:=objects

# YOLO once, add registered-depth association and bucket target outputs.
ros2 launch mira2_vision detection.launch.py \
  config:=$(ros2 pkg prefix --share mira2_vision)/config/bucket_front.yaml \
  detectors:=yolo consumers:=depth_targets,objects

# One ArUco pass shared by marker and docking-board pose consumers.
ros2 launch mira2_vision detection.launch.py camera:=bottom \
  config:=$(ros2 pkg prefix --share mira2_vision)/config/bottom.yaml \
  detectors:=aruco consumers:=markers,docking_board

# HSV pipeline independently of YOLO; optional ArUco marker IDs.
ros2 launch mira2_vision detection.launch.py camera:=bottom \
  config:=$(ros2 pkg prefix --share mira2_vision)/config/pipeline.yaml \
  detectors:=pipeline,aruco consumers:=markers
```

The camera argument sets the output namespace. The configuration sets input image
and calibration topics; select the corresponding configuration when changing
cameras. Each worker can use ROS images, a camera, a file or RTSP through
`utils.image_source`. `utils.depth` supplies camera projection and robust depth
association also used by standalone ZED utilities.

| Plugin | Role and outputs |
| --- | --- |
| `yolo` | One model inference; shared normalized `detections_2d` |
| `objects` | Publishes `objects` (`Detection2DArray`); no additional inference |
| `depth_targets` | Enriches shared detections with registered depth, publishes `target_pose`, `target_point`, `target_color` |
| `aruco` | One grayscale/CLAHE/marker detection; shared corners and IDs |
| `markers` | Individual marker poses on `markers`, optional TF, IDs on `pipeline/markers` |
| `docking_board` | Board solve/filter; `dock_pose` and BT-compatible `dock_target` |
| `vision_bridge` | Shared registered RGB-D inputs for RTAB-Map; odometry to MAVLink external navigation |
| `obstacle_distance` | Depth-rate obstacle projection to MAVLink and `obstacles` LaserScan |
| `pipeline` | Independent HSV/contour detector; `pipeline/offset`, `pipeline/angle` |

Depth association requires registered depth with matching image dimensions,
optical frame and acquisition time. Without it, bounding boxes still publish;
3D target outputs do not. Camera intrinsics are required for pose estimation.
YOLO and ArUco have separate configured rates. Skipped detector frames do not
reuse previous detections or trigger dependent consumers.

## Adding a detector or consumer

Subclass `mira2_vision.plugin.Plugin` and implement `process(frame)`. Set
`requires` to the product keys it reads and `provides` to the keys it writes.
The loader checks missing dependencies, duplicate providers and cycles at startup,
and orders selected plugins. It never silently enables another plugin.

`frame.image`, `frame.header`, `frame.matrix` and `frame.products` are shared
references. Treat the source image and existing products as read-only unless a
plugin explicitly documents an enrichment stage. Draw only on `frame.preview`,
which is `None` when no preview is needed. Products exist for one acquisition
only. A rate-limited detector simply returns without producing a product.

`depth_targets` explicitly enriches the shared detection message in-place and
provides `depth_association`. `objects.after = ('depth_association',)` ensures
publication follows that stage when selected, while allowing YOLO alone.
A custom consumer requiring 3D detections should set
`requires = ('detections_2d', 'depth_association')`. Other optional ordering can
use `after`; those dependencies apply only when their providers are selected.

```python
from mira2_vision.plugin import Plugin

class GateConsumer(Plugin):
    requires = ('detections_2d',)

    def __init__(self, node):
        super().__init__(node)
        node.declare_parameter('gate_min_score', 0.7)
        self.minimum = node.get_parameter('gate_min_score').value

    def process(self, frame):
        detections = frame.products['detections_2d']
        # Filter the shared result and publish your task-specific output here.
```

Select an importable class directly:
`consumers:=objects,my_package.gates:GateConsumer`. Alternatively register it in
its Python distribution:

```python
entry_points={
    'mira2_vision.plugins': ['gate = my_package.gates:GateConsumer'],
}
```

Then select `consumers:=objects,gate`. No changes to `mira2_vision` are needed.
Plugin constructors can declare their own ROS parameters; `close()` releases
plugin resources at shutdown. Extensions should keep processing bounded because
all selected plugins share the camera worker's executor thread.


## Shared depth and navigation

The worker owns the single `depth_topic` subscription and decodes 16UC1
millimetres or 32FC1 metres once, respecting byte order and padded rows. Invalid
pixels become NaN. A bounded eight-frame buffer matches RGB by timestamp,
optical frame and dimensions. `frame.depth` is the matching `DepthFrame`, or
None; `depth_targets` and `vision_bridge` reuse it. The worker optionally
publishes normalized 32FC1 on its relative `depth` output.

A plugin can implement `on_depth(sample)` to receive every fresh depth event,
independent of RGB and detector rates. `sample.image` is a read-only NumPy map
in metres; `sample.header`, `sample.info` and `sample.message` are shared.
`obstacle_distance` uses this interface and the depth camera's intrinsics and
mount TF. `depth_camera_info_topic` defaults to the RGB CameraInfo, suitable
for registered depth. Set it separately for an unregistered depth camera;
obstacle processing works independently, but RGB-D VO and YOLO association
still require registration. This package consumes a camera's depth map; it
does not infer monocular depth or register unaligned images automatically.

`vision_bridge` publishes matched `vo/rgb`, `vo/depth`, `vo/camera_info` under
the worker namespace. RTAB-Map consumes these and publishes `/vision/vo_odom`.
The same plugin converts that odometry to MAVLink, including heading alignment.
The estimator remains a separate ROS node; there is no second raw depth decoder.
Camera-to-base TF must be calibrated. RTAB-Map does not publish odom TF; the
hardware goal bridge publishes the EKF's odom-to-base transform.

```sh
# Hardware: ROS outputs work without a Pixhawk connection.
ros2 launch mira2_vision navigation.launch.py

# Hardware forwarding: use distinct MAVLink-router outputs, never share serial readers.
ros2 launch mira2_hardware guided.launch.py \
  vision_link:=udpin:0.0.0.0:14555 obstacle_link:=udpin:0.0.0.0:14558 \
  goal_link:=udpin:0.0.0.0:14557

# Simulator: retain mira_sim's Gazebo sensors/clock and SITL; stop its old VO/nav scripts.
ros2 launch mira2_hardware guided.launch.py simulation:=true \
  start_x:=11 start_y:=-11.5 auto_arm:=true

# Add YOLO target poses to the same camera worker (choose the model and target class).
ros2 launch mira2_hardware guided.launch.py simulation:=true \
  config:=$(ros2 pkg prefix --share mira2_vision)/config/guided_front.yaml
```

`config/navigation.yaml` selects only the two navigation consumers; YOLO is not
loaded. Set `start_vo:=false` to use an existing estimator. `source:=gt` is a
simulation-only comparison mode and skips RTAB-Map. Hardware input defaults
are `/mira/sensors/camera/front/{image_raw,depth/image_raw,camera_info}`; sim
uses `/vision/{image,depth,camera_info}`. Override `rgb_topic`, `depth_topic`,
`camera_info_topic`, and `namespace` as appropriate.

| Capability | Hardware (`simulation=false`, default) | Simulation |
| --- | --- | --- |
| Clock | Wall/ROS system time; simulated clock rejected | `/clock` |
| Ground-truth odometry | Disabled; forces VO | Optional `source=gt` |
| Spawn translation | Disabled (zero offset) | `start_x`, `start_y` |
| MAVLink endpoints | Empty/disabled until configured | SITL 14555/14558; goal 14557 |
| EKF origin writes | Disabled unless `vo.set_origin` and `vo.origin_configured` are both true | Simulator origin defaults |
| Goal auto-arm | Disabled, even if requested | Explicit `auto_arm=true` |
| Gazebo markers / truth paths | Disabled | Optional visualization |

For hardware without GPS, configure a surveyed `vo.origin_lat/lon/alt` in
YAML along with both origin flags if this consumer should establish the EKF
origin. The simulator's geographic coordinates are never implicitly applied
on hardware. Normal VO, obstacle forwarding, EKF paths and GUIDED goals remain
available on hardware. Connections are nonblocking, send only after a fresh
FCU heartbeat, and close with their plugins.

The goal-pose bridge remains in `mira2_hardware`. `guided.launch.py` composes
that bridge with this package's navigation launch. It is an alternative to
`async_master`, not an additional command owner. Avoidance still requires
proximity/OA-enabled ArduSub firmware, as in mira_sim's patch.

The package, executable, Python imports and extension entry-point group are now
`mira2_vision` / `mira2_vision.plugins`. Update external plugin registrations to
that group; the old package name is not retained as an alias.
