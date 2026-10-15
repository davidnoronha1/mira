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
ros2 launch mira2_hardware guided.launch.py simulation:=true \
  config:=$(ros2 pkg prefix --share mira2_vision)/config/guided_front.yaml
ros2 launch mira2_actions mira2_actions_launch.py use_sim_time:=true
```

The demo defaults to `dock`, matching the supplied model labels. Override with
`target_class:=your_label` when using another model. Set `objects_topic` on
`SelectObject` for another camera. Simulation requires explicit arming or the
hardware launch's simulator-only `auto_arm:=true`. The demo executes once and
exits; it does not claim to implement a complete docking or retrieval mission.
