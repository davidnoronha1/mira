# Control

This package contains the joystick teleoperation node.

## Teleoperation

`joystick_exe` consumes `/joy` and publishes normalized commands to
`/mira/control/command`.

```bash
ros2 launch mira2_control teleop.launch
ros2 launch mira2_hardware master.launch control_source:=teleop
```

Sensitivity, deadzone, and the QGC-style layout are configured in
`config/teleop.yaml`.
