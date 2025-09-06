.PHONY: master alt_master build
SHELL := /bin/bash

WS := source install/setup.bash

controller:
	${WS} && ros2 launch mira2_pid_control controller

build:
	source /opt/ros/jazzy/setup.bash && \
	colcon build --parallel-workers 4 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 

joystick:
	${WS} && ros2 launch mira2_rov teleop.launch

master:
	${WS} && ros2 launch mira2_control_master master.launch

alt_master:
	${WS} && ros2 launch mira2_control_master alt_master.launch
