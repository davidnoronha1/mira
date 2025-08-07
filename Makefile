.PHONY: master alt_master
SHELL := /bin/bash

WS := source install/setup.bash

build:
	colcon build --parallel-workers 4 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 

master:
	${WS} && ros2 launch mira2_pid_control mira2_pid_control.launch.py

alt_master:
	${WS} && ros2 launch mira2_pid_control mira2_pid_control_alt.launch.py