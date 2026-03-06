#!/bin/bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "Launching AUV Vision Pipeline with CLAHE..."
ros2 run mira2_perception bp_ros2_node.py
