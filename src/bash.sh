#!/bin/bash
source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash

ros2 launch bcr_arm_gazebo bcr_arm.gazebo.launch.py
