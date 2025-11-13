#!/bin/bash
# Build ZED packages, skipping Isaac ROS integration packages that require additional setup
colcon build --packages-skip zed_isaac_ros_nitros_sub zed_isaac_ros_april_tag
source /home/orca4/colcon_ws/install/setup.bash