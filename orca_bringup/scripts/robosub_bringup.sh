#!/bin/bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 timesync_node:=MAVLINK
python3 /home/orca4/colcon_ws/src/orca4/orca_bringup/scripts/zed_mavros.py

sudo docker ps
sudo docker container exec -it [container id] bash


ros2 topic list

ros2 topic echo [name]