#!/usr/bin/env bash

# Minimal environment for testing IMU TF broadcaster

if [[ -z "${COLCON_WS}" ]]; then
  export COLCON_WS="$HOME/colcon_ws"
fi

source /opt/ros/humble/setup.bash
source ${COLCON_WS}/install/setup.bash