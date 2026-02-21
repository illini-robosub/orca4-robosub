#!/bin/bash

# Set up the resource path for Gazebo to find models
export GZ_SIM_RESOURCE_PATH=$HOME/colcon_ws/src/orca4/orca_camera_testing/models:$GZ_SIM_RESOURCE_PATH

# Source the colcon workspace
source ~/colcon_ws/install/setup.bash

echo "GZ_SIM_RESOURCE_PATH set to: $GZ_SIM_RESOURCE_PATH"