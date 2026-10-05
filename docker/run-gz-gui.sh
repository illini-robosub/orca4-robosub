#!/usr/bin/env bash

# Attach a native Gazebo GUI to an already-running orca4 container (started by
# run.sh in VNC mode). Use this on Apple Silicon, where the emulated amd64 GUI
# deadlocks under Rosetta; see Dockerfile.gz-gui for details.
#
# It joins orca4's network namespace, so it finds the Gazebo server through
# gz-transport and draws on orca4's Xvfb display (:99), i.e. it shows up in
# the same VNC session. Build it once with:
#   docker build --platform linux/arm64 -f Dockerfile.gz-gui -t orca4-gz-gui:latest .

if ! docker ps --format '{{.Names}}' | grep -qx orca4; then
    echo "orca4 container is not running; start it with run.sh first." >&2
    exit 1
fi

docker run -d \
    --rm \
    --name orca4-gz-gui \
    --network container:orca4 \
    --ipc container:orca4 \
    orca4-gz-gui:latest
