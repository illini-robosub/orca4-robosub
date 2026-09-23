#!/usr/bin/env bash

# --gpus all (and X11 forwarding to a real GPU) requires an NVIDIA container
# runtime on the host. Without one -- e.g. under OrbStack/Docker Desktop on
# macOS, where XQuartz's indirect GLX can't serve modern GL contexts -- fall
# back to rendering in-container against a virtual framebuffer and serving
# the display over VNC instead.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

HAS_GPU=false
if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
    HAS_GPU=true
fi

if [ "$HAS_GPU" = true ]; then
    XAUTH=/tmp/.docker.xauth
    if [ ! -f $XAUTH ]
    then
        xauth_list=$(xauth nlist $DISPLAY)
        xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
        if [ ! -z "$xauth_list" ]
        then
            echo "$xauth_list" | xauth -f $XAUTH nmerge -
        else
            touch $XAUTH
        fi
        chmod a+r $XAUTH
    fi

    echo "NVIDIA GPU runtime detected -- forwarding the host X11 display."
    docker run -it \
        --rm \
        --name orca4 \
        -e DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=$XAUTH \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -v "$XAUTH:$XAUTH" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix" \
        -v "/etc/localtime:/etc/localtime:ro" \
        -v "/dev/input:/dev/input" \
        --privileged \
        --security-opt seccomp=unconfined \
        --gpus all \
        orca4:latest
else
    echo "No NVIDIA GPU runtime detected -- rendering in-container with a"
    echo "virtual framebuffer and serving the display over VNC on port 5900"
    echo "instead of forwarding X11."
    # --ipc shareable lets run-gz-gui.sh's native Gazebo GUI join this IPC
    # namespace; Mesa hands frames to Xvfb via MIT-SHM, which only works when
    # both sides see the same shared-memory segments.
    # The entrypoint is mounted from this directory so edits to it apply on
    # the next run without rebuilding the image.
    docker run -it \
        --rm \
        --name orca4 \
        -p 127.0.0.1:5900:5900 \
        --ipc shareable \
        -v "/etc/localtime:/etc/localtime:ro" \
        -v "/dev/input:/dev/input" \
        -v "$DIR/vnc-entrypoint.sh:/usr/local/bin/vnc-entrypoint.sh:ro" \
        --privileged \
        --security-opt seccomp=unconfined \
        orca4:latest /usr/local/bin/vnc-entrypoint.sh
fi
