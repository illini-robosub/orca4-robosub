#!/usr/bin/env bash

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

# Specific for NVIDIA drivers, required for OpenGL >= 3.3
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
    -v "/usr/local/cuda:/usr/local/cuda" \
    -v "/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu" \
    -v "/tmp:/tmp" \
    --privileged \
    --runtime=nvidia \
    --security-opt seccomp=unconfined \
    --gpus all \
    --device=/dev/video0 \
    --device=/dev/video1 \
    --device=/dev/bus/usb \
    --device=/dev/nvhost-vi \
    --device=/dev/nvhost-nvmap \
    --device=/dev/nvhost-vic \
    --device=/dev/nvhost-isp \
    --device=/dev/nvhost-t194-nvhost-ctxsw-gpu \
    --device=/dev/nvhost-gpu \
    --device=/dev/nvhost-as-gpu \
    --device=/dev/nvmap \
    --device=/dev/nvhost-nvdec \
    --device=/dev/nvhost-nvenc \
    --device=/dev/nvhost-nvcsi \
    --device=/dev/nvhost-msenc \
    --device=/dev/nvhost-ctrl \
    --ipc=host \
    orca4:latest
