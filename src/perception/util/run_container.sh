#!/usr/bin/env bash


# give docker permission to use X
xhost +si:localuser:root

# run container with privilege mode, host network, display, and mount workspace on host
docker run --rm --runtime nvidia -it \
    --name ada_perception_container \
    --ipc=host \
    --shm-size=6g \
    --privileged --network host -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix -v /dev:/dev \
    -v /lib/aarch64-linux-gnu/libffi.so.8.1.0:/lib/aarch64-linux-gnu/libffi.so.8.1.0:ro \
    -v /lib/aarch64-linux-gnu/libffi.so.8:/lib/aarch64-linux-gnu/libffi.so.8:ro \
    -v /dev/shm:/dev/shm \
    -v $HOME/NavOS/src/perception:/perception_ws/src \
    williamhecoin/ada_academy:perception2025v1
