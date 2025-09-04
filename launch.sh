#!/bin/sh

docker run --rm -it \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
    --env="WAYLAND_DISPLAY=$WAYLAND_DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY" \
    --device=/dev/dri:/dev/dri \
    --group-add=video \
    --name ros_jazzy \
    NoSignificantWorries/my-ros-jazzy:latest \
    /bin/bash

