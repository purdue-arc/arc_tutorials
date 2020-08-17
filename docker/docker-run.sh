#!/bin/bash

WS_DIR="$(readlink -f $(dirname $0)/../../../)"
echo "mounting host directory $WS_DIR as container directory /home/$USER/catkin_ws"

docker run --rm -it \
    -e DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT \
    -v $XAUTH:/home/$USER/.Xauthority \
    -v $WS_DIR:/home/$USER/catkin_ws \
    --hostname arc-dev \
    --name arc-dev \
    arc-dev \
    /bin/bash
