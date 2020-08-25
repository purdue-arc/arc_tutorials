#!/bin/bash

docker exec -it \
    -e DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT \
    arc-dev \
    /bin/bash
