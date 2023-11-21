#!/usr/bin/env bash

IMAGE=kyon
NAME=birb

docker run -it --rm \
            --privileged \
            --runtime=nvidia --gpus all \
            --env NVIDIA_DRIVER_CAPABILITIES=all \
            -v /dev/input:/dev/input \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            --env=DISPLAY \
            --name ${NAME} ${IMAGE}
            # --network="host" \
