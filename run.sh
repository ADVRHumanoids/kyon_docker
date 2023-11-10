#!/usr/bin/env bash

IMAGE=kyon

docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix --env=DISPLAY --network="host" ${IMAGE}