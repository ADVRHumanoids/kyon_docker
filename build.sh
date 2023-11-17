#!/usr/bin/env bash

IMAGE=kyon

docker build --build-arg CACHE_DATE="$(date)"--rm -t ${IMAGE} . #--progress=plain --no-cache