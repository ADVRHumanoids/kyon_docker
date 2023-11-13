#!/usr/bin/env bash

IMAGE=kyon

docker build --progress=plain --build-arg CACHE_DATE="$(date)"--rm -t ${IMAGE} . #--no-cache