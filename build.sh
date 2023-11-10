#!/usr/bin/env bash

IMAGE=kyon

docker build --progress=plain --no-cache --build-arg CACHE_DATE="$(date)"--rm -t ${IMAGE} .