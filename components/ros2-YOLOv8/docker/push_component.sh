#!/bin/sh


export DOCKER_CLI_EXPERIMENTAL=enabled


#
# on the terminal (inside the folder of the Dockerfile
# 
docker buildx create --name nimbus-builder 
docker buildx use nimbus-builder
docker run --privileged --rm tonistiigi/binfmt --install all
docker buildx inspect --bootstrap


#
# on the terminal (inside the folder of the Dockerfile
#


docker buildx build --platform linux/arm64 -t cognimbus/yolo_detector:foxy --push .


