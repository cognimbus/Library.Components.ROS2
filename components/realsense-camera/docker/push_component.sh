#!/bin/sh


export DOCKER_CLI_EXPERIMENTAL=enabled


#
# on the terminal (inside the folder of the Dockerfile
# 
sudo docker buildx create --name nimbus-builder 
sudo docker buildx use nimbus-builder
sudo docker run --privileged --rm tonistiigi/binfmt --install all
sudo docker buildx inspect --bootstrap


#
# on the terminal (inside the folder of the Dockerfile
#


sudo docker buildx build --platform linux/arm64,linux/amd64 -t cognimbus/ros2-realsense-camera:humble --push .