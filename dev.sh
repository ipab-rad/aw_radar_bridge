#!/bin/bash
###############################################################################
# Build docker dev stage and add local code for live development              #
###############################################################################

# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
-t aw_radar_bridge_humble \
-f Dockerfile --target dev .

# Run docker image with local code volumes for development
docker run -it --rm --net host \
-v /dev/shm:/dev/shm \
-v ./ecal_to_ros/ros2:/opt/ros_ws/src/ecal_to_ros \
-v ./aw_radar_bridge:/opt/ros_ws/src/aw_radar_bridge \
aw_radar_bridge_humble
