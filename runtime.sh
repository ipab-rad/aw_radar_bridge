#!/bin/bash
# ---------------------------------------------------------------------------
# Build docker image and run ROS code for runtime or interactively with bash
# ---------------------------------------------------------------------------

# Initialise CMD as empty
CMD=""

# If an arg is defined, start container with bash
if [ -n "$1" ]; then
    CMD="bash"
fi

# Build docker image only up to prebuilt stage
DOCKER_BUILDKIT=1 docker build \
-t aw_radar_bridge_humble \
-f Dockerfile --target runtime .

# Run docker image without volumes
docker run -it --rm --net host --privileged \
-v /dev/shm:/dev/shm \
aw_radar_bridge_humble $CMD
