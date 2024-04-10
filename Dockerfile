FROM ros:humble-ros-base-jammy AS base

# Install key dependencies
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive \
        apt -y --quiet --no-install-recommends install \
        # Install ROS msg dependencies
        ros-$ROS_DISTRO-radar-msgs \
        ros-$ROS_DISTRO-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace folders
ENV ROS_WS=/opt/ros_ws
WORKDIR $ROS_WS

# Copy ROS2 msg files and bridge code over
COPY ecal_to_ros/ros2/ src/ecal_to_ros/
COPY aw_radar_bridge  src/aw_radar_bridge

# -----------------------------------------------------------------------

FROM base AS prebuilt

# Source ROS2 setup for dependencies and build our code
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Add command to docker entrypoint to source newly compiled code when running docker container
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# -----------------------------------------------------------------------

FROM base AS dev

# Install basic dev tools (And clean apt cache afterwards)
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive \
        apt -y --quiet --no-install-recommends install \
        # Command-line editor
        nano \
        # Ping network tools
        inetutils-ping \
        # Bash auto-completion for convenience
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Add sourcing local workspace command to bashrc for convenience when running interactively
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Add colcon build alias for convenience
RUN echo 'alias colcon_build="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash"' >> /root/.bashrc

# Enter bash for development
CMD ["bash"]

# -----------------------------------------------------------------------

# Note: Ideally this runtime stage should inherit from base and only copy relevant artifacts to it
FROM prebuilt AS runtime

# Launch ros package
CMD ["ros2", "launch", "aw_radar_bridge", "aw_radar_bridge.launch.xml"]
