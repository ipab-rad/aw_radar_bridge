FROM ros:humble-ros-base-jammy AS base

# Install key dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends install \
        # Install ROS msg dependencies
        ros-"$ROS_DISTRO"-radar-msgs \
        ros-"$ROS_DISTRO"-sensor-msgs \
        # Install Cyclone DDS ROS RMW
        ros-"$ROS_DISTRO"-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace folders
ENV ROS_WS=/opt/ros_ws
WORKDIR $ROS_WS

# Set cyclone DDS ROS RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# -----------------------------------------------------------------------

FROM base AS prebuilt

# Copy ROS2 msg files and bridge code over
COPY ecal_to_ros/ros2/ src/ecal_to_ros/
COPY aw_radar_bridge  src/aw_radar_bridge

# Source ROS2 setup for dependencies and build our code
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# -----------------------------------------------------------------------

FROM base AS dev

# Install basic dev tools (And clean apt cache afterwards)
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends install \
        # Command-line editor
        nano \
        # Ping network tools
        inetutils-ping \
        # Bash auto-completion for convenience
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Add sourcing local workspace command to bashrc when running interactively
# Add colcon build alias for convenience
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo 'alias colcon_build="colcon build --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release && \
            source install/setup.bash"' >> /root/.bashrc

# Enter bash for development
CMD ["bash"]

# -----------------------------------------------------------------------

FROM base AS runtime

COPY --from=prebuilt $ROS_WS/install $ROS_WS/install

# Add command to docker entrypoint to source newly compiled code in container
RUN sed --in-place --expression \
      "\$isource \"$ROS_WS/install/setup.bash\" " \
      /ros_entrypoint.sh

# Launch ros package
CMD ["ros2", "launch", "aw_radar_bridge", "aw_radar_bridge.launch.xml"]
