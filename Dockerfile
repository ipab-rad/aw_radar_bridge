FROM ros:humble-ros-base-jammy

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
        # Install ROS msg dependencies
        ros-$ROS_DISTRO-radar-msgs \
        ros-$ROS_DISTRO-sensor-msgs \
        ros-$ROS_DISTRO-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS workspace folder
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

# Source ROS setup for dependencies and build our code
# RUN . /opt/ros/$ROS_DISTRO/setup.sh \
#     && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Add command to docker entrypoint to source newly compiled code when running docker container
# RUN sed --in-place --expression \
#       '$isource "$ROS_WS/install/setup.bash"' \
#       /ros_entrypoint.sh

# # Add sourcing local workspace command to bashrc for convenience when running interactively
# RUN echo "source $ROS_WS/install/setup.bash" >> /root/.bashrc

# launch ros package
# CMD TODO
