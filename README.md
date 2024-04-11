# AW Radar Bridge

Repo for bridging between internal Continental Radar msgs and standard ROS msgs.

## Info

Consists of a pubsub node which subscribes to the internal msg type, converts it
to ROS standard msgs and publishes it again.

There are multiple Radar sources, so a node should be run per input/output pair.

And a launch file to bring them all, and in the darkness bind them.

This packages is currently tailored to work with the following ipab-rad packages: [radar](https://github.com/ipab-rad/radar) and [ros1_bridge](https://github.com/ipab-rad/ros1_bridge)

## Usage

This repository is designed to be used alongside a Docker container. Quickly build and run the Docker container using `runtime.sh` for runtime or debugging, and `dev.sh` for a convenient development setup.

### Runtime or Debugging

Execute the ROS 2 nodes in runtime mode or start an interactive bash session for detailed debugging:

```bash
./runtime.sh [bash]
```

- **Without arguments**: Activates the container in runtime mode.
- **With `bash`**: Opens an interactive bash session for debugging.

### Development

Prepare a development setting that reflects local code modifications and simplifies the build process:

```bash
./dev.sh
```

- **Live Code Synchronization**: Mounts local `aw_radar_bridge` and `ecal_to_ros` directories with the container.
- **Convenience Alias**: The development container features a `colcon_build` alias, which simplifies the ROS2 build process. Executing `colcon_build` runs `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` and then sources the `setup.bash` to ensure the environment is updated with the latest build artifacts. This alias enhances productivity by combining build commands and environment setup into a single, easy command.