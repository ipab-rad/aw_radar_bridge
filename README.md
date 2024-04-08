# AW Radar Bridge

Repo for bridging between internal Continental Radar msgs and standard ROS msgs.

## Info

Consists of a pubsub node which subscribes to the internal msg type, converts it
to ROS standard msgs and publishes it again.

There are multiple Radar sources, so a node should be run per input/output pair.

And a launch file to bring them all, and in the darkness bind them.

This packages is currently tailored to work with the following ipab-rad packages: [radar](https://github.com/ipab-rad/radar/tree/init_config) and [ros1_bridge](https://github.com/ipab-rad/radar/tree/init_config)
