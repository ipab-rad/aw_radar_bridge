^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aw_radar_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial Radar configuration PR (`#1 <https://github.com/ipab-rad/av_radar_bridge/issues/1>`_)
  Setup Conti->Autoware radar msg pubsub bridge
  Pkg launches 16 composable nodes that subscribe to Continental's radar
  ecal_to_ros RadarDetectionImage msgs and publish the ROS standard
  radar_msgs RadarScan which Autoware consumes
* Upgrade Dockerfile w/ stages & convenient scripts
* Initial repo setup and WIP pubsub implementation
* Contributors: Alejandro Bordallo, Hector Cruz
