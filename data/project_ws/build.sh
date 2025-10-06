#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Cyclone prefers multicast by default, if your router got too much spammed, 
# disable multicast with (https://github.com/ros2/rmw_cyclonedds/issues/489):
export CYCLONEDDS_URI="<Disc><DefaultMulticastAddress>0.0.0.0</></>"


source /opt/ros/jazzy/setup.bash

colcon build --cmake-args -DBUILD_TESTING=ON

. ./install/setup.sh

ros2 launch ros_gz_example_bringup diff_drive.launch.py