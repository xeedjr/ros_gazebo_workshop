#!/bin/bash

cd /root/data/project_ws

apt update
source /opt/ros/humble/setup.bash
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble

source /opt/ros/humble/setup.bash

colcon build --cmake-args -DBUILD_TESTING=ON

. ./install/setup.sh

ros2 launch ros_gz_example_bringup diff_drive.launch.py