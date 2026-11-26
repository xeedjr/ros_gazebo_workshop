#!/bin/bash

cd /root/data/project_ws

source /opt/ros/jazzy/setup.bash
source ./install/setup.sh

ros2 run stereo_camera_package stereo_calib_headless