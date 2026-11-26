#!/bin/bash

cd /root/data/project_ws

source /opt/ros/jazzy/setup.bash
source ./install/setup.sh

ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 9x6 --square 0.04 --no-service-check right:=/stereo/stereo_camera/right/image_raw_raw left:=/stereo/stereo_camera/left/image_raw_raw right_camera:=/stereo/right left_camera:=/stereo/left