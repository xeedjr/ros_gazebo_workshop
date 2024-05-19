export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia glxgears

source /opt/ros/humble/setup.bash

colcon build --cmake-args -DBUILD_TESTING=ON

. ./install/setup.sh

ros2 launch ros_gz_example_bringup diff_drive.launch.py
