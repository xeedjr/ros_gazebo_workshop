source /opt/ros/humble/setup.bash

colcon build --cmake-args -DBUILD_TESTING=ON

. ./install/setup.sh

ros2 launch ros_gz_example_bringup diff_drive.launch.py