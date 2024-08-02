apt update
source /opt/ros/humble/setup.bash
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble