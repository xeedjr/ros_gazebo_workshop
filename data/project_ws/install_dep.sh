apt update
source /opt/ros/jazzy/setup.bash
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro jazzy