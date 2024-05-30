FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND noninteractive
ENV IGNITION_VERSION fortress
ENV ROS_DISTRO humble
ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

LABEL version="1.0"

# Workaround https://unix.stackexchange.com/questions/2544/how-to-work-around-release-file-expired-problem-on-a-local-mirror
RUN echo "Acquire::Check-Valid-Until \"false\";\nAcquire::Check-Date \"false\";" | cat > /etc/apt/apt.conf.d/10no--check-valid-until

RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC 
RUN apt-get -y install tzdata

RUN apt-get update
RUN apt-get install -y lsb-release wget gnupg

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y ignition-fortress

RUN apt install -y terminator

RUN locale  # check for UTF-8

RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN locale  # verify settings

RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update

RUN apt upgrade -y 

RUN apt install -y ros-humble-desktop

RUN apt-get install -y ros-humble-ros-gz

RUN apt-get install -y mc

RUN apt install -y python3-vcstool

RUN apt install -y git

RUN apt-get install -y python3-rosdep
RUN apt-get -y install python3-pip

RUN pip3 install -U colcon-common-extensions

RUN apt install -y ros-humble-rmw-cyclonedds-cpp

RUN apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

RUN apt install -y ros-humble-turtlebot3*

RUN apt install -y ros-humble-slam-toolbox

RUN apt install -y ros-humble-rosbridge-suite

RUN apt-get install -y lshw nvidia-prime nvidia-settings

#ORB SLAM

# Build Pangolin
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.9.1 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14 -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j8 && make install && \
    cd / && rm -rf /tmp/Pangolin

RUN git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3

WORKDIR "/ORB_SLAM3"

RUN chmod +x build.sh

RUN ./build.sh

WORKDIR "/root"

RUN apt install -y ros-humble-vision-opencv && sudo apt install -y ros-humble-message-filters

RUN mkdir -p colcon_ws/src
WORKDIR "/root/colcon_ws/src"
RUN git clone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
WORKDIR "/root/colcon_ws/src/orbslam3_ros2"
RUN git checkout humble
WORKDIR "/root/colcon_ws"
# Source the ROS 2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]
RUN apt-get install ros-humble-sophus
RUN apt install -y python3-ament-package

WORKDIR /ORB_SLAM3/Thirdparty/Sophus/build
RUN make install
#RUN colcon build --symlink-install --packages-select orbslam3

WORKDIR "/root"


#https://github.com/zang09/ORB_SLAM3_ROS2/tree/humble