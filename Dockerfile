FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND noninteractive
ENV IGNITION_VERSION fortress
ENV ROS_DISTRO humble
ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

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