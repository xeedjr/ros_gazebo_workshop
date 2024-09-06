#FROM ubuntu:22.04
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu22.04

ENV LANG en_US.UTF-8

ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display
ENV DEBIAN_FRONTEND noninteractive
ENV IGNITION_VERSION fortress
ENV ROS_DISTRO humble

RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC 
RUN apt-get -y install tzdata

# Update package lists and install pre-requisites
RUN apt update && DEBIAN_FRONTEND=noninteractive && apt install -y \
    locales \
    lsb-release \
    wget \
    gnupg \
    tzdata

RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    update-locale LANG=en_US.UTF-8

# Install ROS and Gazebo keys and repositories
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null ; \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install ROS, Gazebo, and development tools
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-ros-gz \
    python3-vcstool \
    git \
    python3-pip \
    mc \
    terminator

RUN apt install -y glmark2
RUN apt install -y ros-humble-slam-toolbox ros-humble-rplidar-ros ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-v4l2-camera ros-humble-hardware-interface ros-humble-ign-ros2-control ros-humble-joint-state-broadcaster ros-humble-velocity-controllers ros-humble-diff-drive-controller


# Install ROS dependencies (ideally after ROS installation)
RUN pip3 install -U rosdep colcon-common-extensions

# Environment setup
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Clean up (optional)
RUN apt clean && rm -rf /var/lib/apt/lists/*
