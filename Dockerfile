FROM ubuntu:24.04
#FROM nvidia/opengl:1.0-glvnd-devel-ubuntu24.04

LABEL versio=test

ENV LANG en_US.UTF-8

ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display
ENV DEBIAN_FRONTEND noninteractive
ENV GZ_VERSION harmonic
ENV ROS_DISTRO jazzy

RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC 
RUN apt-get -y install tzdata

# Update package lists and install pre-requisites
RUN apt update && DEBIAN_FRONTEND=noninteractive && apt install -y \
    locales \
    lsb-release \
    wget \
    gnupg \
    curl \
    tzdata

RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    update-locale LANG=en_US.UTF-8

# Install ROS and Gazebo keys and repositories
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install ROS, Gazebo, and development tools
RUN apt update && apt install -y \
    gz-harmonic \
    ros-dev-tools \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    python3-vcstool \
    git \
    python3-pip \
    mc \
    terminator

RUN apt install -y glmark2 mc
RUN apt install -y ros-jazzy-slam-toolbox ros-jazzy-rplidar-ros ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-v4l2-camera ros-jazzy-hardware-interface ros-jazzy-gz-ros2-control ros-jazzy-joint-state-broadcaster ros-jazzy-velocity-controllers ros-jazzy-diff-drive-controller ros-jazzy-foxglove-bridge


# Install ROS dependencies (ideally after ROS installation)
RUN pip3 install --break-system-packages -U rosdep colcon-common-extensions

RUN pip3 install --break-system-packages -U RPI.GPIO adafruit-circuitpython-bno08x adafruit-circuitpython-busdevice

# Environment setup
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

# Clean up (optional)
RUN apt clean && rm -rf /var/lib/apt/lists/*

