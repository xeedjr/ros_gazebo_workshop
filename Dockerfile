FROM ubuntu:22.04

ENV LANG en_US.UTF-8

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

# Install ROS dependencies (ideally after ROS installation)
RUN pip3 install -U rosdep colcon-common-extensions

# Environment setup
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Clean up (optional)
RUN apt clean && rm -rf /var/lib/apt/lists/*
