FROM ubuntu:jammy
ARG DEBIAN_FRONTEND=noninteractive

# Update Packages
RUN apt-get update
RUN apt-get install -y apt-utils
RUN apt-get install locales && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8

# Install ROS Package Keys
RUN apt-get install -y curl gnupg2 apt-transport-https gnupg ca-certificates lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package repo for ROS
RUN apt-get update

# Install ROS
RUN apt-get install -y ros-humble-desktop ros-humble-demo-nodes-cpp
RUN apt-get install -y python3-argcomplete wget unzip nano build-essential

RUN apt-get install -y vim

RUN apt-get install -y ros-humble-realsense2-camera
# Configure Enviroment for ROS
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

COPY realsense_launch.py realsense_launch.py


CMD /bin/bash
