# Use official ROS2 Humble image
FROM osrf/ros:humble-desktop

# Set build argument for Ethernet interface
ARG ETHERNET_INTERFACE
ENV ETHERNET_INTERFACE=$ETHERNET_INTERFACE

# Set the shell to bash
SHELL ["/bin/bash", "-c"]

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    software-properties-common \
    gedit \
    vim \
    sed \
    iproute2 \
    && apt-add-repository universe \
    && apt-add-repository restricted \
    && apt-add-repository multiverse \
    && apt-get update \
    && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    ros-humble-ros2-control \
    ros-humble-controller-manager \
    ros-humble-xacro \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS2 workspace
WORKDIR /root/ws/src
RUN git clone https://github.com/synapticon/synapticon_ros2_control

# Modify CMakeLists.txt to suppress warnings
RUN sed -i 's/-Wall -Wextra -Wpedantic/-Wno-pedantic -Wno-error=pedantic/g' /root/ws/src/synapticon_ros2_control/CMakeLists.txt


# Replace 'eno0' with actual Ethernet interface
RUN sed -i "s/eno0/${ETHERNET_INTERFACE}/g" /root/ws/src/synapticon_ros2_control/src/torque_control_executable.cpp
RUN sed -i "s/eno0/${ETHERNET_INTERFACE}/g" /root/ws/src/synapticon_ros2_control/description/ros2_control/single_dof.ros2_control.xacro

# Build the workspace
WORKDIR /root/ws

RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w" -DCMAKE_C_FLAGS="-w"

# Set up entrypoint to source the environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> ~/.bashrc
ENTRYPOINT ["/bin/bash"]

