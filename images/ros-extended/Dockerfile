FROM ros:noetic-ros-core

# Use bash
SHELL ["/bin/bash", "-c"]

# Install deps
RUN sudo apt-get update
RUN sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential iputils-ping ros-noetic-rosbridge-suite python3-pip -y
RUN pip install roslibpy

# Add ros suite to path
RUN source /opt/ros/noetic/setup.bash

# Copy listener script across
COPY listener.py /home/