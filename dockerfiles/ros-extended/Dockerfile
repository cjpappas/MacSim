FROM ros:noetic-ros-core

# Use bash
SHELL ["/bin/bash", "-c"]

# Install deps
RUN sudo apt-get update
RUN sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential iputils-ping -y