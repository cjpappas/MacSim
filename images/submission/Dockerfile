# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-base

# install ros packages
RUN apt-get update && apt-get install -y ros-noetic-ros-base ros-noetic-rosbridge-suite nodejs npm \
    && rm -rf /var/lib/apt/lists/*

# Copy over script to Docker container
COPY images/submission/run_my_system.bash /

# Use your ros_entrypoint
COPY images/submission/ros_entrypoint.sh /

# Copy over src
RUN mkdir /home/macsim
RUN cd /home/macsim && npm install three axios roslib
COPY images/submission/src /home/macsim/src