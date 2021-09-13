FROM ros:noetic-ros-core

# Use bash
SHELL ["/bin/bash", "-c"]

RUN apt-get update && sudo apt-get install -y ros-noetic-rosbridge-suite
CMD source /opt/ros/noetic/setup.bash \
 && roslaunch rosbridge_server rosbridge_websocket.launch