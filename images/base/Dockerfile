FROM npslearninglab/watery_robots:noetic_current

# Use bash
SHELL ["/bin/bash", "-c"]

# useful for debugging
RUN sudo apt-get update && sudo apt-get install -y x11-apps ros-noetic-rosbridge-suite


# build the vrx ROS
RUN mkdir -p ~/vrx_ws/src
RUN cd ~/vrx_ws/src && git clone https://github.com/osrf/vrx

# put our custom things in
COPY src /home/developer/vrx_ws/src

# build it
RUN cd ~/vrx_ws/ \ 
 && source /opt/ros/noetic/setup.bash \
 && catkin_make

# bring over the html to serve
RUN mkdir /home/developer/html
ADD html /home/developer/html

# run the vrx station keeping (headless) - fire up a station keeper and just have it go over and over again
CMD source ~/vrx_ws/devel/setup.bash && while true; do roslaunch vrx_gazebo station_keeping.launch gui:=false; done & \
    source ~/vrx_ws/devel/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch & \
    python3 -m http.server -d /home/developer/html 80

## Should be built with
# docker compose build simulation
## Should be run with 
# docker run -p 9090:9090 -p 80:80 altmattr/simulation
