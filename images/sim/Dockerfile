FROM npslearninglab/watery_robots:noetic_current

# Use bash
SHELL ["/bin/bash", "-c"]

# useful for debugging
RUN sudo apt-get update && sudo apt-get install -y x11-apps 

# build the vrx ROS
RUN mkdir -p ~/vrx_ws/src
RUN cd ~/vrx_ws/src && git clone https://github.com/osrf/vrx
RUN cd ~/vrx_ws/ \ 
 && source /opt/ros/noetic/setup.bash \
 && catkin_make

# run the vrx station keeping (headless) - fire up a station keeper and just have it go over and over again
CMD source ~/vrx_ws/devel/setup.bash \
 && export ROS_MASTER_URI=http://master:11311 \
 && while true; do roslaunch --wait vrx_gazebo station_keeping.launch gui:=false; done