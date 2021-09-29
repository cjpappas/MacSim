FROM npslearninglab/watery_robots:noetic_current

# Use bash
SHELL ["/bin/bash", "-c"]
RUN mkdir -p ~/vrx_ws/src
RUN cd ~/vrx_ws/src && git clone https://github.com/osrf/vrx
RUN cd ~/vrx_ws/ \ 
 && source /opt/ros/noetic/setup.bash \
 && catkin_make
CMD source ~/vrx_ws/devel/setup.bash \
 && export ROS_MASTER_URI=http://master:11311 \
 && while true; do roslaunch --wait vrx_gazebo station_keeping.launch gui:=false; done