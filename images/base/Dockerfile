FROM npslearninglab/watery_robots:noetic_current

# Use bash
SHELL ["/bin/bash", "-c"]

# useful for debugging
RUN sudo apt-get update && sudo apt-get install -y x11-apps ros-noetic-rosbridge-suite pip xvfb ros-noetic-web-video-server nodejs npm

# Install python deps. for server
RUN pip install Flask

# Install npm deps for running local programs inside container
RUN npm install -g three axios roslib

# Setup fake display so gazebo publishes camera info
# https://github.com/PX4/PX4-containers/issues/198
RUN Xvfb :1 --screen 0 1600x1200x16 &
RUN export DISPLAY=:1.0

# build the vrx ROS
RUN mkdir -p /home/developer/vrx_ws/src
RUN cd /home/developer/vrx_ws/src && git clone https://github.com/osrf/vrx

# put our custom things in
COPY images/base/src /home/developer/vrx_ws/src

# build it
RUN cd /home/developer/vrx_ws/ \ 
 && source /opt/ros/noetic/setup.bash \
 && catkin_make

# bring over the html to serve
RUN mkdir /home/developer/html
ADD images/base/html /home/developer/html

# run the vrx station keeping (headless) - fire up a station keeper and just have it go over and over again
CMD source /home/developer/vrx_ws/devel/setup.bash && (roslaunch rosbridge_server rosbridge_websocket.launch &) && \
    (rosrun web_video_server web_video_server &) && \
    export FLASK_APP=/home/developer/html/server.py && python3 -m flask run --host=0.0.0.0 --port=80 

