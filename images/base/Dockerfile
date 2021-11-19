FROM npslearninglab/watery_robots:noetic_current

# Use bash
SHELL ["/bin/bash", "-c"]

# useful for debugging
RUN sudo apt-get update && sudo apt-get install -y x11-apps ros-noetic-rosbridge-suite pip

# Install python deps. for server
RUN pip install Flask

# build the vrx ROS
RUN mkdir -p ~/vrx_ws/src
RUN cd ~/vrx_ws/src && git clone https://github.com/osrf/vrx

# put our custom things in
COPY images/base/src /home/developer/vrx_ws/src

# build it
RUN cd ~/vrx_ws/ \ 
 && source /opt/ros/noetic/setup.bash \
 && catkin_make

# bring over the html to serve
RUN mkdir /home/developer/html
ADD images/base/html /home/developer/html

# run the vrx station keeping (headless) - fire up a station keeper and just have it go over and over again
CMD source ~/vrx_ws/devel/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch & \
    export FLASK_APP=/home/developer/html/server.py && python3 -m flask run --host=0.0.0.0 --port=80 

## Should be built with
# docker compose build simulation
## Should be run with 
# docker run -p 9090:9090 -p 80:80 altmattr/simulation
