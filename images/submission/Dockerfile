FROM altmattr/simulation:latest

# Use bash
SHELL ["/bin/bash", "-c"]

# Copy over src
RUN mkdir /home/developer/submission
RUN cd /home/developer/submission && npm install three axios roslib
COPY images/submission/src /home/developer/submission/src

# run the vrx station keeping (headless) - fire up a station keeper and just have it go over and over again
CMD source /home/developer/vrx_ws/devel/setup.bash && (roslaunch rosbridge_server rosbridge_websocket.launch &) && \
    (rosrun web_video_server web_video_server &) && \
    sleep 8s;\
    node /home/developer/submission/src/main.js