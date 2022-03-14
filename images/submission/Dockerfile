FROM altmattr/simulation
# Copy over src
RUN mkdir /home/macsim
RUN cd /home/macsim && npm install three axios roslib
COPY images/submission/src /home/macsim/src

# run the vrx station keeping (headless) - fire up a station keeper and just have it go over and over again
CMD source /home/developer/vrx_ws/devel/setup.bash && (roslaunch rosbridge_server rosbridge_websocket.launch &) && \
    (rosrun web_video_server web_video_server &) && \
    sleep 8s \
    node /home/macsim/src/main.js