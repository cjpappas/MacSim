#!/bin/bash

# Create ros master if not already started
rostopic list > /dev/null 2>&1
retVal=$?
if [ $retVal -ne 0 ]; then
    roscore &
    echo "Wait for 5s to allow rosmaster to start"
    sleep 5s
else
    echo "rosmaster already setup"
fi

# Add devel stuff to path
source /home/developer/vrx_ws/devel/setup.bash
# Start rosbridge server
roslaunch rosbridge_server rosbridge_websocket.launch &
# Start our program
IP=$COMPETITOR_IP node /home/macsim/src/main.js