#!/bin/bash
echo "running my script"
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

# Add devel stuff to path TODO: not working since that file is not there

source /home/developer/vrx_ws/devel/setup.bash
# Start rosbridge server
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 8s
# Start our program
printenv
node /home/macsim/src/main.js