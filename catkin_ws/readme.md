# README

This folder contains the talker/listener example from the [ROS website](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29). The idea is to demonstrate the example running over a network (in this case using docker).

## What you need

- Docker

## Setup

You first need to create a docker network for the containers to live in.
```bash
docker network create ros-network
```

Once this is done you need to pull the [ros:noetic-ros-core](https://hub.docker.com/layers/ros/library/ros/noetic-ros-core/images/sha256-45f08fb5b042970e1b561d9b3940fdabc46b34958678536c54a07e17c3a253cf?context=explore) image.
```bash
docker pull ros:noetic-ros-core
```

You then need to create two containers, a master and a slave, which are attached to the docker network with the current directory mounted.
```bash
# Master
docker run -itd --network=ros-network --name=ros-master -v <path_to_catkin_ws>:/home/catkin_ws ros:noetic-ros-core

# Slave
docker run -itd --network=ros-network --name=ros-slave -v <path_to_catkin_ws>:/home/catkin_ws ros:noetic-ros-core
```

You can then connect to each of the containers.
```bash
docker exec -it ros-<master/slave> /bin/bash
```

Once connected to the containers you need to install dependencies and install ros on the PATH.
**This needs to be done on BOTH containers**
```bash
sudo apt-get update && sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential iputils-ping

source /opt/ros/noetic/setup.bash

cd /home/catkin_ws
catkin_make

source /home/catkin_ws/devel/setup.bash
```

## Running the example over the network

First you need to obtain the IP address of both the master and slave on the docker network.
```bash
docker network inspect ros-network
```

You can then check the connection between the two containers by connecting to each container and pinging the other.
```bash
ping <master/slave_ip>
```
The two machine need to successfully ping each other for this demo to work.

On the **master** node, you can start `roscore`. From the output keep track of the `ROS_MASTER_URI`.
```bash
docker exec -it ros-master /bin/bash

roscore
```
You can then open another terminal window and connect to the **master** node again and start the listener.
```bash
docker exec -it ros-master /bin/bash

source /opt/ros/noetic/setup.bash
source /home/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=<ros_master_uri_from_roscore>
rosrun demo listener.py
```
You can then open another terminal window and connect to the **slave** node and start the talker.
```bash
docker exec -it ros-slave /bin/bash

source /opt/ros/noetic/setup.bash
source /home/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=<docker_network_ip_of_master:port_from_ros_master_uri>
rosrun demo talker.py
```

The talker should start sending "hello world" messages, and the listner should "hear" these messages.