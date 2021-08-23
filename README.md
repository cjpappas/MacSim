# ROS Demonstration

This repository contains the docker images and code to run a demonstration of a [ROS](http://wiki.ros.org) container (representing a device without the ROS environment) talking to a ROScontainer (representing a ROS master). It utilises the [Rosbridge suite]((http://wiki.ros.org/rosbridge_suite)) to allow communication between ROS and non-ROS devices. 

## Requirements
- [Docker](https://www.docker.com)
- [Docker Compose](https://docs.docker.com/compose/install/)

## Running The Demo

Docker compose should take care of the setup.
```bash
docker compose up -d
```
OR
```bash
docker-compose up -d
```

This will create:
- A docker network, ros-network, which will act as the network both devices are connected to.
- A ROS master container, called ros-master, which will be running the rosbridge suite.
- A non-ROS container, called no-ros, which is running python with the [roslibpy](https://github.com/gramaziokohler/roslibpy) library.

Next we need to get the rosbridge server running on the master.
```bash
docker exec -it ros-master /bin/bash

source /opt/ros/noetic/setup.bash

roslaunch rosbridge_server rosbridge_websocket.launch
```
In a seperate terminal we can run the listener.
```bash
docker exec -it ros-master /bin/bash

python3 /home/listener.py
```
Finally, in another terminal, we can start the talker.

**Note**: Make sure the host in the talker script matches the address of the ros-master container on the ros-network!
```bash
docker exec -it no-ros /bin/bash

python /home/talker.py
```
The containers should now be talking to each other :).

## Clean-up

To stop the demonstration and clean-up the containers and network.
```bash
docker-compose down
```
