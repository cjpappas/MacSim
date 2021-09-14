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
- A ROS master container, called `master`, which will be running the rosbridge suite.
- A non-ROS container, called `logic_python`, which is running python with the [roslibpy](https://github.com/gramaziokohler/roslibpy) library.
- A headless gazebo simulation (a ROS container) running the `vrx` scenario.

All images start up the required servers and begin communicating.  The `ROS_MASTER_URI` is set in the gazebo node so that it uses the master node in the `master` container.

You can connect to the master node to watch what is happening on the various topics with

```bash
docker exec -it master /bin/bash
```
You still need to introduce all the ros paths etc in this shell, you can do that be prefacing any ros commands with `/ros_entrypoint.sh <command>` of by sourcing `>source /opt/ros/noetic/setup.bash`

### For example
You can watch the location of the wam-v in this scenario with

`/ros_entrypoint.sh rostopic echo /wamv/sensors/gps/gps/fix`

## Clean-up

To stop the demonstration and clean-up the containers and network.
```bash
docker-compose down
```

# Running a simultion to connect to some other hardware/software

The `master` container binds all necessary ports to localhost when you run `docker compose up`, so you can talk to ROS directly with `localhost:11311` and to rosbridge on `localhost:9090`.  It is not yet tested, but I expect other ROS nodes on the network can set their `ROS_MASTER_URI` as necessary to connect to the `master` node running in the docker composition.
