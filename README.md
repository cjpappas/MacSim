# ROS Demonstration

This repository contains the docker images and code to run a demonstration of a [ROS](http://wiki.ros.org) container (representing a device without the ROS environment) talking to a ROScontainer (representing a ROS master). It utilises the [Rosbridge suite]((http://wiki.ros.org/rosbridge_suite)) to allow communication between ROS and non-ROS devices. 

## Requirements
- [Docker](https://www.docker.com)
- [Docker Compose](https://docs.docker.com/compose/install/)

## Running The Demo

Docker compose should take care of the setup.
```bash
docker-compose up -d
```

This will create:
- A docker network, ros-network, which will act as the network both devices are connected to.
- A ROS master container, called ros-master, which will be running the rosbridge suite.
- A non-ROS container, called no-ros, which is running python with the [roslibpy](https://github.com/gramaziokohler/roslibpy) library.

## Clean-up

To stop the demonstration and clean-up the containers and network.
```bash
docker-compose down
```
