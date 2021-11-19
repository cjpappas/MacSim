# MacSim

This repository contains a docker image to run a [vrx gazebo simulation](https://github.com/osrf/vrx). This simulation uses [ROS](http://wiki.ros.org) for communication. In addition, we use the [Rosbridge suite]((http://wiki.ros.org/rosbridge_suite)) to allow for communication between non-ros clients such as the browser or a [node](https://nodejs.org/en/) applicaiton.

## Requirements
- [Docker](https://www.docker.com)

## Running The Image
To build image locally, clone this repository build the image in `images/base`:
```bash
git clone https://github.com/cjpappas/MacSim.git
docker build -f images/base/Dockerfile -t altmattr/simulation .
```
Alternatively, you can pull the latest version from [Dockerhub](https://hub.docker.com/). This is done automatically if the image isn't found locally.

To run the image:
```bash
docker run -itd \
    --name sim \
    -p 9090:9090 \
    -p 8080:8080 \
    -p 80:80 \
    altmattr/simulation
```

You can connect to the image and source ROS with:
```bash
docker exec -it sim /bin/bash
source /opt/ros/noetic/setup.bash
source ~/vrx_ws/devel/setup.bash
```

# Connecting to the Simulation

The container hosts a small webserver that allows you to connect to the simulation via a webpage. To access the simulation you can navigate to `http://localhost` or you can open the `image/base/html/hud/html` in the browser and it will provide a visula overview of the simualtion. You can also connect to the simulation from other programs such as node. Detailed documentation on the api can be found on the repositories wiki [here](https://github.com/cjpappas/MacSim/wiki/api).

# Troubleshooting

If your docker build fails, try increasing the memory and the swap file available to docker (docker desktop > preferences)
