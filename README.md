# slam_docker_collection

[![Build Status](https://travis-ci.org/koide3/slam_docker_collection.svg?branch=master)](https://travis-ci.org/koide3/slam_docker_collection)

This is my personal attempt to create reusable and portable environments for 3D SLAM with docker.

## Basic Usage

Build docker image:
```bash
cd slam_docker_collection/hdl_graph_slam
./build.sh
```

Run docker image:
```bash
cd slam_docker_collection/hdl_graph_slam
./run.sh -v ~/datasets:/datasets  # you can put more docker run arguments

# in docker
source /ros_entrypoint.sh
roslaunch hdl_graph_slam hdl_graph_slam_400.launch
```