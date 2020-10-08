#!/bin/bash
sudo docker run --net=host -it --rm -w /root -v $(realpath loam_velodyne):/root/catkin_ws/src/loam_velodyne $@ loam
