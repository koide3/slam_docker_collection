#!/bin/bash
sudo docker run --net=host -it --rm \
                -v $(realpath loam_velodyne):/root/catkin_ws/src/loam_velodyne \
                -w /root/catkin_ws/src/loam_velodyne \
                $@ \
                loam
