#!/bin/bash
sudo docker run --net=host -it --rm \
                -v $(realpath LINS---LiDAR-inertial-SLAM):/root/catkin_ws/src/LINS---LiDAR-inertial-SLAM \
                -w /root/catkin_ws/src/LINS---LiDAR-inertial-SLAM \
                $@ \
                lins
