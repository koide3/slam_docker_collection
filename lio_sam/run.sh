#!/bin/bash
sudo docker run --net=host -it --rm \
                -w /root/catkin_ws/src/LINS---LiDAR-inertial-SLAM \
                $@ \
                lio_sam
