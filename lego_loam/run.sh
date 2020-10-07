#!/bin/bash
sudo docker run --net=host -it --rm -w /root -v $(realpath LeGO-LOAM-BOR):/root/catkin_ws/src/LeGO-LOAM-BOR $@ lego_loam
