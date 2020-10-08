#!/bin/bash
sudo docker run --net=host -it --rm -w /root -v $(realpath hdl_graph_slam):/root/catkin_ws/src/hdl_graph_slam $@ hdl_graph_slam
