#!/bin/bash
xhost + local:root

sudo docker run -it --rm \
			--gpus all \
			-e DISPLAY=$DISPLAY \
			-v /tmp/.X11-unix:/tmp/.X11-unix \
			-v $(realpath SuMa/config):/root/catkin_ws/src/SuMa/config \
			-w /root/catkin_ws/src/SuMa/bin \
			--name glxgears \
			$@ \
			suma

# xhost - local:root
