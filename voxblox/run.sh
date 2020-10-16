#!/bin/bash
xhost + local:root

sudo docker run -it --rm \
            --net host \
			--gpus all \
			--privileged \
			-e DISPLAY=$DISPLAY \
			-v /tmp/.X11-unix:/tmp/.X11-unix \
			$@ \
			voxgraph
