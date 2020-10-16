#!/bin/bash
sudo docker run --net host -it --rm \
                -w /root/catkin_ws2/src/ethzasl_icp_mapping \
                -v $(realpath ethzasl_icp_mapping):/root/catkin_ws2/src/ethzasl_icp_mapping \
				$@ \
				ethzasl_icp
