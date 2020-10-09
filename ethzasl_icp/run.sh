#!/bin/bash
sudo docker run --net host -it --rm \
				$@ \
				ethzasl_icp
