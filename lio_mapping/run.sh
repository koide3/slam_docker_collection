#!/bin/bash
sudo docker run --net=host -it --rm \
                $@ \
                lio_mapping
