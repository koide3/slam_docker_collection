FROM ros:kinetic

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
 && apt-get install -y --no-install-recommends wget nano build-essential \
                                               ros-kinetic-pcl-ros ros-kinetic-tf-conversions ros-kinetic-eigen-conversions ros-kinetic-map-msgs ros-kinetic-interactive-markers \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /root


# libnano & libpointmatcher
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace'

RUN git clone https://github.com/ethz-asl/libnabo.git
RUN git clone https://github.com/ethz-asl/libpointmatcher.git

# build
WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make_isolated -DCMAKE_BUILD_TYPE=Release'

# ethzasl_icp_mapping
WORKDIR /root/catkin_ws2/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace'

# RUN git clone https://github.com/ethz-asl/ethzasl_icp_mapping
COPY ethzasl_icp_mapping /root/catkin_ws2/src/ethzasl_icp_mapping/

# build
WORKDIR /root/catkin_ws2
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; . /root/catkin_ws/devel_isolated/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release'

RUN sed -i "6i source \"/root/catkin_ws/devel_isolated/setup.bash\"" /ros_entrypoint.sh
RUN sed -i "7i source \"/root/catkin_ws2/devel/setup.bash\"" /ros_entrypoint.sh


WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
