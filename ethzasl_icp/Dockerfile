FROM ros:kinetic

RUN apt-get update && apt-get install -y --no-install-recommends && apt-get install -y --no-install-recommends wget nano build-essential ros-kinetic-pcl-ros
# RUN apt-get install -y --no-install-recommends libomp-dev

RUN apt-get install -y --no-install-recommends unzip

WORKDIR /root


# libnano & libpointmatcher
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace'

RUN git clone https://github.com/ethz-asl/libnabo.git
RUN git clone https://github.com/ethz-asl/libpointmatcher.git

RUN apt-get install -y --no-install-recommends ros-kinetic-tf-conversions ros-kinetic-eigen-conversions

# build
WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make_isolated -DCMAKE_BUILD_TYPE=Release'

# ethzasl_icp_mapping
WORKDIR /root/catkin_ws2/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace'
RUN git clone https://github.com/ethz-asl/ethzasl_icp_mapping
# RUN git clone https://github.com/mgb45/ethzasl_icp_mapping.git

RUN apt-get install -y --no-install-recommends ros-kinetic-map-msgs ros-kinetic-interactive-markers

# build
WORKDIR /root/catkin_ws2
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; . /root/catkin_ws/devel_isolated/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release'
# RUN /bin/bash -c '. /root/catkin_ws/devel_isolated/setup.bash'

RUN sed -i "6i source \"/root/catkin_ws/devel_isolated/setup.bash\"" /ros_entrypoint.sh
RUN sed -i "7i source \"/root/catkin_ws2/devel/setup.bash\"" /ros_entrypoint.sh


# RUN apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
