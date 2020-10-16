FROM ros:melodic

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
 && apt-get install -y --no-install-recommends wget nano build-essential libgoogle-glog-dev automake libtool m4 automake \
                                               python-wstool python-catkin-tools protobuf-compiler autoconf \
                                               libmetis-dev mesa-utils x11-apps \
                                               ros-melodic-cmake-modules ros-melodic-eigen-conversions ros-melodic-tf-conversions ros-melodic-rviz ros-melodic-cv-bridge \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /root

RUN mkdir -p /root/catkin_ws/src
# RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace'

WORKDIR /root/catkin_ws
RUN catkin init
RUN catkin config --extend /opt/ros/melodic
RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN catkin config --merge-devel

WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/ethz-asl/voxblox.git
# COPY voxblox /root/catkin_ws/src/voxblox
RUN wstool init . ./voxblox/voxblox_https.rosinstall
RUN wstool update

WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin build voxblox_ros'

RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
