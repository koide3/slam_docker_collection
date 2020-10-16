FROM ros:melodic

RUN apt-get update && apt-get install -y --no-install-recommends && apt-get install -y --no-install-recommends wget nano build-essential ros-melodic-pcl-ros
RUN apt-get install -y --no-install-recommends libomp-dev
RUN apt-get install -y --no-install-recommends unzip

RUN apt-get install -y --no-install-recommends libgtest-dev libeigen3-dev libboost-all-dev qtbase5-dev libglew-dev

RUN apt-get install -y --no-install-recommends python-pip
RUN pip install -U pip setuptools
RUN pip install catkin_tools catkin_tools_fetch empy

WORKDIR /root
RUN wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
RUN unzip gtsam.zip
RUN mkdir /root/gtsam-4.0.0-alpha2/build
WORKDIR /root/gtsam-4.0.0-alpha2/build
RUN cmake .. && make -j16 2> /dev/null
RUN make install

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/ros/catkin.git

COPY SuMa /root/catkin_ws/src/SuMa/

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin init'
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin deps fetch'
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin build --save-config -i --cmake-args -DCMAKE_BUILD_TYPE=Release -DOPENGL_VERSION=430 -DENABLE_NVIDIA_EXT=YES'

RUN apt-get install -y --no-install-recommends libmetis-dev mesa-utils x11-apps

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
