FROM ros:melodic

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
 && apt-get install -y --no-install-recommends wget nano unzip build-essential libomp-dev python3-pip \
                                               libmetis-dev ros-melodic-pcl-ros ros-melodic-image-transport \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /root
RUN wget -O gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
RUN unzip gtsam.zip
RUN mkdir /root/gtsam-4.0.0-alpha2/build
WORKDIR /root/gtsam-4.0.0-alpha2/build
RUN cmake .. && make -j16 2> /dev/null
RUN make install

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace'

RUN sed -i '9c #include "lz4.h"' /usr/include/flann/util/serialization.h
RUN sed -i '10c #include "lz4hc.h"' /usr/include/flann/util/serialization.h

COPY LeGO-LOAM-BOR /root/catkin_ws/src/LeGO-LOAM-BOR/
# RUN git clone https://github.com/facontidavide/LeGO-LOAM-BOR

RUN pip3 install pyquaternion

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release'

RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh

WORKDIR /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
