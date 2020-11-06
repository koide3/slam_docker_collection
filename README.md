# slam_docker_collection

This is my personal attempt to create reusable and portable environments for 3D SLAM with docker.

## Basic Usage

Update submodule:
```bash
cd slam_docker_collection
git submodule init
git submodule update hdl_graph_slam
```

Build docker image:
```bash
cd slam_docker_collection/hdl_graph_slam/docker
./build.sh
```

Run docker image:
```bash
cd slam_docker_collection/hdl_graph_slam/docker
./run.sh -v ~/datasets:/datasets  # you can put more docker run arguments here

# in docker
roslaunch hdl_graph_slam hdl_graph_slam_400.launch
```

## Dockernized packages
- [![Build Status](https://travis-ci.org/koide3/ethzasl_icp_mapping.svg?branch=reintegrate%2Fmaster_into_indigo_devel)](https://travis-ci.org/koide3/ethzasl_icp_mapping) [ethzasl_icp_mapping](https://github.com/ethz-asl/ethzasl_icp_mapping) [[usage]](https://github.com/koide3/ethzasl_icp_mapping/blob/reintegrate/master_into_indigo_devel/docker/howtouse.md)
- [![Build Status](https://travis-ci.org/koide3/hdl_graph_slam.svg?branch=master)](https://travis-ci.org/koide3/hdl_graph_slam) [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) [[usage]](https://github.com/koide3/hdl_graph_slam/blob/master/docker/howtouse.md)
- [![Build Status](https://travis-ci.org/koide3/loam_velodyne.svg?branch=master)](https://travis-ci.org/koide3/loam_velodyne) [LOAM](https://github.com/laboshinl/loam_velodyne) [[usage]](https://github.com/koide3/loam_velodyne/blob/master/docker/howtouse.md)
- [![Build Status](https://travis-ci.org/koide3/LeGO-LOAM-BOR.svg?branch=master)](https://travis-ci.org/koide3/LeGO-LOAM-BOR) [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) [[usage]](https://github.com/koide3/LeGO-LOAM-BOR/blob/master/docker/howtouse.md)
- [![Build Status](https://travis-ci.org/koide3/SuMa.svg?branch=master)](https://travis-ci.org/koide3/SuMa) [SuMa](https://github.com/jbehley/SuMa) [[usage]](https://github.com/koide3/SuMa/blob/master/docker/howtouse.md)
- [Voxblox](https://github.com/ethz-asl/voxblox)
- [Voxgraph](https://github.com/ethz-asl/voxgraph)
- [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM)
- [LIO-mapping](https://github.com/hyye/lio-mapping)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
