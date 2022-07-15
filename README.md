# slam_docker_collection

This is my personal attempt to create reusable and portable environments for 3D SLAM with docker. I think SLAM packages are always difficult to build from scratch in particular when we use different versions of ROS/Ubuntu/libraries. 

## Basic Usage

### 1. Update submodule:
```bash
cd slam_docker_collection
git submodule init
git submodule update hdl_graph_slam
```

### 2. Build docker image:
```bash
cd slam_docker_collection/hdl_graph_slam/docker
./build.sh
```

### 3. Run docker image:
```bash
cd slam_docker_collection/hdl_graph_slam/docker
./run.sh -v ~/datasets:/datasets  # you can put more docker run arguments here

# in docker
roslaunch hdl_graph_slam hdl_graph_slam_400.launch
```

## Dockernized packages
| Build status | Package | Short Usage |
| ------------ | ------- | ----------- |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/ethzasl_icp.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/ethzasl_icp.yml) | [ethzasl_icp_mapping](https://github.com/ethz-asl/ethzasl_icp_mapping) | [[usage]](https://github.com/koide3/ethzasl_icp_mapping/blob/reintegrate/master_into_indigo_devel/docker/howtouse.md) | ![ethzasl_icp](https://user-images.githubusercontent.com/31344317/98346757-c4bf9480-2059-11eb-93b0-d97dc637fe16.gif) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/hdl_graph_slam.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/hdl_graph_slam.yml) | [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) | [[usage]](https://github.com/koide3/hdl_graph_slam/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/loam.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/loam.yml) | [LOAM](https://github.com/laboshinl/loam_velodyne) | [[usage]](https://github.com/koide3/loam_velodyne/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/lego_loam.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/lego_loam.yml) | [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) | [[usage]](https://github.com/koide3/LeGO-LOAM-BOR/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/suma.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/suma.yml) | [SuMa](https://github.com/jbehley/SuMa) | [[usage]](https://github.com/koide3/SuMa/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/lins.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/lins.yml) | [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM) | [[usage]](https://github.com/koide3/LINS---LiDAR-inertial-SLAM/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/lio_mapping.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/lio_mapping.yml) | [LIO-mapping](https://github.com/hyye/lio-mapping) | [[usage]](https://github.com/koide3/lio-mapping/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/lio_sam.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/lio_sam.yml) | [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) | [[usage]](https://github.com/koide3/LIO-SAM/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/fast_lio.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/fast_lio.yml) | [FAST-LIO](https://github.com/hku-mars/FAST_LIO) | [[usage]](https://github.com/koide3/FAST_LIO/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/direct_lidar_odometry.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/direct_lidar_odometry.yml) | [direct_lidar_odometry](https://github.com/vectr-ucla/direct_lidar_odometry) | [[usage]](https://github.com/koide3/direct_lidar_odometry/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/dliom.yaml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/dliom.yaml) | [D-LIOM](https://github.com/peterWon/D-LIOM) | [[usage]](https://github.com/koide3/D-LIOM/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/clins.yaml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/clins.yaml) | [clins](https://github.com/APRIL-ZJU/clins) | [[usage]](https://github.com/koide3/clins/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/voxblox.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/voxblox.yml) | [Voxblox](https://github.com/ethz-asl/voxblox) | [[usage]](https://github.com/koide3/voxblox/blob/master/docker/howtouse.md) |
| [![Build](https://github.com/koide3/slam_docker_collection/actions/workflows/voxgraph.yml/badge.svg)](https://github.com/koide3/slam_docker_collection/actions/workflows/voxgraph.yml) | [Voxgraph](https://github.com/ethz-asl/voxgraph) | [[usage]](https://github.com/koide3/voxgraph/blob/master/docker/howtouse.md) |


| LOAM | hdl_graph_slam | LIO-SAM |
| ---- | -------------- | ---- |
| <img style="max-height: 320pix; width: auto;" src="https://user-images.githubusercontent.com/31344317/98347880-5da2df80-205b-11eb-8aae-abfd8fc67f70.gif"/> | <img style="max-height: 320pix; width: auto;" src="https://user-images.githubusercontent.com/31344317/98347836-4fed5a00-205b-11eb-931c-158f6cd056bf.gif"/> | <img style="max-height: 320pix; width: auto;" src="https://user-images.githubusercontent.com/31344317/98347870-5bd91c00-205b-11eb-82f0-8dec94dc3aec.gif"/> |

| LINS | SuMa | voxgraph |
| ---- | ---- | -------- |
| <img style="max-height: 320pix; width: auto;" src="https://user-images.githubusercontent.com/31344317/98347847-54197780-205b-11eb-988b-ac497d3ec8f8.gif"/> | <img  style="max-height: 320pix; width: auto;" src="https://user-images.githubusercontent.com/31344317/98347890-60053980-205b-11eb-97fa-de73c2f9448f.gif"/> | <img style="max-height: 320pix; width: auto;" src="https://user-images.githubusercontent.com/31344317/98347899-64315700-205b-11eb-92d5-1f2df959af6f.gif"/> |
