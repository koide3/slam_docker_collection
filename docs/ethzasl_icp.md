# ethzasl_icp_mapping

Original repository: https://github.com/ethz-asl/ethzasl_icp_mapping


## Build
```bash
cd ethzasl_icp_mapping
./docker/build.sh
```

## Run

On the host:
```bash
roscore
```

```bash
rosparam set use_sim_time true

cd ethzasl_icp_mapping/docker
rviz -d ethzasl_icp_mapping.rviz
```

```bash
rosbag play --clock hdl_400.bag
```
http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz

On the docker image:
```bash
cd ethzasl_icp_mapping
./docker/run.sh

roslaunch ethzasl_icp_mapper grizzly_dynamic_mapper_outdoor.launch
```
