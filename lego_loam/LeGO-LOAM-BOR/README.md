This is a customized LeGO-LOAM to save the mapping result in the dump format that is readable from [interactive_slam](https://github.com/SMRT-AIST/interactive_slam). After running ```run.launch```, you can find the dumped mapping result at ```/tmp/dump```. For the other functionalities, see [the original LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) and [facontidavide's fork](https://github.com/facontidavide/LeGO-LOAM-BOR).


# LeGO-LOAM-BOR

This is a fork of the original [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM).

This is a "friendly fork", in other words, we will be happy to work with the original authors to merge
these improvements with the original LeGO-LOAM... if they want to!

The original author deserves all the credits, we just use good software engineering practices to
make the code more readable and efficient.

The purpose of this fork is:

- To improve the quality of the code, making it more readable, consistent and easier to understand and modify.
- To remove hard-coded values and use proper configuration files to describe the hardware.
- To improve performance, in terms of amount of CPU used to calculate the same result.
- To convert a multi-process application into a single-process / multi-threading one; this makes the algorithm
  more deterministic and slightly faster.
- To make it easier and faster to work with rosbags: processing a rosbag should be done at maximum
  speed allowed by the CPU and in a deterministic way (usual speed improvement in the order of 5X-10X).
- As a consequence of the previous point, creating unit and regression tests will be easier.

The purpose of this fork (for the time being) is **not** to modify and/or improve the original algorithm.

Please do not submit to this repository any issue related to the algorithm, since we are focusing on the
software implementation.

# About the original LeGO-LOAM
This repository contains code for a lightweight and ground optimized lidar odometry and mapping (LeGO-LOAM) system for ROS compatible UGVs.
The system takes in point cloud  from a Velodyne VLP-16 Lidar (palced horizontal) and optional IMU data as inputs.
It outputs 6D pose estimation in real-time. A demonstration of the system can be found here -> https://www.youtube.com/watch?v=O3tz_ftHV48
<!--
[![Watch the video](/LeGO-LOAM/launch/demo.gif)](https://www.youtube.com/watch?v=O3tz_ftHV48)
-->
<p align='center'>
    <img src="/LeGO-LOAM/launch/demo.gif" alt="drawing" width="800"/>
</p>

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with indigo and kinetic)
- [gtsam](https://github.com/borglab/gtsam/releases) (Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)
  ```
  wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
  cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
  cd ~/Downloads/gtsam-4.0.0-alpha2/
  mkdir build && cd build
  cmake ..
  sudo make install
  ```

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/facontidavide/LeGO-LOAM-BOR.git
cd ..
catkin_make
```

## The system

LeGO-LOAM is speficifally optimized for a horizontally placed lidar on a ground vehicle. It assumes there is always a ground plane in the scan. The UGV we are using is Clearpath Jackal.

<p align='center'>
    <img src="/LeGO-LOAM/launch/jackal-label.jpg" alt="drawing" width="400"/>
</p>

The package performs segmentation before feature extraction.

<p align='center'>
    <img src="/LeGO-LOAM/launch/seg-total.jpg" alt="drawing" width="400"/>
</p>

Lidar odometry performs two-step Levenberg Marquardt optimization to get 6D transformation.

<p align='center'>
    <img src="/LeGO-LOAM/launch/odometry.jpg" alt="drawing" width="400"/>
</p>

## New sensor and configuration

To customize the behavior of the algorithm or to use a lidar different from VLP-16, edit the file **config/loam_config.yaml**.

One important thing to keep in mind is that our current implementation for range image projection is only suitable for sensors that have evenly distributed channels.
If you want to use our algorithm with Velodyne VLP-32c or HDL-64e, you need to write your own implementation for such projection.

If the point cloud is not projected properly, you will lose many points and performance.

**The IMU has been remove from the original code.** Deal with it.

## Run the package

You may process a rosbag using the following command:

```
roslaunch lego_loam_bor run.launch rosbag:=/path/to/your/rosbag lidar_topic:=/velodyne_points
```

Change the parameters `rosbag`, `lidar_topic` as needed.


Some sample bags can be downloaded from [here](https://github.com/RobustFieldAutonomyLab/jackal_dataset_20170608).

## New data-set

This dataset, [Stevens data-set](https://github.com/TixiaoShan/Stevens-VLP16-Dataset), is captured using a Velodyne VLP-16, which is mounted on an UGV - Clearpath Jackal, on Stevens Institute of Technology campus.
The VLP-16 rotation rate is set to 10Hz. This data-set features over 20K scans and many loop-closures.

<p align='center'>
    <img src="/LeGO-LOAM/launch/dataset-demo.gif" alt="drawing" width="600"/>
</p>
<p align='center'>
    <img src="/LeGO-LOAM/launch/google-earth.png" alt="drawing" width="600"/>  
</p>

## Cite *LeGO-LOAM*

Thank you for citing our *LeGO-LOAM* paper if you use any of this code: 
```
@inproceedings{legoloam2018,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Tixiao Shan and Brendan Englot},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```
