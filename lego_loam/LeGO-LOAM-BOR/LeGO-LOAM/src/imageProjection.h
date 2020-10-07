#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include <Eigen/QR>

class ImageProjection {
 public:

  ImageProjection(ros::NodeHandle& nh,
                  Channel<ProjectionOut>& output_channel);

  ~ImageProjection() = default;

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

 private:
  void findStartEndAngle();
  void resetParameters();
  void projectPointCloud();
  void groundRemoval();
  void cloudSegmentation();
  void labelComponents(int row, int col);
  void publishClouds();

  pcl::PointCloud<PointType>::Ptr _laser_cloud_in;

  pcl::PointCloud<PointType>::Ptr _full_cloud;
  pcl::PointCloud<PointType>::Ptr _full_info_cloud;

  pcl::PointCloud<PointType>::Ptr _ground_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure;
  pcl::PointCloud<PointType>::Ptr _outlier_cloud;

  ros::NodeHandle& _nh;
  int _vertical_scans;
  int _horizontal_scans;
  float _ang_bottom;
  float _ang_resolution_X;
  float _ang_resolution_Y;
  float _segment_alpha_X;
  float _segment_alpha_Y;
  float _segment_theta;
  int _segment_valid_point_num;
  int _segment_valid_line_num;
  int _ground_scan_index;
  float _sensor_mount_angle;

  Channel<ProjectionOut>& _output_channel;

  ros::Subscriber _sub_laser_cloud;

  ros::Publisher _pub_full_cloud;
  ros::Publisher _pub_full_info_cloud;

  ros::Publisher _pub_ground_cloud;
  ros::Publisher _pub_segmented_cloud;
  ros::Publisher _pub_segmented_cloud_pure;
  ros::Publisher _pub_segmented_cloud_info;
  ros::Publisher _pub_outlier_cloud;

  cloud_msgs::cloud_info _seg_msg;

  int _label_count;

  Eigen::MatrixXf _range_mat;   // range matrix for range image
  Eigen::MatrixXi _label_mat;   // label matrix for segmentaiton marking
  Eigen::Matrix<int8_t,Eigen::Dynamic,Eigen::Dynamic> _ground_mat;  // ground matrix for ground cloud marking


};



#endif  // IMAGEPROJECTION_H
