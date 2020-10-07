#ifndef FEATUREASSOCIATION_H
#define FEATUREASSOCIATION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include "lego_loam/nanoflann_pcl.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

class FeatureAssociation {

 public:
  FeatureAssociation( ros::NodeHandle& node,
                     Channel<ProjectionOut>& input_channel,
                     Channel<AssociationOut>& output_channel);

  ~FeatureAssociation();

  void runFeatureAssociation();

 private:

  ros::NodeHandle& nh;

  int _vertical_scans;
  int _horizontal_scans;
  float _scan_period;
  float _edge_threshold;
  float _surf_threshold;
  float _nearest_feature_dist_sqr;
  int _mapping_frequency_div;

  std::thread _run_thread;

  Channel<ProjectionOut>& _input_channel;
  Channel<AssociationOut>& _output_channel;

  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointType> downSizeFilter;

  double timeScanCur;

  cloud_msgs::cloud_info segInfo;
  std_msgs::Header cloudHeader;

  int systemInitCount;
  bool systemInited;

  std::vector<smoothness_t> cloudSmoothness;
  std::vector<float> cloudCurvature;
  std::vector<int> cloudNeighborPicked;
  std::vector<int> cloudLabel;

  ros::Publisher _pub_cloud_corner_last;
  ros::Publisher _pub_cloud_surf_last;
  ros::Publisher pubLaserOdometry;
  ros::Publisher _pub_outlier_cloudLast;

  int skipFrameNum;
  bool systemInitedLM;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  std::vector<int> pointSelCornerInd;
  std::vector<float> pointSearchCornerInd1;
  std::vector<float> pointSearchCornerInd2;

  std::vector<int> pointSelSurfInd;
  std::vector<float> pointSearchSurfInd1;
  std::vector<float> pointSearchSurfInd2;
  std::vector<float> pointSearchSurfInd3;

  float transformCur[6];
  float transformSum[6];

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  nanoflann::KdTreeFLANN<PointType> kdtreeCornerLast;
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  nav_msgs::Odometry laserOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;

  bool isDegenerate;

  int frameCount;
  size_t _cycle_count;

 private:
  void initializationValue();
  void adjustDistortion();
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();

  void TransformToStart(PointType const *const pi, PointType *const po);
  void TransformToEnd(PointType const *const pi, PointType *const po);

  void AccumulateRotation(float cx, float cy, float cz, float lx, float ly,
                          float lz, float &ox, float &oy, float &oz);

  void findCorrespondingCornerFeatures(int iterCount);
  void findCorrespondingSurfFeatures(int iterCount);

  bool calculateTransformationSurf(int iterCount);
  bool calculateTransformationCorner(int iterCount);
  bool calculateTransformation(int iterCount);

  void checkSystemInitialization();
  void updateTransformation();

  void integrateTransformation();
  void publishCloud();
  void publishOdometry();

  void adjustOutlierCloud();
  void publishCloudsLast();

};

#endif // FEATUREASSOCIATION_H
