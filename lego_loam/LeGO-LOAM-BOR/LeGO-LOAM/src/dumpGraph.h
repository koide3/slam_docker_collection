#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>

void dump(
    const std::string& dump_directory, const gtsam::ISAM2& isam,
    const gtsam::Values& isam_current_estimate,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& corner_cloud_keyframes,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& surf_cloud_keyframes,
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& outlier_cloud_keyframes
);