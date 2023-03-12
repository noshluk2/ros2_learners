#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

int
main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader cloud_reader;
  pcl::PCDWriter cloud_writer;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  std::string path="/home/luqman/ros2_ws/src/ros2_learners/point_cloud_perception/point_clouds/";
  // Reading the cloud
  cloud_reader.read (path+std::string("cloud.pcd"),*cloud);


//   Planner_segmentation

    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.01);
    plane_seg.setInputCloud(cloud);
    plane_seg.segment(*inliers,*coefficients);


// Extracting Points
    pcl::ExtractIndices<pcl::PointXYZ> extract_indicies;
    extract_indicies.setInputCloud(cloud);
    extract_indicies.setIndices(inliers);
    extract_indicies.setNegative(true);
    extract_indicies.filter(*plane_seg_cloud);


  // Write
  cloud_writer.write<pcl::PointXYZ>(path+std::string("non_plane_seg.pcd"),*plane_seg_cloud, false);


  return (0);
}