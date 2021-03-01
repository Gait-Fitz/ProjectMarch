#include "pointcloud_processor/region_creator.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionsVector = std::vector<pcl::PointIndices>;

// Construct a basic RegionCreator class
RegionCreator::RegionCreator(YAML::Node config_tree):
    config_tree_{config_tree}
{

}

/** This function should take in a pointcloud with matching normals and cluster them
 in regions, based on the parameters in the YAML node given at construction **/
void SimpleRegionCreator::create_regions(PointCloud::Ptr pointcloud,
                                         Normals::Ptr normal_pointcloud,
                                         boost::shared_ptr<RegionsVector>
                                             regions_vector)
{
  pointcloud_ = pointcloud;
  normal_pointcloud_ = normal_pointcloud;
  regions_vector_ = regions_vector;
  ROS_INFO_STREAM("Creating regions with SimpleRegionCreator");
}

