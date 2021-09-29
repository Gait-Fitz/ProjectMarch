#include "utilities/realsense_category_utilities.h"
#include "yaml-cpp/yaml.h"
#include "pointcloud_processor/highest_point_finder.h"
#include <ros/package.h>
#include <ros/ros.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using PointXYZ = pcl::PointXYZ;

// Finds the highest point in a pointcloud
bool HighestPointFinder::findHighestPoint(PointCloud::Ptr pointcloud,
    Normals::Ptr pointcloud_normals,
    RealSenseCategory const realsense_category,
    std::string frame_id_to_transform_to)
{
    pointcloud_ = pointcloud;
    pointcloud_normals_ = pointcloud_normals;
    frame_id_to_transform_to_ = frame_id_to_transform_to;
    realsense_category_.emplace(realsense_category);

    PointXYZ temp;
    int max_z = INT_MIN;

    for (int point_index = 0; point_index < pointcloud_->points.size(); ++point_index) {
        PointXYZ point = pointcloud_->points[point_index];
        if (point.z > max_z) {
            max_z = point.z;
            temp = point;
        }
    }

    highest_point_ = temp;

    return true;
};