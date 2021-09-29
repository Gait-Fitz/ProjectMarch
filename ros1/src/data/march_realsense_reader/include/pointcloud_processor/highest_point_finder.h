#ifndef MARCH_HIGHESTPOINTFINDER_H
#define MARCH_HIGHESTPOINTFINDER_H

#include "utilities/realsense_category_utilities.h"
#include <march_realsense_reader/pointcloud_parametersConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using PointXYZ = pcl::PointXYZ;

class HighestPointFinder {
public:

    bool findHighestPoint(PointCloud::Ptr pointcloud,
        Normals::Ptr pointcloud_normals,
        RealSenseCategory const realsense_category,
        std::string frame_id_to_transform_to);

    virtual ~HighestPointFinder() = default;

    PointCloud::Ptr pointcloud_;
    PointXYZ highest_point_;
    Normals::Ptr pointcloud_normals_;
    std::optional<RealSenseCategory> realsense_category_ = std::nullopt;

protected:
    std::string frame_id_to_transform_to_;

};

#endif // MARCH_HIGHESTPOINTFINDER_H