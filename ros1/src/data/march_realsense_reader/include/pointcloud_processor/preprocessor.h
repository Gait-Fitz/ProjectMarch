#ifndef MARCH_PREPROCESSOR_H
#define MARCH_PREPROCESSOR_H

#include <string>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;

class Preprocessor {
  public:
    Preprocessor(YAML::Node config_tree,
                 std::shared_ptr<PointCloud> pointcloud,
                 std::shared_ptr<Normals> normal_pointcloud);
    Preprocessor(std::string file_name,
                 std::shared_ptr<PointCloud> pointcloud,
                 std::shared_ptr<Normals> normal_pointcloud);
    virtual void preprocess()=0; // This function is required to be implemented by
    // any preprocessor
    virtual ~Preprocessor() {};

  private:
    std::shared_ptr<PointCloud> pointcloud_;
    std::shared_ptr<Normals> normal_pointcloud_;
    YAML::Node config_tree_;
};

class SimplePreprocessor : Preprocessor {
public:
    void preprocess();
};

#endif //MARCH_PREPROCESSOR_H