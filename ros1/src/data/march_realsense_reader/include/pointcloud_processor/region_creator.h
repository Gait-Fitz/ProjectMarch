/*
 * Copyright (C) 2021 Katja Schmahl, Thijs Raymakers, Thijs Veen,
 *                    Wolf Nederpel
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * Version 3 as published by the Free Software Foundation WITH
 * additional terms published by Project MARCH per section 7 of
 * the GNU General Public License Version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License INCLUDING the additional terms for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * AND the additional terms along with this program. If not,
 * see <https://projectmarch.nl/s/LICENSE> and
 * <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>.
 */

#ifndef MARCH_REGION_CREATOR_H
#define MARCH_REGION_CREATOR_H

#include <march_realsense_reader/pointcloud_parametersConfig.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <string>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using Normals = pcl::PointCloud<pcl::Normal>;
using RegionVector = std::vector<pcl::PointIndices>;

class RegionCreator {
public:
    explicit RegionCreator(bool debugging);
    // This function is required to be implemented by any region creator
    virtual bool createRegions(PointCloud::Ptr pointcloud,
        Normals::Ptr pointcloud_normals,
        boost::shared_ptr<RegionVector> region_vector)
        = 0;
    virtual ~RegionCreator() = default;
    virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_visualisation() = 0;

    /** This function is called upon whenever a parameter from config is
     * changed, including when launching the node
     */
    virtual void readParameters(
        march_realsense_reader::pointcloud_parametersConfig& config)
        = 0;

protected:
    PointCloud::Ptr pointcloud_;
    Normals::Ptr pointcloud_normals_;
    boost::shared_ptr<RegionVector> region_vector_;
    bool debugging_;
};

class RegionGrower : RegionCreator {
public:
    // Use the constructors defined in the super class
    explicit RegionGrower(bool debugging);
    /** Create cluster using the region growing algorithm, takes algorithm
     * configuration from the dynamic parameter server, and fills parameter
     * region_vector with clusters. **/
    bool createRegions(PointCloud::Ptr pointcloud,
        Normals::Ptr pointcloud_normals,
        boost::shared_ptr<RegionVector> region_vector) override;

    /**
     * @return A pointer to a single pointcloud, with unique colours for every
     * cluster
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_visualisation() override;

    /** This function is called upon whenever a parameter from config is
     * changed, including when launching the node
     */
    void readParameters(
        march_realsense_reader::pointcloud_parametersConfig& config) override;

private:
    /**
     * Configure region growing algorithm
     */
    bool setupRegionGrower();

    /**
     * Extract clusters from region_grower object
     * @return true if succesful
     */
    bool extractRegions();

    // Region Growing Object
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> region_grower;

    // Region Growing configuration parameters
    int number_of_neighbours;
    int min_cluster_size;
    int max_cluster_size;
    float smoothness_threshold;
    float curvature_threshold;
};

#endif // MARCH_PREPROCESSOR_H
