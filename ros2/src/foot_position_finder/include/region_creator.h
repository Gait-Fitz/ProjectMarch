#ifndef MARCH_REGION_CREATOR_H
#define MARCH_REGION_CREATOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <unordered_set>
#include <pcl/filters/extract_indices.h> 
#include <cmath>

using Normal = pcl::Normal;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;
using PointIndices = pcl::PointIndices;
using IndicesVector = std::vector<PointIndices::Ptr>;

class RegionCreator {
public:

    explicit RegionCreator(PointCloud::Ptr pointcloud, NormalCloud::Ptr normalcloud);

    ~RegionCreator() = default;

    virtual bool createRegions(boost::shared_ptr<IndicesVector> regions) = 0;

protected:
    PointCloud::Ptr pointcloud_;
    NormalCloud::Ptr normalcloud_;
};

class SeedRegionGrower : RegionCreator {
public:

    explicit SeedRegionGrower(PointCloud::Ptr pointcloud, NormalCloud::Ptr normalcloud);

    bool createRegions(boost::shared_ptr<IndicesVector> regions) override;

protected:

    bool computeDotProducts();

    bool growRegionsWithSeeds(unsigned int minimum_points_in_region,
                              unsigned int num_neirest_neighbours,
                              boost::shared_ptr<IndicesVector> regions);

    std::unordered_set<int> queue_;
    std::unordered_set<int> checked_;
    std::vector<int> indices_;
    std::vector<double> dotproducts_;

    const double neighbour_angle_diff_threshold = cos(20 * M_PI / 180);
    const double max_seed_angle = cos(35 * M_PI / 180);

    const Normal z_unit = Normal(0, -0.5 * sqrt(2), -0.5 * sqrt(2));

    int minimum_points_in_region;
    int num_neirest_neighbours;
    
};


#endif // MARCH_REGION_CREATOR_H