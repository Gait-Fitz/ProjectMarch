#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <region_creator.h>
#include <float.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h> 
#include <pcl/filters/extract_indices.h> 
#include <utilities/linear_algebra_utilities.h>
#include <cmath>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using Normal = pcl::Normal;
using NormalCloud = pcl::PointCloud<Normal>;
using PointIndices = pcl::PointIndices;
using IndicesVector = std::vector<PointIndices::Ptr>;

RegionCreator::RegionCreator(PointCloud::Ptr pointcloud, NormalCloud::Ptr normalcloud)
    : pointcloud_ { pointcloud }
    , normalcloud_ { normalcloud }
    {}

SeedRegionGrower::SeedRegionGrower(PointCloud::Ptr pointcloud, NormalCloud::Ptr normalcloud)
    : RegionCreator(pointcloud, normalcloud)
    , indices_(pointcloud->points.size())
    , dotproducts_(pointcloud->points.size())
    , queue_ {}
    , checked_ {}
{
    for (std::size_t i = 0; i < pointcloud->points.size(); i++)
    {
        indices_[i] = i;
    }
    computeDotProducts();
}

bool SeedRegionGrower::createRegions(boost::shared_ptr<IndicesVector> regions)
{
    growRegionsWithSeeds(20, 5, regions);
    return true;
}

bool SeedRegionGrower::growRegionsWithSeeds(unsigned int minimum_points_in_region,
                                            unsigned int num_neirest_neighbours,   
                                            boost::shared_ptr<IndicesVector> regions)
{
    int current_index;
    PointIndices::Ptr region = boost::make_shared<PointIndices>();
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pointcloud_);

    while (checked_.size() < indices_.size())
    {
        if (queue_.size() > 0)
        {
            current_index = *queue_.begin();
            queue_.erase(current_index);
        }
        else 
        {
            if (region->indices.size() > minimum_points_in_region) {
                regions->push_back(region);
            }
            
            PointIndices::Ptr new_region = boost::make_shared<PointIndices>();
            region = new_region;
            
            current_index = std::distance(dotproducts_.begin(), std::min_element(dotproducts_.begin(), dotproducts_.end()));
            dotproducts_[current_index] = DBL_MAX;
            region->indices.push_back(current_index);
        }

        if (checked_.find(current_index) == checked_.end()) {
            checked_.insert(current_index);
        }

        const Normal normal = normalcloud_->points[current_index];
        const Point point = pointcloud_->points[current_index];

        std::vector<int> idx(num_neirest_neighbours);
        std::vector<float> distances(num_neirest_neighbours);

        if (kdtree.nearestKSearch(point, num_neirest_neighbours, idx, distances) > 0)
        {
            for (auto &n_idx : idx)
            {
                const double dot = std::abs(linalg::dotProductNormal(normal, (*normalcloud_)[n_idx]));

                if (dot >= neighbour_angle_diff_threshold)
                {
                    if (checked_.find(n_idx) == checked_.end() && queue_.find(n_idx) == queue_.end())
                    {
                        region->indices.push_back(current_index);
                        if (max_seed_angle <= dotproducts_[n_idx] && dotproducts_[n_idx] <= 1)
                            queue_.insert(n_idx);
                    }
                }
            }
        }
    }
    return true;
}

bool SeedRegionGrower::computeDotProducts()
{
    for (std::size_t i = 0; i < normalcloud_->points.size(); i++)
    {
        dotproducts_[i] = std::abs(linalg::dotProductNormal(z_unit, normalcloud_->points[i]));
    }
    return true;
}
