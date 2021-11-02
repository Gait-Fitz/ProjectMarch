#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <hull_finder.h>
#include <float.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h> 
#include <pcl/filters/extract_indices.h> 
#include <utilities/linear_algebra_utilities.h>
#include <cmath>
#include <chrono>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <utilities/linear_algebra_utilities.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>


using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using PointIndices = pcl::PointIndices;
using Normal = pcl::Normal;
using NormalCloud = pcl::PointCloud<Normal>;
using CoefficientsVector = std::vector<pcl::ModelCoefficients::Ptr>;
using Polygon = std::vector<pcl::Vertices>;
using Hull = pcl::PointCloud<Point>;

HullFinder::HullFinder(PointCloud::Ptr pointcloud,
                       NormalCloud::Ptr normalcloud,
                       boost::shared_ptr<IndicesVector> regions)
    : pointcloud_ { pointcloud }
    , normalcloud_ { normalcloud }
    , regions_ { regions }
    {}

PlaneHullFinder::PlaneHullFinder(PointCloud::Ptr pointcloud,
                                 NormalCloud::Ptr normalcloud,
                                 boost::shared_ptr<IndicesVector> regions)
    : HullFinder(pointcloud,  normalcloud, regions)
    {
        region_points_vector_ = boost::make_shared<PointsVector>();
        region_normals_vector_ = boost::make_shared<NormalsVector>();
        plane_coefficients_ = boost::make_shared<CoefficientsVector>();
        hulls_ = boost::make_shared<HullVector>();
        createRegionPointClouds();
    }

bool PlaneHullFinder::createRegionPointClouds()
{
    for (PointIndices::Ptr &region : *regions_)
    { 

        PointCloud::Ptr region_pointcloud = boost::make_shared<PointCloud>();
        NormalCloud::Ptr region_normals = boost::make_shared<NormalCloud>();

        pcl::copyPointCloud(*pointcloud_, *region, *region_pointcloud);
        pcl::copyPointCloud(*normalcloud_, *region, *region_normals);

        region_points_vector_->push_back(region_pointcloud);
        region_normals_vector_->push_back(region_normals);
    }
    return true;
}

bool PlaneHullFinder::findHulls(boost::shared_ptr<HullVector> &hulls,
                                boost::shared_ptr<CoefficientsVector> &plane_coefficients)
{
    for (auto i = 0; i < region_points_vector_->size() ; i++)
    {
        auto region_point_cloud = (*region_points_vector_)[i];

        determinePlaneCoefficients(region_point_cloud);
        projectPointsToPlanes(region_point_cloud, (*plane_coefficients_)[i]);
        create2DHullFromRegion(region_point_cloud);
    }
    hulls = hulls_;
    plane_coefficients = plane_coefficients_;
    return true;
}

bool PlaneHullFinder::determinePlaneCoefficients(PointCloud::Ptr region_point_cloud)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new PointIndices);
    pcl::SACSegmentation<Point> seg;
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setNumberOfThreads(2);
    seg.setInputCloud(region_point_cloud);
    seg.segment(*inliers, *coefficients);

    plane_coefficients_->push_back(coefficients);

    return true;
}

bool PlaneHullFinder::projectPointsToPlanes(PointCloud::Ptr region_point_cloud,                     
                                            pcl::ModelCoefficients::Ptr coefficients)
{
    const auto a = coefficients->values[0];
    const auto b = coefficients->values[1];
    const auto c = coefficients->values[2];
    const auto d = coefficients->values[3];

    for (auto &point : *region_point_cloud)
    {
        point.z = -(a * point.x + b * point.y + d) / c;
    }
    
    return true;
}

bool PlaneHullFinder::create2DHullFromRegion(PointCloud::Ptr region_point_cloud)
{
    Polygon polygon;
    polygon.clear();
    Hull::Ptr hull_ = boost::make_shared<Hull>();
    pcl::ConvexHull<Point> convex_hull;
    convex_hull.setInputCloud(region_point_cloud);
    convex_hull.setDimension(2);
    convex_hull.reconstruct(*hull_, polygon);
    hulls_->push_back(hull_);
    return true;
}