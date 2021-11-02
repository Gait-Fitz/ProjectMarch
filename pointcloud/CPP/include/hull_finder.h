#ifndef MARCH_HULL_FINDER_H
#define MARCH_HULL_FINDER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <unordered_set>
#include <pcl/filters/extract_indices.h> 
#include <cmath>
#include <pcl/ModelCoefficients.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;
using PointsVector = std::vector<PointCloud::Ptr>;
using NormalsVector = std::vector<NormalCloud::Ptr>;
using PointIndices = pcl::PointIndices;
using IndicesVector = std::vector<PointIndices::Ptr>;
using CoefficientsVector = std::vector<pcl::ModelCoefficients::Ptr>;
using MatrixVector = std::vector<Eigen::Matrix3d>;
using Hull = pcl::PointCloud<pcl::PointXYZ>;
using HullVector = std::vector<Hull::Ptr>;


class HullFinder {
public:

    explicit HullFinder(PointCloud::Ptr pointcloud,
                        NormalCloud::Ptr normalcloud,
                        boost::shared_ptr<IndicesVector> regions);

    ~HullFinder() = default;

    virtual bool findHulls(boost::shared_ptr<HullVector> &hulls,
                           boost::shared_ptr<CoefficientsVector> &plane_coefficients) = 0;

protected:
    PointCloud::Ptr pointcloud_;
    NormalCloud::Ptr normalcloud_;
    boost::shared_ptr<IndicesVector> regions_;

};

class PlaneHullFinder : HullFinder {
public:

    explicit PlaneHullFinder(PointCloud::Ptr pointcloud,
                             NormalCloud::Ptr normalcloud,
                             boost::shared_ptr<IndicesVector> regions);

    bool findHulls(boost::shared_ptr<HullVector> &hulls,
                   boost::shared_ptr<CoefficientsVector> &plane_coefficients) override;

protected:

    bool createRegionPointClouds();

    bool determinePlaneCoefficients(PointCloud::Ptr region_point_cloud);

    bool projectPointsToPlanes(PointCloud::Ptr region_point_cloud, pcl::ModelCoefficients::Ptr coefficients);

    bool create2DHullFromRegion(PointCloud::Ptr region_point_cloud);

    boost::shared_ptr<PointsVector> region_points_vector_;
    boost::shared_ptr<NormalsVector> region_normals_vector_;
    boost::shared_ptr<CoefficientsVector> plane_coefficients_;
    boost::shared_ptr<HullVector> hulls_;

};


#endif // MARCH_SURFACE_FINDER_H