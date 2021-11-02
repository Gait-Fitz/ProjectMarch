#ifndef MARCH_POSITION_FINDER_H
#define MARCH_POSITION_FINDER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <unordered_set>
#include <pcl/filters/extract_indices.h> 
#include <cmath>
#include <pcl/ModelCoefficients.h>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;
using PointIndices = pcl::PointIndices;
using IndicesVector = std::vector<PointIndices::Ptr>;
using MatrixVector = std::vector<Eigen::Matrix3d>;
using Hull = pcl::PointCloud<pcl::PointXYZ>;
using HullVector = std::vector<Hull::Ptr>;
using CoefficientsVector = std::vector<pcl::ModelCoefficients::Ptr>;
using ModelCoefficients = pcl::ModelCoefficients;

class PositionFinder {
public:

    explicit PositionFinder(boost::shared_ptr<HullVector> hulls,
                            boost::shared_ptr<CoefficientsVector> plane_coefficients);

    ~PositionFinder() = default;

    virtual bool findPositions(PointCloud::Ptr possible_foot_locations) = 0;

protected:
    boost::shared_ptr<HullVector> hulls_;
    boost::shared_ptr<CoefficientsVector> plane_coefficients_;
};

class RaysPlaneIntersector : PositionFinder {
public:

    explicit RaysPlaneIntersector(boost::shared_ptr<HullVector> hulls,
                                  boost::shared_ptr<CoefficientsVector> plane_coefficients);

    bool findPositions(PointCloud::Ptr possible_foot_locations) override;

protected:
    bool intersectRaysWithHull(Hull::Ptr hull, ModelCoefficients::Ptr, PointCloud::Ptr possible_foot_locations);

    bool isPointInHull(Hull::Ptr hull, Eigen::Vector3d point);

    bool isLeftOfEdge(const Point a, const Point b, const Eigen::Vector3d c);

    bool transformPointsBack();

};


#endif // MARCH_POSITION_FINDER_H