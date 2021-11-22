#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <position_finder.h>
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

PositionFinder::PositionFinder(boost::shared_ptr<HullVector> hulls,
                               boost::shared_ptr<CoefficientsVector> plane_coefficients)
    {
        hulls_ = hulls;
        plane_coefficients_ = plane_coefficients;
    }

RaysPlaneIntersector::RaysPlaneIntersector(boost::shared_ptr<HullVector> hulls,
                                           boost::shared_ptr<CoefficientsVector> plane_coefficients)
    : PositionFinder(hulls, plane_coefficients)
    {}

bool RaysPlaneIntersector::findPositions(PointCloud::Ptr possible_foot_locations)
{
    for (std::size_t i = 0; i < hulls_->size(); i++)
    {
        auto hull = (*hulls_)[i];
        auto plane_coefficients = (*plane_coefficients_)[i];
        intersectRaysWithHull(hull, plane_coefficients, possible_foot_locations);
    }
    return true;
}

bool RaysPlaneIntersector::intersectRaysWithHull(Hull::Ptr hull,
                                                 ModelCoefficients::Ptr plane_coefficients,
                                                 PointCloud::Ptr possible_foot_locations)
{

    double side = 0.4;
    unsigned int number_of_points = 4;
    // int mask[number_of_points][number_of_points] = {};
    std::vector<std::vector<int>> mask(number_of_points, std::vector<int>(number_of_points));

    auto a = plane_coefficients->values[0];
    auto b = plane_coefficients->values[1];
    auto c = plane_coefficients->values[2];
    auto d = plane_coefficients->values[3];

    auto plane_normal = Eigen::Vector3d(a, b, c);
    auto plane_point = Eigen::Vector3d(0, 0, -d/c);

    for (std::size_t i = 0; i < number_of_points; i++)
    {
        auto x = -side/2.0 + i * (side / (number_of_points - 1));
        for (std::size_t j = 0; j < number_of_points; j++)
        {
            auto y = -side/2.0 + j * (side / (number_of_points - 1));

            auto start = Eigen::Vector3d(x, y, 0.5);
            auto direction = Eigen::Vector3d(0, -sqrt(2)/2, -sqrt(2)/2);
            double t = (plane_normal.dot(plane_point) - plane_normal.dot(start)) / plane_normal.dot(direction);
            Eigen::Vector3d intersect = start + t * direction;

            if (isPointInHull(hull, intersect))
            {
                mask[i][j] = 1;
                possible_foot_locations->push_back(Point(intersect[0], intersect[1], intersect[2]));
            }
        }
    }
    return true;
}


bool RaysPlaneIntersector::isPointInHull(Hull::Ptr hull, Eigen::Vector3d point)
{
    auto hull_size = hull->size();
    bool point_in_hull = true;
    for (std::size_t i = 0; i < hull_size - 1; i++)
    {
        auto j = (i + 1) % hull_size;
        point_in_hull &= isLeftOfEdge((*hull)[i], (*hull)[j], point);
    }
    return point_in_hull;
}

bool RaysPlaneIntersector::isLeftOfEdge(const Point a, const Point b, const Eigen::Vector3d c)
{
    return ((b.x - a.x) * (c[1] - a.y) - (b.y - a.y) * (c[0] - a.x)) < 0;
}
