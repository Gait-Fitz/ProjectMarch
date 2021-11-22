#ifndef MARCH_LINEAR_ALGEBRA_UTILITIES_H
#define MARCH_LINEAR_ALGEBRA_UTILITIES_H

#include <cmath>
// #include <ros/ros.h>
#include <vector>

namespace linalg {
// Calculate a dot product of two vectors
template <typename T>
T dotProductVector(std::vector<T> vector1, std::vector<T> vector2);

// Calculate a dot product of two objects with x, y and z attributes
template <typename T> double dotProductPoint(T point1, T point2);

template <typename T> double dotProductNormal(T point1, T point2);

// Calculate a dot product of two objects with x, y and z attributes
template <typename T> double dotProductPoint(T point1, T point2)
{
    return point1.x * point2.x + point1.y * point2.y + point1.z * point2.z;
}

template <typename T> double dotProductNormal(T point1, T point2)
{
    return point1.normal_x * point2.normal_x + point1.normal_y * point2.normal_y
        + point1.normal_z * point2.normal_z;
}

template <typename T> T crossProductNormal(T point1, T point2)
{
    T result;
    result.normal_x = point1.normal_y * point2.normal_z - point1.normal_z * point2.normal_y;
    result.normal_y = -(point1.normal_x * point2.normal_z - point1.normal_z * point2.normal_x);
    result.normal_z = point1.normal_x * point2.normal_y - point1.normal_y * point2.normal_x;
    return result;
}

template <typename T> void normalizeNormal(T &point1)
{
    auto length = sqrt(dotProductNormal(point1, point1));
    if (length != 0)
    {
        point1.normal_x /= length;
        point1.normal_y /= length;
        point1.normal_z /= length;
    }
    return;
}

template <typename T> void matrixVectorDot(Eigen::Matrix3d M, T &point)
{
    Eigen::Vector3d v;
    v << point.x, point.y, point.z;
    auto tmp = M * v;
    point.x = tmp[0];
    point.y = tmp[1];
    point.z = tmp[2];
    return;
}
}

#endif // MARCH_LINEAR_ALGEBRA_UTILITIES_H
