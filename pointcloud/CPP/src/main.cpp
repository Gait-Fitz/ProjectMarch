#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API
#include <iostream>
#include <algorithm> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include "utilities/realsense_to_pcl.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include "preprocessor.h"
#include "region_creator.h"
#include "hull_finder.h"
#include "position_finder.h"
#include <chrono>
#include <pcl/ModelCoefficients.h>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointIndices = pcl::PointIndices;
using IndicesVector = std::vector<PointIndices::Ptr>;
using MatrixVector = std::vector<Eigen::Matrix3d>;
using Hull = pcl::PointCloud<pcl::PointXYZ>;
using HullVector = std::vector<Hull::Ptr>;
using CoefficientsVector = std::vector<pcl::ModelCoefficients::Ptr>;

int main() 
{

    rs2::pointcloud pc;
    rs2::points points;

    rs2::pipeline pipe;
    pipe.start();

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    viewer->addCoordinateSystem (1.0);

    auto count = 0;
    auto id = 0;
    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        points = pc.calculate(depth);

        auto s0 = std::chrono::high_resolution_clock::now();
        PointCloud::Ptr pointcloud = points_to_pcl(points);
        auto s1 = std::chrono::high_resolution_clock::now();
        std::cout << "IO " << 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(s1-s0).count() << std::endl;

        NormalCloud::Ptr normalcloud(new NormalCloud());
        boost::shared_ptr<IndicesVector> regions = boost::make_shared<IndicesVector>();
        boost::shared_ptr<HullVector> hulls = boost::make_shared<HullVector>();
        boost::shared_ptr<CoefficientsVector> plane_coefficients = boost::make_shared<CoefficientsVector>();
        PointCloud::Ptr possible_foot_locations(new PointCloud());
        
        s0 = std::chrono::high_resolution_clock::now();
        NormalsPreprocessor preprocessor(pointcloud, normalcloud);
        preprocessor.preprocess();
        s1 = std::chrono::high_resolution_clock::now();
        std::cout << "Preprocessor " << 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(s1-s0).count() << std::endl;

        s0 = std::chrono::high_resolution_clock::now();
        SeedRegionGrower seedRegionGrower(pointcloud, normalcloud);
        seedRegionGrower.createRegions(regions);
        s1 = std::chrono::high_resolution_clock::now();
        std::cout << "SeedRegionGrower " << 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(s1-s0).count() << std::endl;

        s0 = std::chrono::high_resolution_clock::now();
        PlaneHullFinder planeHullFinder(pointcloud, normalcloud, regions);
        planeHullFinder.findHulls(hulls, plane_coefficients);
        s1 = std::chrono::high_resolution_clock::now();
        std::cout << "PlaneHullFinder " << 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(s1-s0).count() << std::endl;

        s0 = std::chrono::high_resolution_clock::now();
        RaysPlaneIntersector planeIntersector(hulls, plane_coefficients);
        planeIntersector.findPositions(possible_foot_locations);
        s1 = std::chrono::high_resolution_clock::now();
        std::cout << "RaysPlaneIntersector " << 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(s1-s0).count() << std::endl;

        count++;

        if (count % 1 == 0)
        {

            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            
            viewer->addPointCloud<pcl::PointXYZ> (pointcloud, "pc");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pc");

            viewer->addPointCloud<pcl::PointXYZ> (possible_foot_locations, "footpositions");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "footpositions");

            for (auto &hull : *hulls)
            {
                for (auto i = 0; i < hull->size(); i++)
                {
                    auto j = (i + 1) % hull->size();
                    viewer->addLine<Point>((*hull)[i], (*hull)[j], 0.5, 1.0, 0.5, std::to_string(id));
                    id++;
                }
            }
            
            // if (plane_coefficients->size() > 0)
            // {
            //     double side = 0.4;
            //     int number_of_points = 3;

            //     for (auto i = 0; i < number_of_points; i++)
            //     {
            //         auto x = -side/2.0 + i * (side / (number_of_points - 1));
            //         for (auto j = 0; j < number_of_points; j++)
            //         {
            //             auto y = -side/2.0 + j * (side / (number_of_points - 1));
            //             Eigen::Vector3d D = Eigen::Vector3d(0, -1, -1);
            //             Eigen::Vector3d O = Eigen::Vector3d(x, y, 0.5);
            //             Eigen::Vector3d E = O + 1 * D;
            //             O = O - 0.5 * D;
            //             viewer->addLine<Point>(Point(O[0], O[1], O[2]), Point(E[0], E[1], E[2]), 1.0, 0.4, 0.4, std::to_string(id));
            //             id++;
            //         }
            //     }
            // }
            
            viewer->spinOnce();
            // while (!viewer->wasStopped()) { viewer->spinOnce (100); }
        }

    }
    
    return 0;
}