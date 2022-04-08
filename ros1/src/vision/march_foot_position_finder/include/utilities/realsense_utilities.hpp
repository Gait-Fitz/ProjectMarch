/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_REALSENSE_UTILITIES
#define MARCH_REALSENSE_UTILITIES

#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

rs2::decimation_filter dec_filter_;
rs2::spatial_filter spat_filter_;
rs2::temporal_filter temp_filter_;

/**
 * Transforms pointclouds from the realsense cameras to pointcloud from the PCL
 * library
 *
 * @param points realsense pointcloud
 * @return PointCloud::Ptr pcl pointcloud
 */
PointCloud::Ptr points_to_pcl(const rs2::points& points)
{
    PointCloud::Ptr cloud = boost::make_shared<PointCloud>();

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto x_min = -0.5;
    auto x_max = 0.5;
    auto y_min = -1;
    auto y_max = 1;
    auto z_min = -0.10;
    auto z_max = 2;

    int point_count = 0;
    auto ptr = points.get_vertices();
    for (std::size_t i = 0; i < points.size(); i++) {
        (*cloud)[point_count].x = ptr->x;
        (*cloud)[point_count].y = ptr->y;
        (*cloud)[point_count].z = ptr->z;
        point_count++;
        ptr++;
    }
    cloud->points.resize(point_count);

    return cloud;
}

/**
 * Connects to a physical realsense device.
 *
 * @param serial_number serial number of the camera
 * @param pipe the pipeline to publish depth frames on
 * @return boolean whether the connection was succesful
 */
bool connectToRealSenseDevice(std::string serial_number, rs2::pipeline& pipe)
{
    rs2::config config_;

    try {
        config_.enable_device(serial_number);
        config_.enable_stream(RS2_STREAM_DEPTH, /*width=*/640,
            /*height=*/480, RS2_FORMAT_Z16, /*framerate=*/15);
        pipe.start(config_);
    } catch (const rs2::error& e) {
        std::string error_message = e.what();
        ROS_WARN("Error while initializing %s RealSense camera: %s",
            serial_number.c_str(), error_message.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        return false;
    }

    return true;
}

/**
 * Retrieves the newest depth frame from a RealSense camera, and converts it to
 * a PCL pointcloud pointer.
 *
 * @param pipe the pipeline to read depth frames from
 * @return PointCloud::Ptr a pointer to a PCL point cloud
 */
PointCloud::Ptr getNextRealsensePointcloud(rs2::pipeline& pipe)
{
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();

    depth = dec_filter_.process(depth);
    depth = spat_filter_.process(depth);
    depth = temp_filter_.process(depth);

    // Allow default constructor for pc
    // NOLINTNEXTLINE
    rs2::pointcloud pc;
    rs2::points points = pc.calculate(depth);

    PointCloud::Ptr pointcloud = points_to_pcl(points);
    return pointcloud;
}

#endif // MARCH_REALSENSE_UTILITIES
