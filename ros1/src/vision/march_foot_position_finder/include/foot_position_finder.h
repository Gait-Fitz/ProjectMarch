/**
 * @author Tuhin Das - MARCH 7
 */

#ifndef MARCH_FOOT_POSITION_FINDER_H
#define MARCH_FOOT_POSITION_FINDER_H

#include "march_shared_msgs/FootPosition.h"
#include <cmath>
#include <librealsense2/rs.hpp>
#include <march_foot_position_finder/parametersConfig.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class FootPositionFinder {
public:
    explicit FootPositionFinder(
        ros::NodeHandle* n, const std::string& left_or_right);

    void readParameters(
        march_foot_position_finder::parametersConfig& config, uint32_t level);

    ~FootPositionFinder() = default;

protected:
    void chosenPointCallback(const march_shared_msgs::FootPosition msg);

    void processRealSenseDepthFrames(const ros::TimerEvent&);

    void processSimulatedDepthFrames(
        const sensor_msgs::PointCloud2 input_cloud);

    void processPointCloud(const PointCloud::Ptr& pointcloud);

    Point computeTemporalAveragePoint(const Point& new_point);

    Point transformPoint(Point& point, const std::string& frame_from,
        const std::string& frame_to);

    ros::NodeHandle* n_;

    ros::Publisher point_publisher_;
    ros::Subscriber pointcloud_subscriber_;

    ros::Publisher preprocessed_pointcloud_publisher_;
    ros::Publisher point_marker_publisher_;
    ros::Subscriber chosen_point_subscriber_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;

    bool running_;

    ros::Timer realsenseTimer;
    rs2::pipeline pipe_;
    rs2::config config_;
    rs2::context context_;

    rs2::decimation_filter dec_filter_;
    rs2::spatial_filter spat_filter_;
    rs2::temporal_filter temp_filter_;

    std::string topic_camera_front_;
    std::string topic_chosen_point_;

    std::string left_or_right_;
    std::string other_;
    int switch_factor_;
    bool physical_cameras_;

    double foot_gap_;
    double step_distance_;
    double outlier_distance_;
    int sample_size_;

    std::string base_frame_;
    std::string other_frame_id_;
    std::string current_frame_id_;
    std::string current_aligned_frame_id_;
    std::string other_aligned_frame_id_;

    std::vector<Point> found_points_;
    
    double last_height_;


    // Point last_chosen_point_;
    // Point last_chosen_point_world_;
    Point last_displacement_;
    // Point expected_other_point_;
    Point start_point_hip_;
    Point previous_covid_point_hip_;
    // Point start_displacement_world_;
    // Point displacement_world_;
    // Point start_point_;
    // Point desired_point_;

    Point visualize_relative_search_point_;

    Point covid_2;
    Point start_2;
};

#endif // MARCH_FOOT_POSITION_FINDER_H
