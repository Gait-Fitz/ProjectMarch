/**
 * @author Tuhin Das - MARCH 7
 */

#include "foot_position_finder.h"
#include "point_finder.h"
#include "preprocessor.h"
#include "utilities/math_utilities.hpp"
#include "utilities/publish_utilities.hpp"
#include "utilities/realsense_to_pcl.hpp"
#include <iostream>
#include <march_foot_position_finder/parametersConfig.h>
#include <ros/console.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

/**
 * Constructs an object that listens to simulated or real realsense depth frames
 * and processes these frames with a PointFinder.
 *
 * @param n NodeHandle for running ROS commands
 * @param realsense whether realsense cameras are connected
 * @param left_or_right whether the FootPositionFinder runs for the left or
 * right foot
 */
// No lint is used to allow uninitialized variables (ros parameters)
// NOLINTNEXTLINE
FootPositionFinder::FootPositionFinder(ros::NodeHandle* n,
    const std::string& left_or_right) // NOLINT
    : n_(n)
    , left_or_right_(left_or_right)
{

    if (left_or_right_ == "left") {
        other_ = "right";
        switch_factor_ = 1;
    } else {
        switch_factor_ = -1;
        other_ = "left";
    }

    other_frame_id_ = "foot_" + other_;
    current_frame_id_ = "foot_" + left_or_right;
    current_aligned_frame_id_ = "toes_" + left_or_right_ + "_aligned";
    other_aligned_frame_id_ = "toes_" + other_ + "_aligned";

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);
    topic_camera_front_
        = "/camera_front_" + left_or_right + "/depth/color/points";
    topic_chosen_point_
        = "/chosen_foot_position/" + other_; // in current_aligned_frame_id_

    point_publisher_ = n_->advertise<march_shared_msgs::FootPosition>(
        "/foot_position/" + left_or_right_, /*queue_size=*/1);
    preprocessed_pointcloud_publisher_ = n_->advertise<PointCloud>(
        "/camera_" + left_or_right_ + "/preprocessed_cloud", /*queue_size=*/1);
    point_marker_publisher_ = n_->advertise<visualization_msgs::Marker>(
        "/camera_" + left_or_right_ + "/found_points", /*queue_size=*/1);

    chosen_point_subscriber_
        = n_->subscribe<march_shared_msgs::FootPosition>(topic_chosen_point_,
            /*queue_size=*/1, &FootPositionFinder::chosenPointCallback, this);

    running_ = false;
}

/**
 * Callback for when parameters are updated with ros reconfigure.
 */
// No lint is used to avoid linting in the realsense library, where a potential
// memory leak error is present
// NOLINTNEXTLINE
void FootPositionFinder::readParameters(
    march_foot_position_finder::parametersConfig& config, uint32_t level)
{
    physical_cameras_ = config.physical_cameras;
    base_frame_ = config.base_frame;
    base_frame_ = "world";
    foot_gap_ = config.foot_gap;
    step_distance_ = config.step_distance;
    sample_size_ = config.sample_size;
    outlier_distance_ = config.outlier_distance;
    found_points_.resize(sample_size_);

    // Initialize the depth frame callbacks the first time parameters are read
    if (!running_ && physical_cameras_) {
        config_.enable_stream(RS2_STREAM_DEPTH, /*width=*/640, /*height=*/480,
            RS2_FORMAT_Z16, /*framerate=*/30);

        if (left_or_right_ == "left") {
            config_.enable_device("944622074337");
        } else {
            config_.enable_device("944622071535");
        }

        pipe_.start(config_);
        ROS_INFO("Realsense camera (%s) connected", left_or_right_.c_str());

        realsenseTimer = n_->createTimer(ros::Duration(/*t=*/0.005),
            &FootPositionFinder::processRealSenseDepthFrames, this);

    } else if (!running_ && !physical_cameras_) {
        pointcloud_subscriber_ = n_->subscribe<sensor_msgs::PointCloud2>(
            topic_camera_front_, /*queue_size=*/1,
            &FootPositionFinder::processSimulatedDepthFrames, this);
    }


    start_point_hip_ = Point(/*_x=*/0, /*_y=*/0, /*_z=*/0);
    start_point_hip_ = transformPoint(start_point_hip_, current_aligned_frame_id_, "hip_base_aligned");

    previous_covid_point_hip_ = Point(/*_x=*/0, /*_y=*/0, /*_z=*/0);
    previous_covid_point_hip_ = transformPoint(previous_covid_point_hip_, other_aligned_frame_id_, "hip_base_aligned");

    visualize_relative_search_point_ = rotateRight(transformPoint(start_point_hip_, "hip_base_aligned", "world"));
    start_2 = transformPoint(start_point_hip_, "hip_base_aligned", current_aligned_frame_id_);
    covid_2 = transformPoint(previous_covid_point_hip_, "hip_base_aligned", current_aligned_frame_id_);



    ROS_INFO("Parameters updated in %s foot position finder",
        left_or_right_.c_str());

    running_ = true;

}

/**
 * Callback function for when the gait selection node selects a point.
 */
// Suppress lint error "make reference of argument" (breaks callback)
void FootPositionFinder::chosenPointCallback(
    const march_shared_msgs::FootPosition msg) // NOLINT
{
    start_point_hip_ = Point(msg.point.x, msg.point.y, msg.point.z);
    previous_covid_point_hip_ = Point(msg.point_world.x, msg.point_world.y, msg.point_world.z);
    last_displacement_ = Point(msg.displacement.x, msg.displacement.y, msg.displacement.z);


    visualize_relative_search_point_ = rotateRight(transformPoint(start_point_hip_, "hip_base_aligned", "world"));
    start_2 = transformPoint(start_point_hip_, "hip_base_aligned", current_aligned_frame_id_);
    covid_2 = transformPoint(previous_covid_point_hip_, "hip_base_aligned", current_aligned_frame_id_);
}

/**
 * Listen for realsense frames from a camera, apply filters to them and process
 * the eventual pointcloud.
 */
void FootPositionFinder::processRealSenseDepthFrames(const ros::TimerEvent&)
{
    rs2::frameset frames = pipe_.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();

    depth = dec_filter_.process(depth);
    depth = spat_filter_.process(depth);
    depth = temp_filter_.process(depth);

    // Allow default constructor for pc
    // NOLINTNEXTLINE
    rs2::pointcloud pc;
    rs2::points points = pc.calculate(depth);

    PointCloud::Ptr pointcloud = points_to_pcl(points);
    pointcloud->header.frame_id
        = "camera_front_" + left_or_right_ + "_depth_optical_frame";
    processPointCloud(pointcloud);
}

/**
 * Callback function for when a simulated realsense depth frame arrives.
 */
// Suppress lint error "make reference of argument" (breaks callback)
void FootPositionFinder::processSimulatedDepthFrames(
    const sensor_msgs::PointCloud2 input_cloud) // NOLINT
{
    PointCloud converted_cloud;
    pcl::fromROSMsg(input_cloud, converted_cloud);
    PointCloud::Ptr pointcloud
        = boost::make_shared<PointCloud>(converted_cloud);
    processPointCloud(pointcloud);
}

/**
 * Run a complete processing pipeline for a point cloud with as a result a new
 * point.
 */
void FootPositionFinder::processPointCloud(const PointCloud::Ptr& pointcloud)


//  BLUE ARROW -> from desired (purple) to covid estimate (red)
//  GREEN ARROW -> origin current aligned frame -> last displacement / eventual position other leg (green)

// GREEN POINT: where are we looking for new points with covid, relative to where the other leg will stand in 0.20 s
// GREEN ARROW: where the other foot currently moving to, in the frame of the current leg


{
    NormalCloud::Ptr normalcloud(new NormalCloud());
    Preprocessor preprocessor(n_, pointcloud, normalcloud);
    preprocessor.preprocess();
    // Publish cloud for visualization
    publishCloud(preprocessed_pointcloud_publisher_, *pointcloud);


    // Green = Purple + usual foot displacement

    visualize_relative_search_point_ = rotateRight(transformPoint(start_point_hip_, "hip_base_aligned", "world"));
    publishRelativeSearchPoint(point_marker_publisher_, visualize_relative_search_point_);

    
    Point start_point(covid_2.x - start_2.x,
            covid_2.y - start_2.y,
            covid_2.z - start_2.z);

    Point desired_point_;

    if (left_or_right_ == "left") {
        desired_point_ = Point(start_point.x - (float)step_distance_,
            start_point.y - (float)foot_gap_, start_point.z);
    } else {
        desired_point_ = Point(start_point.x - (float)step_distance_,
            start_point.y + (float)foot_gap_, start_point.z);
    }

    desired_point_ = rotateRight(transformPoint(desired_point_, current_aligned_frame_id_, base_frame_));
    PointFinder pointFinder(n_, pointcloud, left_or_right_, desired_point_);
    std::vector<Point> position_queue;
    pointFinder.findPoints(&position_queue);

    // Publish search region for visualization
    publishSearchRectangle(point_marker_publisher_, desired_point_,
        pointFinder.getDisplacements(), left_or_right_);

    if (position_queue.size() > 0) {
        Point found_covid_point_world = computeTemporalAveragePoint(position_queue[0]);  // red

        // Retrieve 3D points between current and new foot position
        Point start(/*_x=*/0, /*_y=*/0, /*_z=*/0);
        start = transformPoint(start, current_aligned_frame_id_, base_frame_);
        start = rotateRight(start);
        std::vector<Point> track_points
            = pointFinder.retrieveTrackPoints(start, found_covid_point_world);

        // Publish for visualization
        // publishTrackMarkerPoints(point_marker_publisher_, track_points);
        publishMarkerPoint(point_marker_publisher_, found_covid_point_world);
        publishPossiblePoints(point_marker_publisher_, position_queue);

        found_covid_point_world = rotateLeft(found_covid_point_world);
        // the desired displacement? -> blue
        // publishArrow(point_marker_publisher_, expected_other_point_, found_covid_point_world);

        // Point pp1 = Point(0, 0, 0);
        // Point pp2 = last_displacement_;
        // pp1 = transformPoint(pp1, current_aligned_frame_id_, "world");
        // pp2 = transformPoint(pp2, current_aligned_frame_id_, "world");
        // the displacement given by the gait callback -> green arrow
        // publishArrow(point_marker_publisher_, start_displacement_world_, displacement_world_); // blue

        Point found_covid_point_current_aligned
            = transformPoint(found_covid_point_world, base_frame_, current_aligned_frame_id_);

        std::vector<Point> relative_track_points;
        // for (Point& p : track_points) {
        //     Point point = rotateLeft(p);
        //     point = transformPoint(point, base_frame_, current_aligned_frame_id_);
        //     relative_track_points.emplace_back(point);
        // }

        // std::cout << left_or_right_ << std::endl;

        // std::cout << left_or_right_ << std::endl;
        // std::cout << "Covid (world) " << left_or_right_ << std::endl;
        // std::cout << found_covid_point_world.z << std::endl;
        // std::cout << "Previous point" << std::endl;
        // std::cout << start_point_.z << std::endl;
        
        Point displacement = subtractPoints(found_covid_point_current_aligned, start_point);

        if (std::abs(displacement.z) < 0.05) {
            displacement.z = 0;
        }

        start_point.z = 0;
        found_covid_point_current_aligned.z = displacement.z;

        Point found_covid_point_hip = transformPoint(found_covid_point_current_aligned, current_aligned_frame_id_, "hip_base_aligned");
        Point start_point_hip = transformPoint(start_point, current_aligned_frame_id_, "hip_base_aligned");

        publishPoint(point_publisher_, start_point_hip, found_covid_point_hip,
            displacement, relative_track_points);
    }
}

/**
 * Computes a temporal average of the last X points and removes any outliers,
 * then publishes the average of the points without the outliers.
 */
Point FootPositionFinder::computeTemporalAveragePoint(const Point& new_point)
{
    if (found_points_.size() < sample_size_) {
        found_points_.push_back(new_point);
    } else {
        std::rotate(found_points_.begin(), found_points_.begin() + 1,
            found_points_.end());
        found_points_[sample_size_ - 1] = new_point;
        Point avg = computeAveragePoint(found_points_);

        std::vector<Point> non_outliers;
        for (Point& p : found_points_) {
            if (pcl::squaredEuclideanDistance(p, avg) < outlier_distance_) {
                non_outliers.push_back(p);
            }
        }

        if (non_outliers.size() == sample_size_) {
            Point final_point = computeAveragePoint(non_outliers);
            return final_point;
        }
    }
}

/**
 * Transforms a point in place from one frame to another using ROS
 * transformations
 *
 * @param point Point to transform between frames
 * @param frame_from source frame in which the point is currently
 * @param frame_to target frame in which point is transformed
 */
Point FootPositionFinder::transformPoint(
    Point& point, const std::string& frame_from, const std::string& frame_to)
{
    PointCloud::Ptr desired_point = boost::make_shared<PointCloud>();
    desired_point->push_back(point);

    geometry_msgs::TransformStamped transform_stamped;

    try {
        if (tfBuffer_->canTransform(frame_to, frame_from, ros::Time(/*t=*/0),
                ros::Duration(/*t=*/1.0))) {
            transform_stamped = tfBuffer_->lookupTransform(
                frame_to, frame_from, ros::Time(/*t=*/0));
        }
    } catch (tf2::TransformException& ex) {
        ROS_WARN_STREAM(
            "Something went wrong when transforming the pointcloud: "
            << ex.what());
    }

    pcl_ros::transformPointCloud(
        *desired_point, *desired_point, transform_stamped.transform);

    return desired_point->points[0];
}
