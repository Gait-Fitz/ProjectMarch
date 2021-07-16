#include "pointcloud_processor/parameter_determiner.h"
#include "march_shared_msgs/GaitParameters.h"
#include "pointcloud_processor/parameter_determiner.h"
#include "utilities/color_utilities.h"
#include "utilities/linear_algebra_utilities.h"
#include "utilities/output_utilities.h"
#include "utilities/realsense_category_utilities.h"
#include "yaml-cpp/yaml.h"
#include <cmath>
#include <ctime>
#include <pcl/filters/crop_hull.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <utility>

#define EPSILON 0.0001
#define DEBUG_MARKER_SIZE 0.03

using PointCloud2D = pcl::PointCloud<pcl::PointXY>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using Normals = pcl::PointCloud<pcl::Normal>;
using Region = pcl::PointIndices;
using PlaneCoefficients = pcl::ModelCoefficients;
using LineCoefficients = pcl::ModelCoefficients;
using Hull = pcl::PointCloud<pcl::PointXYZ>;
using Polygon = std::vector<pcl::Vertices>;
using RegionVector = std::vector<Region>;
using PlaneCoefficientsVector = std::vector<PlaneCoefficients::Ptr>;
using HullVector = std::vector<Hull::Ptr>;
using PolygonVector = std::vector<Polygon>;
using GaitParameters = march_shared_msgs::GaitParameters;

ParameterDeterminer::ParameterDeterminer(bool debugging)
    : debugging_ { debugging }
{
}

// Construct a basic HullParameterDeterminer class
HullParameterDeterminer::HullParameterDeterminer(bool debugging)
    : ParameterDeterminer(debugging)
{
}

void HullParameterDeterminer::readParameters(
    march_realsense_reader::pointcloud_parametersConfig& config)
{
    number_of_optional_foot_locations
        = config.parameter_determiner_foot_locations;
    hull_dimension = config.hull_dimension;

    min_x_stairs_up
        = (float)config.parameter_determiner_stairs_up_locations_min_x;
    max_x_stairs_up
        = (float)config.parameter_determiner_stairs_up_locations_max_x;
    min_z_stairs_up
        = (float)config.parameter_determiner_stairs_up_locations_min_z;
    max_z_stairs_up
        = (float)config.parameter_determiner_stairs_up_locations_max_z;

    general_most_desirable_location_is_mid
        = config.parameter_determiner_most_desirable_loc_is_mid;
    general_most_desirable_location_is_small
        = config.parameter_determiner_most_desirable_loc_is_small;

    foot_length_back = (float)config.parameter_determiner_foot_length_back;
    foot_length_front = (float)config.parameter_determiner_foot_length_front;
    foot_width = (float)config.parameter_determiner_foot_width;
    hull_dimension = config.hull_dimension;

    ramp_min_search_area
        = (float)config.parameter_determiner_ramp_min_search_area;
    ramp_max_search_area
        = (float)config.parameter_determiner_ramp_max_search_area;

    x_flat_down = (float)config.parameter_determiner_ramp_x_flat_down;
    z_flat_down = (float)config.parameter_determiner_ramp_z_flat_down;
    x_steep_down = (float)config.parameter_determiner_ramp_x_steep_down;
    z_steep_down = (float)config.parameter_determiner_ramp_z_steep_down;

    x_flat_up = (float)config.parameter_determiner_ramp_x_flat_up;
    z_flat_up = (float)config.parameter_determiner_ramp_z_flat_up;
    x_steep_up = (float)config.parameter_determiner_ramp_x_steep_up;
    z_steep_up = (float)config.parameter_determiner_ramp_z_steep_up;

    y_location = (float)config.parameter_determiner_y_location;

    max_allowed_z_deviation_foot
        = (float)config.parameter_determiner_max_allowed_z_deviation_foot;
    max_distance_to_line
        = (float)config.parameter_determiner_ramp_max_distance_to_line;

    debugging_ = config.debug;
}

/** This function takes in a pointcloud with matching normals and
 * hulls, and turn this into a location where the foot can be placed,
 * from this location, gaits parameters are made. **/
bool HullParameterDeterminer::determineParameters(
    boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
    boost::shared_ptr<HullVector> const hull_vector,
    boost::shared_ptr<PolygonVector> const polygon_vector,
    RealSenseCategory const realsense_category,
    boost::shared_ptr<GaitParameters> gait_parameters,
    std::string frame_id_to_transform_to, std::string subgait_name)
{
    time_t start_determine_parameters = clock();

    ROS_DEBUG("Determining parameters with hull parameter determiner");

    hull_vector_ = hull_vector;
    gait_parameters_ = gait_parameters;
    plane_coefficients_vector_ = plane_coefficients_vector;
    polygon_vector_ = polygon_vector;
    realsense_category_.emplace(realsense_category);
    subgait_name_ = subgait_name;
    frame_id_to_transform_to_ = frame_id_to_transform_to;
    // Initialize the optimal foot location at the origin and the gait
    // parameters at -1 in case the calculation fails
    optimal_foot_location = pcl::PointNormal();
    gait_parameters_->first_parameter = -1;
    gait_parameters_->second_parameter = -1;
    gait_parameters_->side_step_parameter = -1;

    // Since the parameter determining for e.g. ramp down is very similar to
    // ramp up set variables like the size a step on a flat ramp equal to the
    // relevant (up or down) value and continue treating ramp up and ramp down
    // the same
    initializeGaitDimensions();

    if (debugging_) {
        initializeDebugOutput();
        addDebugGaitInformation();
    }

    bool success = true;

    // Only calculate the gait parameters if an optimal foot location has been
    // found
    if (success &= getOptimalFootLocation()) {
        success &= getGaitParametersFromFootLocation();
    }

    if (debugging_) {
        addDebugMarkersToArray();
    }

    if (success) {
        ROS_DEBUG_STREAM("The optimal foot location is "
            << output_utilities::pointToString(optimal_foot_location)
            << "\n With corresponding parameters (size, height, side) ("
            << gait_parameters_->first_parameter << ", "
            << gait_parameters_->second_parameter << ", "
            << gait_parameters_->side_step_parameter << ") ");
    }

    time_t end_determine_parameters = clock();
    double time_taken
        = double(end_determine_parameters - start_determine_parameters)
        / double(CLOCKS_PER_SEC);
    ROS_DEBUG_STREAM("Time taken by the hull parameter determiner,   is : "
        << std::fixed << time_taken << std::setprecision(5) << " sec "
        << std::endl);

    return success;
};

void HullParameterDeterminer::initializeDebugOutput()
{
    visualization_msgs::MarkerArray debug_marker_array;

    int id = 0;
    foot_locations_to_try_marker_list = initializeMarkerListWithId(id);

    id = 1;
    possible_foot_locations_marker_list = initializeMarkerListWithId(id);

    id = 2;
    optimal_foot_location_marker = initializeMarkerListWithId(id);
    // Make the optimal foot location stand out more
    optimal_foot_location_marker.scale.x = DEBUG_MARKER_SIZE * 1.2;
    optimal_foot_location_marker.scale.y = DEBUG_MARKER_SIZE * 1.2;
    optimal_foot_location_marker.scale.z = DEBUG_MARKER_SIZE * 1.2;

    id = 3;
    gait_information_marker_list = initializeMarkerListWithId(id);
}

visualization_msgs::Marker HullParameterDeterminer::initializeMarkerListWithId(
    int id)
{
    visualization_msgs::Marker marker_list;
    marker_list.id = id;
    marker_list.header.frame_id = frame_id_to_transform_to_;
    // Places the marker up right (axis aligned with that of its frame id)
    marker_list.pose.orientation.w = 1.0;
    marker_list.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_list.scale.x = DEBUG_MARKER_SIZE;
    marker_list.scale.y = DEBUG_MARKER_SIZE;
    marker_list.scale.z = DEBUG_MARKER_SIZE;
    return marker_list;
}

void HullParameterDeterminer::addDebugGaitInformation()
{
    std_msgs::ColorRGBA marker_color = color_utilities::RED;

    switch (realsense_category_.value()) {
        case RealSenseCategory::stairs_down:
        case RealSenseCategory::stairs_up: {
            geometry_msgs::Point marker_point;
            marker_point.y = y_location;

            marker_point.x = min_x_stairs;
            marker_point.z = min_z_stairs;
            gait_information_marker_list.points.push_back(marker_point);
            gait_information_marker_list.colors.push_back(marker_color);

            marker_point.x = max_x_stairs;
            marker_point.z = min_z_stairs;
            gait_information_marker_list.points.push_back(marker_point);
            gait_information_marker_list.colors.push_back(marker_color);

            marker_point.x = min_x_stairs;
            marker_point.z = max_z_stairs;
            gait_information_marker_list.points.push_back(marker_point);
            gait_information_marker_list.colors.push_back(marker_color);

            marker_point.x = max_x_stairs;
            marker_point.z = max_z_stairs;
            gait_information_marker_list.points.push_back(marker_point);
            gait_information_marker_list.colors.push_back(marker_color);
            break;
        }
        case RealSenseCategory::ramp_down:
        case RealSenseCategory::ramp_up: {
            geometry_msgs::Point marker_point;
            marker_point.y = y_location;

            marker_point.x = x_flat;
            marker_point.z = z_flat;
            gait_information_marker_list.points.push_back(marker_point);
            gait_information_marker_list.colors.push_back(marker_color);

            marker_point.x = x_steep;
            marker_point.z = z_steep;
            gait_information_marker_list.points.push_back(marker_point);
            gait_information_marker_list.colors.push_back(marker_color);
            break;
        }
        default: {
            ROS_WARN_STREAM("gait debug information is not implemented yet "
                            "for realsense category "
                << realsense_category_.value());
        }
    }
}

void HullParameterDeterminer::addDebugMarkersToArray()
{
    debug_marker_array.markers.push_back(foot_locations_to_try_marker_list);
    debug_marker_array.markers.push_back(possible_foot_locations_marker_list);
    debug_marker_array.markers.push_back(optimal_foot_location_marker);
    debug_marker_array.markers.push_back(gait_information_marker_list);
}

void HullParameterDeterminer::initializeGaitDimensions()
{
    switch (realsense_category_.value()) {
        case RealSenseCategory::ramp_down: {
            x_flat = x_flat_down;
            x_steep = x_steep_down;
            z_flat = z_flat_down;
            z_steep = z_steep_down;
            break;
        }
        case RealSenseCategory::ramp_up: {
            x_flat = x_flat_up;
            x_steep = x_steep_up;
            z_flat = z_flat_up;
            z_steep = z_steep_up;
            break;
        }
        case RealSenseCategory::stairs_up: {
            min_x_stairs = min_x_stairs_up;
            max_x_stairs = max_x_stairs_up;
            min_z_stairs = min_z_stairs_up;
            max_z_stairs = max_z_stairs_up;
            break;
        }
        case RealSenseCategory::stairs_down: {
            min_x_stairs = min_x_stairs_up;
            max_x_stairs = max_x_stairs_up;
            // Because the sign gets flipped, the minimum value becomes the
            // maximum value.
            max_z_stairs = -min_z_stairs_up;
            min_z_stairs = -max_z_stairs_up;
            break;
        }
    }
    // If the subgait is a swing subgait, double the gait parameters as
    // the step size (and height) of a swing subgait are twice that of
    // an open or close subgait
    // Do the same when the gait name is left open as this subgait
    // also traverses twice the distance of a normal open gait
    if (subgait_name_.substr(subgait_name_.size() - 5) == "swing"
        || subgait_name_ == "left_open") {
        x_flat *= 2;
        x_steep *= 2;
        z_flat *= 2;
        z_steep *= 2;
        min_x_stairs *= 2;
        max_x_stairs *= 2;
        min_z_stairs *= 2;
        max_z_stairs *= 2;
    }
}

// Find the parameters from the foot location by finding at what percentage of
// the end points it is
bool HullParameterDeterminer::getGaitParametersFromFootLocation()
{
    bool success = true;
    switch (realsense_category_.value()) {
        case RealSenseCategory::stairs_down:
        case RealSenseCategory::stairs_up: {
            success &= getGaitParametersFromFootLocationStairs();
            break;
        }
        case RealSenseCategory::ramp_down:
        case RealSenseCategory::ramp_up: {
            success &= getGaitParametersFromFootLocationRamp();
            break;
        }
        default: {
            ROS_ERROR_STREAM(
                "No way to transform a foot location to parameters "
                "is implemented yet for obstacle "
                << realsense_category_.value());
            return false;
        }
    }
    return success;
}
bool HullParameterDeterminer::getGaitParametersFromFootLocationStairs()
{
    gait_parameters_->first_parameter = (optimal_foot_location.x - min_x_stairs)
        / (max_x_stairs - min_x_stairs);
    gait_parameters_->second_parameter
        = (optimal_foot_location.z - min_z_stairs)
        / (max_z_stairs - min_z_stairs);
    // The side step parameter is unused for the stairs gait so we set it to -1
    gait_parameters_->side_step_parameter = -1;
    // As we interpret the second (height) parameter as being high when the
    // stair is steep, flip it for the stairs down gait as it is one when the
    // optimal location is close to the highest (not absolute) value.
    if (realsense_category_.value() == RealSenseCategory::ramp_down) {
        gait_parameters_->second_parameter
            = 1 - gait_parameters_->second_parameter;
    }
    return true;
}

bool HullParameterDeterminer::getGaitParametersFromFootLocationRamp()
{
    // As we can only execute gaits in a certain line,
    // project to the line and find where on the line the point falls.
    // The distance to the line is capped by max_distance_to_line
    pcl::PointXYZ projected_optimal_foot_location
        = linear_algebra_utilities::projectPointToLine(
            optimal_foot_location, executable_locations_line_coefficients_);

    optimal_foot_location.x = projected_optimal_foot_location.x;
    optimal_foot_location.y = projected_optimal_foot_location.y;
    optimal_foot_location.z = projected_optimal_foot_location.z;

    double parameter_from_x
        = (optimal_foot_location.x - x_flat) / (x_steep - x_flat);
    double parameter_from_z
        = (optimal_foot_location.z - z_flat) / (z_steep - z_flat);

    if (parameter_from_x - parameter_from_z < EPSILON) {
        gait_parameters_->first_parameter
            = (optimal_foot_location.x - x_flat) / (x_steep - x_flat);
    } else {
        ROS_ERROR_STREAM(
            "The optimal foot location for the ramp gait was not on a linear "
            "line "
            "between the flat and steep gait, unable to determine parameter.");
        return false;
    }

    // The step height and side step parameter are unused for the ramp down
    // gait, so they are set to -1
    gait_parameters_->second_parameter = -1;
    gait_parameters_->side_step_parameter = -1;
    return true;
}

// Get the optimal foot location by finding which possible foot location is
// closest to the most desirable foot location
bool HullParameterDeterminer::getOptimalFootLocation()
{
    bool success = true;
    // Get some locations on the ground we might want to place our foot
    foot_locations_to_try = boost::make_shared<PointCloud2D>();
    success &= getOptionalFootLocations(foot_locations_to_try);

    // Crop those locations to only be left with locations where it is possible
    // to place the foot
    possible_foot_locations = boost::make_shared<PointNormalCloud>();
    success &= cropCloudToHullVectorUnique(
        foot_locations_to_try, possible_foot_locations);

    success &= getOptimalFootLocationFromPossibleLocations();

    if (debugging_) {
        geometry_msgs::Point marker_point;
        marker_point.x = optimal_foot_location.x;
        marker_point.y = optimal_foot_location.y;
        marker_point.z = optimal_foot_location.z;
        std_msgs::ColorRGBA marker_color = color_utilities::WHITE;

        optimal_foot_location_marker.points.push_back(marker_point);
        optimal_foot_location_marker.colors.push_back(marker_color);
    }

    return success;
}

// Get the optimal foot location by selecting one from the possible locations
bool HullParameterDeterminer::getOptimalFootLocationFromPossibleLocations()
{
    bool success = true;
    switch (realsense_category_.value()) {
        case RealSenseCategory::stairs_down:
        case RealSenseCategory::stairs_up: {
            // Get the location where we would ideally place the foot
            success &= getGeneralMostDesirableLocation();

            // Get the possible location which is closest to the ideal location
            success &= getPossibleMostDesirableLocation();
            break;
        }
        case RealSenseCategory::ramp_down:
        case RealSenseCategory::ramp_up: {
            // Get the line on which it is possible to stand for a ramp gait.
            success &= getExecutableLocationsLine();

            // Get the possible location which is closest to the line
            success &= getPossibleMostDesirableLocation();
            break;
        }
        default: {
            ROS_ERROR_STREAM("getOptimalFootLocationFromPossibleLocations method is not implemented "
                             "for selected obstacle "
                << realsense_category_.value());
            return false;
        }
    }
    return success;
}

bool HullParameterDeterminer::getExecutableLocationsLine()
{
    // Interpreted as (x(t), y(t), z(t))^T = ([0], [1], [2])^T * t + ([3], [4],
    // [5])^T
    executable_locations_line_coefficients_->values.resize(/*__new_size=*/6);

    executable_locations_line_coefficients_->values[0] = x_flat - x_steep;
    executable_locations_line_coefficients_->values[1] = 0;
    executable_locations_line_coefficients_->values[2] = z_flat - z_steep;

    executable_locations_line_coefficients_->values[3] = x_steep;
    executable_locations_line_coefficients_->values[4] = y_location;
    executable_locations_line_coefficients_->values[5] = z_steep;

    return true;
}

// From the possible foot locations, find which one is closes to some object
// For the stair gaits this object is a most desirable location
// For the ramp gait this is the possible locations line
bool HullParameterDeterminer::getPossibleMostDesirableLocation()
{
    bool success = true;
    if (possible_foot_locations->points.size() == 0) {
        ROS_ERROR_STREAM(
            "The possible foot locations cloud is empty. "
            "Unable to compute corresponding possible foot locations");
        return false;
    }

    double min_distance_to_object = std::numeric_limits<double>::max();
    double distance_to_object;

    for (pcl::PointNormal& possible_foot_location : *possible_foot_locations) {
        if (not isValidLocation(possible_foot_location)) {
            continue;
        }

        success
            &= getDistanceToObject(possible_foot_location, distance_to_object);

        if (distance_to_object < min_distance_to_object) {
            min_distance_to_object = distance_to_object;
            optimal_foot_location = possible_foot_location;
        }
    }
    if (min_distance_to_object != std::numeric_limits<double>::max()) {
        ROS_DEBUG_STREAM("The optimal foot location is "
            << min_distance_to_object << " removed from its ideal location");
        return success;
    } else {
        ROS_ERROR_STREAM("No valid foot location could be found for the "
                         "current selected gait "
            << realsense_category_.value());
        return false;
    }
}

// get the distance from a location to some object depending on the obstacle
bool HullParameterDeterminer::getDistanceToObject(
    pcl::PointNormal possible_foot_location, double& distance)
{
    switch (realsense_category_.value()) {
        case RealSenseCategory::stairs_up:
        case RealSenseCategory::stairs_down: {
            // For stairs gait find which point is closest to the most desirable
            // location
            distance = linear_algebra_utilities::distanceBetweenPoints(
                possible_foot_location, most_desirable_foot_location_);
            break;
        }
        case RealSenseCategory::ramp_up:
        case RealSenseCategory::ramp_down: {
            // For the ramp find which point is closest to the possible
            // locations line
            distance = linear_algebra_utilities::distancePointToLine(
                possible_foot_location,
                executable_locations_line_coefficients_);
            break;
        }
        default: {
            ROS_ERROR_STREAM("getDistanceToObject method is not implemented "
                             "for selected obstacle "
                << realsense_category_.value());
            distance = std::numeric_limits<double>::max();
            return false;
        }
    }
    return true;
}

// Verify that the found location is valid for the requested gait
bool HullParameterDeterminer::isValidLocation(
    pcl::PointNormal possible_foot_location)
{
    // Less and larger than signs are swapped for the x coordinate as the
    // positive x axis points in the backwards direction of the exoskeleton
    switch (realsense_category_.value()) {
        case RealSenseCategory::stairs_down:
        case RealSenseCategory::stairs_up: {

            if (debugging_) {
                geometry_msgs::Point marker_point;
                marker_point.x = possible_foot_location.x;
                marker_point.y = possible_foot_location.y;
                marker_point.z = possible_foot_location.z;

                std_msgs::ColorRGBA marker_color;
                if (!(possible_foot_location.x < min_x_stairs
                        && possible_foot_location.x > max_x_stairs
                        && possible_foot_location.z > min_z_stairs
                        && possible_foot_location.z < max_z_stairs)) {
                    marker_color = color_utilities::YELLOW;
                } else if (!entireFootCanBePlaced(possible_foot_location)) {
                    marker_color = color_utilities::PURPLE;
                } else {
                    marker_color = color_utilities::GREEN;
                }
                possible_foot_locations_marker_list.points.push_back(
                    marker_point);
                possible_foot_locations_marker_list.colors.push_back(
                    marker_color);
            }

            // A possible foot location for the stairs gait is valid if it is
            // reachable by the stairs gait and the location offers support
            // for the entire foot
            return (possible_foot_location.x < min_x_stairs
                && possible_foot_location.x > max_x_stairs
                && possible_foot_location.z > min_z_stairs
                && possible_foot_location.z < max_z_stairs
                && entireFootCanBePlaced(possible_foot_location));
        }
        case RealSenseCategory::ramp_down:
        case RealSenseCategory::ramp_up: {
            pcl::PointXYZ projected_point
                = linear_algebra_utilities::projectPointToLine(
                    possible_foot_location,
                    executable_locations_line_coefficients_);
            double distance = linear_algebra_utilities::distanceBetweenPoints(
                projected_point, possible_foot_location);

            if (debugging_) {
                geometry_msgs::Point marker_point;
                marker_point.x = possible_foot_location.x;
                marker_point.y = possible_foot_location.y;
                marker_point.z = possible_foot_location.z;

                std_msgs::ColorRGBA marker_color;
                if (!(projected_point.x < x_steep
                        && projected_point.x > x_flat)) {
                    marker_color = color_utilities::YELLOW;
                } else if (!(distance < max_distance_to_line)) {
                    marker_color = color_utilities::PURPLE;
                } else {
                    marker_color = color_utilities::GREEN;
                }
                possible_foot_locations_marker_list.points.push_back(
                    marker_point);
                possible_foot_locations_marker_list.colors.push_back(
                    marker_color);
            }

            // only points which are close enough to the line are valid
            // Only points on the line which are between the two given values
            // are valid
            return (projected_point.x < x_steep && projected_point.x > x_flat
                && distance < max_distance_to_line);
        }
        default: {
            ROS_ERROR_STREAM(
                "isValidLocation method has not been implemented for obstacle "
                << realsense_category_.value() << ". Returning false.");
            return false;
        }
    }
}

// Verify if there is support for the entire foot around the possible foot
// location
bool HullParameterDeterminer::entireFootCanBePlaced(
    pcl::PointNormal possible_foot_location)
{
    bool success = true;
    // First create a pointcloud containing the edge points (vertices) of the
    // foot on the ground
    PointCloud2D::Ptr foot_pointcloud = boost::make_shared<PointCloud2D>();
    fillFootPointCloud(foot_pointcloud, possible_foot_location);

    // Then find possible foot locations associated with the foot vertices
    PointNormalCloud::Ptr potential_foot_support_cloud
        = boost::make_shared<PointNormalCloud>();
    success &= cropCloudToHullVectorUnique(
        foot_pointcloud, potential_foot_support_cloud);

    // The location is only valid if all foot vertices can be placed
    success
        &= (potential_foot_support_cloud->size() == foot_pointcloud->size());

    // The location is only valid if the foot vertices have a z value close
    // enough to the locations z value
    for (pcl::PointNormal potential_foot_support :
        *potential_foot_support_cloud) {
        success &= (abs(potential_foot_support.z - possible_foot_location.z)
            < max_allowed_z_deviation_foot);
    }
    return success;
}

// Fill a point cloud with vertices of the foot on the ground around a possible
// foot location
void HullParameterDeterminer::fillFootPointCloud(
    const PointCloud2D::Ptr& foot_pointcloud,
    pcl::PointNormal possible_foot_location)
{
    foot_pointcloud->points.resize(/*__new_size=*/4);

    // Deviation back is added as the forward direction of the exoskeleton
    // is the negative x direction in the simulation
    foot_pointcloud->points[0].x = possible_foot_location.x + foot_length_back;
    foot_pointcloud->points[0].y = possible_foot_location.y - foot_width / 2.0F;

    foot_pointcloud->points[1].x = possible_foot_location.x + foot_length_back;
    foot_pointcloud->points[1].y = possible_foot_location.y + foot_width / 2.0F;

    // Deviation front is subtracted as the forward direction of the exoskeleton
    // is the negative x direction in the simulation
    foot_pointcloud->points[2].x = possible_foot_location.x - foot_length_front;
    foot_pointcloud->points[2].y = possible_foot_location.y - foot_width / 2.0F;

    foot_pointcloud->points[3].x = possible_foot_location.x - foot_length_front;
    foot_pointcloud->points[3].y = possible_foot_location.y + foot_width / 2.0F;
}

// Compute the optimal foot location as if one were not limited by anything.
bool HullParameterDeterminer::getGeneralMostDesirableLocation()
{
    if (general_most_desirable_location_is_mid) {
        most_desirable_foot_location_.x = (min_x_stairs + max_x_stairs) / 2.0F;
        most_desirable_foot_location_.y = y_location;
        most_desirable_foot_location_.z = (min_z_stairs + max_z_stairs) / 2.0F;
    } else if (general_most_desirable_location_is_small) {
        most_desirable_foot_location_.x = min_x_stairs;
        most_desirable_foot_location_.y = y_location;
        most_desirable_foot_location_.z = min_z_stairs;
    } else {
        ROS_ERROR_STREAM(
            "No method for finding the general most desirable foot location "
            "was given. "
            "Unable to compute general most desirable foot location.");
        return false;
    }
    if (debugging_) {
        std_msgs::ColorRGBA marker_color = color_utilities::RED;
        geometry_msgs::Point marker_point;
        marker_point.x = most_desirable_foot_location_.x;
        marker_point.y = most_desirable_foot_location_.y;
        marker_point.z = most_desirable_foot_location_.z;

        gait_information_marker_list.points.push_back(marker_point);
        gait_information_marker_list.colors.push_back(marker_color);
    }
    return true;
}

// Create a point cloud with points on the ground where the points represent
// where it should be checked if there is a valid foot location
bool HullParameterDeterminer::getOptionalFootLocations(
    const PointCloud2D::Ptr& foot_locations_to_try)
{
    bool success = true;
    foot_locations_to_try->points.resize(number_of_optional_foot_locations);
    switch (realsense_category_.value()) {
        case RealSenseCategory::stairs_down:
        case RealSenseCategory::stairs_up: {
            success
                &= fillOptionalFootLocationCloud(min_x_stairs, max_x_stairs);
            break;
        }
        case RealSenseCategory::ramp_down:
        case RealSenseCategory::ramp_up: {
            // A point further than x_flat can never be project on the right
            // part of the potential foot locations line assuming that the
            // x_flat > x_steep and z_flat < z_steep
            success &= fillOptionalFootLocationCloud(
                ramp_min_search_area, ramp_max_search_area);
            break;
        }
        default: {
            ROS_ERROR_STREAM("The selected obstacle "
                << realsense_category_.value()
                << " does not have a way to create the optional foot locations "
                   "to try cloud");
            return false;
        }
    }
    return success;
}

// Fill the foot locations to try cloud with a line of points from (start, 0) to
// (end, 0)
bool HullParameterDeterminer::fillOptionalFootLocationCloud(
    float start, float end)
{
    if (number_of_optional_foot_locations == 0) {
        ROS_WARN_STREAM(
            "The number of optional foot locations parameter is set to 0, "
            "not filling the foot_locations_to_try cloud");
        return false;
    }
    for (int i = 0; i < number_of_optional_foot_locations; i++) {
        float x_location = start
            + (end - start) * (float)i
                / ((float)number_of_optional_foot_locations - 1.0F);
        foot_locations_to_try->points[i].x = x_location;
        foot_locations_to_try->points[i].y = y_location;

        if (debugging_) {
            geometry_msgs::Point marker_point;
            marker_point.x = x_location;
            marker_point.y = y_location;
            marker_point.z = 0;

            std_msgs::ColorRGBA marker_color = color_utilities::BLUE;

            foot_locations_to_try_marker_list.points.push_back(marker_point);
            foot_locations_to_try_marker_list.colors.push_back(marker_color);
        }
    }
    return true;
}

/** For each hull, the input cloud's z coordinate is set so that it
 * lies on the corresponding plane, then the input cloud is cropped, the points
 * inside the hull (the cropped cloud) are moved to the output cloud with the
 * normal of the plane This process is repeated for each hull. If each point in
 * the input_cloud has been moved to the output cloud,
 * result is set to true, it is set to false otherwise **/
bool HullParameterDeterminer::cropCloudToHullVector(
    PointCloud2D::Ptr const& input_cloud,
    const PointNormalCloud::Ptr& output_cloud)
{
    if (input_cloud->points.size() == 0) {
        ROS_WARN_STREAM("cropCloudToHullVector method called with an input "
                        "cloud of size zero. "
                        "No cropping can be done, returning.");
        return false;
    } else if (hull_vector_->size() == 0) {
        ROS_WARN_STREAM(
            "cropCloudToHull method called with emtpy hull_vector_. "
            "No cropping can be done, returning.");
        return false;
    }
    bool success = true;
    for (int hull_index = 0; hull_index < hull_vector_->size(); hull_index++) {
        PointCloud::Ptr elevated_cloud = boost::make_shared<PointCloud>();
        success &= addZCoordinateToCloudFromPlaneCoefficients(input_cloud,
            plane_coefficients_vector_->at(hull_index), elevated_cloud);

        success &= cropCloudToHull(elevated_cloud, hull_vector_->at(hull_index),
            polygon_vector_->at(hull_index));

        PointNormalCloud::Ptr elevated_cloud_with_normals
            = boost::make_shared<PointNormalCloud>();
        success &= addNormalToCloudFromPlaneCoefficients(elevated_cloud,
            plane_coefficients_vector_->at(hull_index),
            elevated_cloud_with_normals);

        *output_cloud += *elevated_cloud_with_normals;
    }

    return success;
}

// Crops a single point to a hull vector.
bool HullParameterDeterminer::cropPointToHullVector(
    pcl::PointXY const input_point, const PointNormalCloud::Ptr& output_cloud)
{
    PointCloud2D::Ptr input_cloud = boost::make_shared<PointCloud2D>();
    input_cloud->push_back(input_point);

    bool success = cropCloudToHullVector(input_cloud, output_cloud);
    return success;
}

// Crops a cloud to a hull vector, but only puts each input point in
// the highest hull it falls into
bool HullParameterDeterminer::cropCloudToHullVectorUnique(
    PointCloud2D::Ptr const& input_cloud,
    const PointNormalCloud::Ptr& output_cloud)
{
    bool success = true;

    for (pcl::PointXY ground_point : *input_cloud) {
        PointNormalCloud::Ptr potential_foot_locations_of_point
            = boost::make_shared<PointNormalCloud>();
        success &= HullParameterDeterminer::cropPointToHullVector(
            ground_point, potential_foot_locations_of_point);

        if (potential_foot_locations_of_point->points.size() != 0) {
            auto result
                = std::max_element(potential_foot_locations_of_point->begin(),
                    potential_foot_locations_of_point->end(),
                    linear_algebra_utilities::pointIsLower);
            output_cloud->push_back(*result);
        }
    }
    return success;
}

// Elevate the 2D points so they have z coordinate as if they lie on the plane
// of the hull
bool HullParameterDeterminer::addZCoordinateToCloudFromPlaneCoefficients(
    PointCloud2D::Ptr const& input_cloud,
    PlaneCoefficients::Ptr const& plane_coefficients,
    const PointCloud::Ptr& elevated_cloud)
{
    elevated_cloud->points.resize(input_cloud->points.size());

    int point_index = 0;
    for (pcl::PointXYZ& elevated_point : *elevated_cloud) {
        // using z = - (d + by + ax) / c from plane equation ax + by + cz + d =
        // 0
        pcl::PointXY input_point = input_cloud->points[point_index];
        elevated_point.x = input_point.x;
        elevated_point.y = input_point.y;
        elevated_point.z = -(plane_coefficients->values[3]
                               + plane_coefficients->values[1]
                                   * input_cloud->points[point_index].y
                               + plane_coefficients->values[0]
                                   * input_cloud->points[point_index].x)
            / plane_coefficients->values[2];

        point_index++;
    }
    return true;
}

// Remove all points from a cloud which do not fall in the hull
bool HullParameterDeterminer::cropCloudToHull(
    const PointCloud::Ptr& elevated_cloud, const Hull::Ptr& hull,
    const Polygon& polygon)
{
    if (elevated_cloud->points.size() == 0) {
        ROS_WARN_STREAM("The cloud to be cropped in the "
                        "HullParameterDeterminer contains no points.");
        return false;
    }
    pcl::CropHull<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(elevated_cloud);
    crop_filter.setHullCloud(hull);
    crop_filter.setHullIndices(polygon);
    crop_filter.setDim(hull_dimension);
    crop_filter.filter(*elevated_cloud);
    return true;
}

// Add normals to the elevated cloud which correspond to the normal vector of
// the plane
bool HullParameterDeterminer::addNormalToCloudFromPlaneCoefficients(
    PointCloud::Ptr const& elevated_cloud,
    PlaneCoefficients::Ptr const& plane_coefficients,
    const PointNormalCloud::Ptr& elevated_cloud_with_normals)
{
    elevated_cloud_with_normals->width = elevated_cloud->width;
    elevated_cloud_with_normals->height = elevated_cloud->height;
    elevated_cloud_with_normals->points.resize(elevated_cloud->points.size());

    float normalising_constant
        = plane_coefficients->values[0] * plane_coefficients->values[0]
        + plane_coefficients->values[1] * plane_coefficients->values[1]
        + plane_coefficients->values[2] * plane_coefficients->values[2];

    if (normalising_constant < std::numeric_limits<double>::epsilon()) {
        ROS_ERROR_STREAM("The normal vector of the current plane is too close "
                         "to the zero vector.");
        return false;
    }

    int point_index = 0;
    for (pcl::PointNormal& elevated_point_with_normal :
        *elevated_cloud_with_normals) {
        pcl::PointXYZ elevated_point = elevated_cloud->points[point_index];
        elevated_point_with_normal.x = elevated_point.x;
        elevated_point_with_normal.y = elevated_point.y;
        elevated_point_with_normal.z = elevated_point.z;

        // using that [a b c]^T is perpendicular to the plane in plane equation
        // ax + by + cz + d = 0
        elevated_point_with_normal.normal_x
            = plane_coefficients->values[0] / normalising_constant;
        elevated_point_with_normal.normal_y
            = plane_coefficients->values[1] / normalising_constant;
        elevated_point_with_normal.normal_z
            = plane_coefficients->values[2] / normalising_constant;
        point_index++;
    }
    return true;
}

bool SimpleParameterDeterminer::determineParameters(
    boost::shared_ptr<PlaneCoefficientsVector> const plane_coefficients_vector,
    boost::shared_ptr<HullVector> const hull_vector,
    boost::shared_ptr<PolygonVector> const polygon_vector,
    RealSenseCategory const realsense_category,
    boost::shared_ptr<GaitParameters> gait_parameters,
    std::string frame_id_to_transform_to, std::string subgait_name)
{
    ROS_DEBUG("Determining parameters with simple parameter determiner");
    hull_vector_ = hull_vector;
    realsense_category_.emplace(realsense_category);
    gait_parameters_ = gait_parameters;
    plane_coefficients_vector_ = plane_coefficients_vector;
    polygon_vector_ = polygon_vector;
    frame_id_to_transform_to_ = frame_id_to_transform_to;

    // Return a standard step parameter, which works for medium stairs and
    // medium ramp
    gait_parameters_->first_parameter = 0.5;
    gait_parameters_->second_parameter = 0.5;
    return true;
};
