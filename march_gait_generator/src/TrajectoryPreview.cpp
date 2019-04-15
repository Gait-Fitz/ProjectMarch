// Copyright 2019 Project March.

#include <march_gait_generator/TrajectoryPreview.h>

#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <kdl/trajectory.hpp>

TrajectoryPreview::TrajectoryPreview(Gait gait) {


    trajectory_msgs::JointTrajectory trajectory = gait.toJointTrajectory();

    ROS_INFO_STREAM(trajectory);
}
