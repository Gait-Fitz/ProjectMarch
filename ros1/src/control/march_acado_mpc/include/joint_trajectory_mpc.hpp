/*
 * Copyright (C) 2021 Bas Volkers, Maarten ten Voorde, Thijs Raymakers,
 *                    Thijs Veen
 * Copyright (C) 2020 Maarten ten Voorde
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * Version 3 as published by the Free Software Foundation WITH
 * additional terms published by Project MARCH per section 7 of
 * the GNU General Public License Version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License INCLUDING the additional terms for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * AND the additional terms along with this program. If not,
 * see <https://projectmarch.nl/s/LICENSE> and
 * <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>.
 */

#ifndef MARCH_HARDWARE_TRAJECTORY_MPC_H
#define MARCH_HARDWARE_TRAJECTORY_MPC_H

// ROS Control includes
#include "model_predictive_controller.hpp"
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <joint_trajectory_controller/trajectory_interface/quintic_spline_segment.h>

// Other includes
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <vector>

// Publishing and msgs
#include <march_shared_msgs/MpcMsg.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <cassert>
#include <ctime>
#include <memory>

// Create a State alias
namespace joint_trajectory_controller {
typedef trajectory_interface::QuinticSplineSegment<double> SegmentImpl;
typedef JointTrajectorySegment<SegmentImpl> Segment;
typedef typename Segment::State State;
} // namespace joint_trajectory_controller

// Create the HardwareInterfaceAdapter class that handles the initializing,
// starting, updating and stopping of the controller
template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface,
    joint_trajectory_controller::State> {
public:
    HardwareInterfaceAdapter()
        : joint_handles_ptr_(nullptr)
    {
    }

    /**
     * \brief Initialize the controller by establishing the pointer to the
     * joints
     */
    bool init(std::vector<hardware_interface::JointHandle>& joint_handles,
        ros::NodeHandle& nh);

    /**
     * \brief Starts the controller by checking if the joint handle pointer is
     * filled and sets the initial command to zero so that the joint doesn't
     * start moving without a desired trajectory
     */
    void starting(const ros::Time& /*time*/);

    /**
     * \brief Updates the commanded effort for each individual joint and
     * estimates the inertia on each joint and publishes that information on a
     * topic
     */
    void updateCommand(const ros::Time& /*time*/, const ros::Duration& period,
        const std::vector<
            joint_trajectory_controller::State>& /*desired_states*/,
        const joint_trajectory_controller::State& state_error);

    /**
     * \brief Procedure, if required, for stopping the controller
     */
    void stopping(const ros::Time& /*time*/);
    /**
     * \brief Start MPC topic and resize vectors in the MPC msg.
     */
    void initMpcMsg();
    /**
     * \brief Set and publish the message with MPC outputs, and MPC
     * configuration
     */
    void setMpcMsg(int joint_number);

private:
    /**
     * @brief Retrieve the weights from the parameter server for a joint.
     *        Weights are used for tuning the MPC
     * @param joint_name Joint to retrieve weights for
     * @return Returns a vector: The weight values belonging to the diagonal of
     * the MPC weight matrix.
     */
    std::vector<float> getWeights(const std::string& joint_name);

    std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;

    unsigned int num_joints_ {};
    double command {};

    std::vector<ModelPredictiveController> model_predictive_controllers_;
    vector<double> state;

    std::unique_ptr<
        realtime_tools::RealtimePublisher<march_shared_msgs::MpcMsg>>
        mpc_pub_;
};

// Assign an alias to the class definition
typedef HardwareInterfaceAdapter<hardware_interface::EffortJointInterface,
    joint_trajectory_controller::State>
    ModelPredictiveControllerInterface;

#endif // MARCH_HARDWARE_TRAJECTORY_MPC_H