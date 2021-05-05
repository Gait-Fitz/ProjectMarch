/*
 * Copyright (C) 2021 Katja Schmahl, Thijs Raymakers
 * Copyright (C) 2020 Roel Vos
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

// Copyright 2019 Project March.

#include "march_shared_msgs/ChangeComLevel.h"
#include "march_shared_msgs/GetPossibleComLevels.h"
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <march_gazebo_plugins/walk_controller.h>
#include <march_shared_msgs/CurrentGait.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#ifndef MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H
#define MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H

namespace gazebo {
class ComControllerPlugin : public ModelPlugin {
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override;
    void onRosMsg(const march_shared_msgs::CurrentGaitConstPtr& msg);
    bool onChangeComLevel(march_shared_msgs::ChangeComLevel::Request& req,
        march_shared_msgs::ChangeComLevel::Response& res);
    bool onGetPossibleComLevels(
        march_shared_msgs::GetPossibleComLevels::Request& req,
        march_shared_msgs::GetPossibleComLevels::Response& res);

    // Called by the world update start event
    void onUpdate();

private:
    void queueThread();

    physics::ModelPtr model_;

    std::unique_ptr<ObstacleController> controller_;

    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;

    /// \brief A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> ros_node_;

    /// \brief A ROS subscriber
    ros::Subscriber ros_sub_;

    /// \brief A ROS callbackqueue that helps process messages
    ros::CallbackQueue ros_queue_;

    /// \brief A thread the keeps running the ros_queue
    std::thread ros_queue_thread_;

    /// \brief A service used for sharing the available com_levels
    ros::ServiceServer get_possible_com_levels_service_;
    ros::ServiceServer change_com_level_service_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ComControllerPlugin)
} // namespace gazebo

#endif // MARCH_GAZEBO_PLUGINS_COM_CONTROLLER_PLUGIN_H
