/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Katja Schmahl
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

#ifndef CONTACT_CONTACT_PLUGIN_H
#define CONTACT_CONTACT_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>

namespace gazebo {
/// \brief An example plugin for a contact sensor.
class ContactPlugin : public SensorPlugin {

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
public:
    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

    /// \brief Callback that receives the contact sensor's update signal.
private:
    void OnUpdate();

    /// \brief Pointer to the contact sensor
    sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    event::ConnectionPtr updateConnection;

    /// \brief The name of the contact sensor
    std::string name;
    /// \brief A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> ros_node_;

    /// \brief A ROS subscriber
    ros::Publisher ros_pub_;
};
} // namespace gazebo
#endif