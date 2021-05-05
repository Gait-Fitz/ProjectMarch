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

#include <contact/contact_plugin.h>

namespace gazebo {
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
    // Get the parent sensor.
    parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!parentSensor) {
        gzerr << "ContactPlugin requires a ContactSensor.\n";
        return;
    }
    name = parentSensor->Name();

    // Create our ROS node.
    ros_node_ = std::make_unique<ros::NodeHandle>(name);

    // Create a named topic, and subscribe to it.
    std::ostringstream topic_name;
    topic_name << "/march/contact/" << name;
    ros_pub_ = ros_node_->advertise<std_msgs::Bool>(
        topic_name.str(), /*queue_size=*/10);

    // Connect to the sensor update event.
    updateConnection = parentSensor->ConnectUpdated(
        std::bind(&ContactPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    parentSensor->SetActive(/*_value=*/true);

    // Log the initialization
    std::ostringstream initialized;
    initialized << "Succesfully initialized contact plugin: " << name;
    ROS_INFO_STREAM(initialized.str());
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
    // Get all the contacts.
    msgs::Contacts contacts;
    contacts = parentSensor->Contacts();
    std_msgs::Bool msg;
    // Set the msg based on contacts, casting to right type
    msg.data = static_cast<uint8_t>(false);
    if (contacts.contact_size() > 0) {
        msg.data = static_cast<uint8_t>(true);
    }
    ros_pub_.publish(msg);
}
} // namespace gazebo