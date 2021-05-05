/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2019 Isha Dijks, Olav de Haas
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
#ifndef MARCH_TEMPERATURE_SENSOR_CONTROLLER_MARCH_TEMPERATURE_SENSOR_CONTROLLER_H
#define MARCH_TEMPERATURE_SENSOR_CONTROLLER_MARCH_TEMPERATURE_SENSOR_CONTROLLER_H
#include <vector>

#include <boost/shared_ptr.hpp>
#include <controller_interface/controller.h>
#include <march_hardware_interface/march_temperature_sensor_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/Temperature.h>

namespace march_temperature_sensor_controller {
// this controller gets access to the MarchTemperatureSensorInterface
class MarchTemperatureSensorController
    : public controller_interface::Controller<MarchTemperatureSensorInterface> {
public:
    MarchTemperatureSensorController() = default;

    bool init(MarchTemperatureSensorInterface* hw, ros::NodeHandle& root_nh,
        ros::NodeHandle& controller_nh) override;
    void starting(const ros::Time& time) override;
    void update(
        const ros::Time& time, const ros::Duration& /*period*/) override;
    void stopping(const ros::Time& /*time*/) override;

private:
    std::vector<MarchTemperatureSensorHandle> temperature_sensors_;
    typedef boost::shared_ptr<
        realtime_tools::RealtimePublisher<sensor_msgs::Temperature>>
        RtPublisherPtr;
    std::vector<RtPublisherPtr> realtime_pubs_;
    std::vector<ros::Time> last_publish_times_;
    double publish_rate_ {};
};
} // namespace march_temperature_sensor_controller

#endif // MARCH_TEMPERATURE_SENSOR_CONTROLLER_MARCH_TEMPERATURE_SENSOR_CONTROLLER_H
