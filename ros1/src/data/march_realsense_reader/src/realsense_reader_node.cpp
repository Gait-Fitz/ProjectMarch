/*
 * Copyright (C) 2021 Katja Schmahl, Thijs Raymakers, Thijs Veen,
 *                    Wolf Nederpel
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

#include <march_realsense_reader/pointcloud_parametersConfig.h>
#include <march_realsense_reader/realsense_reader.h>
#include <ros/ros.h>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "march_realsense_reader");
    ros::NodeHandle n;

    RealSenseReader reader = RealSenseReader(&n);

    dynamic_reconfigure::Server<
        march_realsense_reader::pointcloud_parametersConfig>
        dserver;
    dynamic_reconfigure::Server<
        march_realsense_reader::pointcloud_parametersConfig>::CallbackType f;

    f = std::bind(&RealSenseReader::readConfigCb, &reader,
        std::placeholders::_1, std::placeholders::_2);
    dserver.setCallback(f);

    ros::spin();
    ros::shutdown();
    return 0;
}
