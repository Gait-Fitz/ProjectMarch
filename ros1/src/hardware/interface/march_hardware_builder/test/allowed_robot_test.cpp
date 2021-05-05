/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Joris Weeda, Olav de Haas
 * Copyright (C) 2019 Isha Dijks, Olav de Haas, Sophie Bekker
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
#include "march_hardware_builder/hardware_builder.h"

#include <vector>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

TEST(AllowedRobotTest, TestMarch4Creation)
{
    urdf::Model urdf;
    urdf.initFile(ros::package::getPath("march_description")
                      .append(/*__s=*/"/urdf/march4.urdf"));
    ASSERT_NO_THROW(
        HardwareBuilder(AllowedRobot::march4, urdf).createMarchRobot());
}

// Fails because the March 3 does not have safety limits
// TEST(AllowedRobotTest, TestMarch3Creation)
//{
//  urdf::Model urdf;
//  urdf.initFile(ros::package::getPath("march_description").append("/urdf/march3.urdf"));
//  ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::march3,
//  urdf).createMarchRobot());
//}

// Fails because the rotational test joint is not correctly calibrated
// TEST(AllowedRobotTest, TestTestRotationalSetupCreation)
//{
//  urdf::Model urdf;
//  urdf.initFile(ros::package::getPath("march_description").append("/urdf/test_joint_rotational.urdf"));
//  ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::test_joint_rotational,
//  urdf).createMarchRobot());
//}

TEST(AllowedRobotTest, TestTestLinearSetupCreation)
{
    urdf::Model urdf;
    urdf.initFile(ros::package::getPath("march_description")
                      .append(/*__s=*/"/urdf/test_joint_linear.urdf"));
    ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::test_joint_linear, urdf)
                        .createMarchRobot());
}

TEST(AllowedRobotTest, TestTestRotationalSetupCreation)
{
    urdf::Model urdf;
    urdf.initFile(ros::package::getPath("march_description")
                      .append(/*__s=*/"/urdf/test_joint_rotational.urdf"));
    ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::test_joint_rotational, urdf)
                        .createMarchRobot());
}
