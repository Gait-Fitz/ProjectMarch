/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
 * Copyright (C) 2019 Tim Buckers
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

// Copyright 2018 Project March.
#include "march_hardware/power/net_driver_offsets.h"

#include <sstream>

#include <gtest/gtest.h>

class NetDriverOffsetsTest : public ::testing::Test {
protected:
    const int lowVoltageNetOnOff = 1;
    const int highVoltageNetOnOff = 1;
    const int highVoltageNetEnableDisable = 1;
};

TEST_F(NetDriverOffsetsTest, ValidValues)
{
    NetDriverOffsets netDriverOffsets(
        lowVoltageNetOnOff, highVoltageNetOnOff, highVoltageNetEnableDisable);

    EXPECT_EQ(lowVoltageNetOnOff, netDriverOffsets.getLowVoltageNetOnOff());
    EXPECT_EQ(highVoltageNetOnOff, netDriverOffsets.getHighVoltageNetOnOff());
    EXPECT_EQ(highVoltageNetEnableDisable,
        netDriverOffsets.getHighVoltageEnableDisable());
}
TEST_F(NetDriverOffsetsTest, EmptyConstructor)
{
    NetDriverOffsets netDriverOffsets;
    EXPECT_THROW(netDriverOffsets.getLowVoltageNetOnOff(), std::runtime_error);
    EXPECT_THROW(netDriverOffsets.getHighVoltageNetOnOff(), std::runtime_error);
    EXPECT_THROW(
        netDriverOffsets.getHighVoltageEnableDisable(), std::runtime_error);
}

TEST_F(NetDriverOffsetsTest, InValidValues)
{
    EXPECT_THROW(NetDriverOffsets netDriverOffsets(
                     lowVoltageNetOnOff, -7, highVoltageNetEnableDisable),
        std::runtime_error);
    EXPECT_THROW(NetDriverOffsets netDriverOffsets(
                     -1, highVoltageNetOnOff, highVoltageNetEnableDisable),
        std::runtime_error);
    EXPECT_THROW(NetDriverOffsets netDriverOffsets(
                     lowVoltageNetOnOff, highVoltageNetOnOff, -12),
        std::runtime_error);
}

TEST_F(NetDriverOffsetsTest, Equals)
{
    NetDriverOffsets netDriverOffsets1(
        lowVoltageNetOnOff, highVoltageNetOnOff, highVoltageNetEnableDisable);
    NetDriverOffsets netDriverOffsets2(
        lowVoltageNetOnOff, highVoltageNetOnOff, highVoltageNetEnableDisable);

    EXPECT_TRUE(netDriverOffsets1 == netDriverOffsets2);
}
TEST_F(NetDriverOffsetsTest, NotEquals)
{
    NetDriverOffsets netDriverOffsets1(
        /*lowVoltageNetOnOff=*/11, highVoltageNetOnOff,
        highVoltageNetEnableDisable);
    NetDriverOffsets netDriverOffsets2(lowVoltageNetOnOff,
        /*highVoltageNetOnOff=*/22, highVoltageNetEnableDisable);
    NetDriverOffsets netDriverOffsets3(lowVoltageNetOnOff, highVoltageNetOnOff,
        /*highVoltageNetEnableDisable=*/33);

    EXPECT_FALSE(netDriverOffsets1 == netDriverOffsets2);
    EXPECT_FALSE(netDriverOffsets1 == netDriverOffsets3);
    EXPECT_FALSE(netDriverOffsets2 == netDriverOffsets3);
}

TEST_F(NetDriverOffsetsTest, TestStream)
{
    NetDriverOffsets netDriverOffsets(
        lowVoltageNetOnOff, highVoltageNetOnOff, highVoltageNetEnableDisable);
    std::stringstream ss;
    ss << netDriverOffsets;
    EXPECT_EQ("NetDriverOffsets(lowVoltageNetOnOff: 1, highVoltageNetOnOff: 1, "
              "highVoltageNetEnableDisable: 1)",
        ss.str());
}
