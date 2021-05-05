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
#include "march_hardware/power/net_monitor_offsets.h"

#include <sstream>

#include <gtest/gtest.h>

class NetMonitorOffsetsTest : public ::testing::Test {
protected:
    const int powerDistributionBoardCurrent = 1;
    const int lowVoltageNet1Current = 2;
    const int lowVoltageNet2Current = 3;
    const int highVoltageNetCurrent = 4;
    const int lowVoltageState = 5;
    const int highVoltageOvercurrentTrigger = 6;
    const int highVoltageEnabled = 7;
    const int highVoltageState = 8;
};
TEST_F(NetMonitorOffsetsTest, ValidValues)
{
    NetMonitorOffsets netMonitorOffsets(powerDistributionBoardCurrent,
        lowVoltageNet1Current, lowVoltageNet2Current, highVoltageNetCurrent,
        lowVoltageState, highVoltageOvercurrentTrigger, highVoltageEnabled,
        highVoltageState);

    EXPECT_EQ(powerDistributionBoardCurrent,
        netMonitorOffsets.getPowerDistributionBoardCurrent());
    EXPECT_EQ(
        lowVoltageNet1Current, netMonitorOffsets.getLowVoltageNetCurrent(1));
    EXPECT_EQ(
        lowVoltageNet2Current, netMonitorOffsets.getLowVoltageNetCurrent(2));
    EXPECT_EQ(
        highVoltageNetCurrent, netMonitorOffsets.getHighVoltageNetCurrent());
    EXPECT_EQ(lowVoltageState, netMonitorOffsets.getLowVoltageState());
    EXPECT_EQ(highVoltageOvercurrentTrigger,
        netMonitorOffsets.getHighVoltageOvercurrentTrigger());
    EXPECT_EQ(highVoltageEnabled, netMonitorOffsets.getHighVoltageEnabled());
    EXPECT_EQ(highVoltageState, netMonitorOffsets.getHighVoltageState());
}
TEST_F(NetMonitorOffsetsTest, EmptyConstructor)
{
    NetMonitorOffsets netMonitorOffsets;

    EXPECT_THROW(netMonitorOffsets.getPowerDistributionBoardCurrent(),
        std::runtime_error);
    EXPECT_THROW(
        netMonitorOffsets.getLowVoltageNetCurrent(1), std::runtime_error);
    EXPECT_THROW(
        netMonitorOffsets.getLowVoltageNetCurrent(2), std::runtime_error);
    EXPECT_THROW(
        netMonitorOffsets.getHighVoltageNetCurrent(), std::runtime_error);
    EXPECT_THROW(netMonitorOffsets.getLowVoltageState(), std::runtime_error);
    EXPECT_THROW(netMonitorOffsets.getHighVoltageOvercurrentTrigger(),
        std::runtime_error);
    EXPECT_THROW(netMonitorOffsets.getHighVoltageEnabled(), std::runtime_error);
    EXPECT_THROW(netMonitorOffsets.getHighVoltageState(), std::runtime_error);
}

TEST_F(NetMonitorOffsetsTest, GetNotExistingNet)
{
    NetMonitorOffsets netMonitorOffsets(powerDistributionBoardCurrent,
        lowVoltageNet1Current, lowVoltageNet2Current, highVoltageNetCurrent,
        lowVoltageState, highVoltageOvercurrentTrigger, highVoltageEnabled,
        highVoltageState);
    EXPECT_THROW(
        netMonitorOffsets.getLowVoltageNetCurrent(42), std::runtime_error);
}

TEST_F(NetMonitorOffsetsTest, InValidValues)
{
    EXPECT_THROW(
        NetMonitorOffsets netMonitorOffsets(powerDistributionBoardCurrent,
            lowVoltageNet1Current, lowVoltageNet2Current, highVoltageNetCurrent,
            lowVoltageState, highVoltageOvercurrentTrigger, highVoltageEnabled,
            -1),
        std::runtime_error);
}

TEST_F(NetMonitorOffsetsTest, Equals)
{
    NetMonitorOffsets netMonitorOffsets1(powerDistributionBoardCurrent,
        lowVoltageNet1Current, lowVoltageNet2Current, highVoltageNetCurrent,
        lowVoltageState, highVoltageOvercurrentTrigger, highVoltageEnabled,
        highVoltageState);
    NetMonitorOffsets netMonitorOffsets2(powerDistributionBoardCurrent,
        lowVoltageNet1Current, lowVoltageNet2Current, highVoltageNetCurrent,
        lowVoltageState, highVoltageOvercurrentTrigger, highVoltageEnabled,
        highVoltageState);
    EXPECT_TRUE(netMonitorOffsets1 == netMonitorOffsets2);
}
TEST_F(NetMonitorOffsetsTest, NotEquals)
{
    NetMonitorOffsets netMonitorOffsets1(
        /*powerDistributionBoardCurrentByteOffset=*/24, lowVoltageNet1Current,
        lowVoltageNet2Current, highVoltageNetCurrent, lowVoltageState,
        highVoltageOvercurrentTrigger, highVoltageEnabled, highVoltageState);
    NetMonitorOffsets netMonitorOffsets2(powerDistributionBoardCurrent,
        lowVoltageNet1Current, lowVoltageNet2Current, highVoltageNetCurrent,
        /*lowVoltageStateByteOffset=*/42, highVoltageOvercurrentTrigger,
        highVoltageEnabled, highVoltageState);
    NetMonitorOffsets netMonitorOffsets3(powerDistributionBoardCurrent,
        lowVoltageNet1Current, lowVoltageNet2Current, highVoltageNetCurrent,
        /*lowVoltageStateByteOffset=*/42, highVoltageOvercurrentTrigger,
        highVoltageEnabled, /*highVoltageStateByteOffset=*/11);

    EXPECT_FALSE(netMonitorOffsets1 == netMonitorOffsets2);
    EXPECT_FALSE(netMonitorOffsets1 == netMonitorOffsets3);
    EXPECT_FALSE(netMonitorOffsets2 == netMonitorOffsets3);
}

TEST_F(NetMonitorOffsetsTest, TestStream)
{
    NetMonitorOffsets netMonitorOffsets(powerDistributionBoardCurrent,
        lowVoltageNet1Current, lowVoltageNet2Current, highVoltageNetCurrent,
        lowVoltageState, highVoltageOvercurrentTrigger, highVoltageEnabled,
        highVoltageState);
    std::stringstream ss;
    ss << "netMonitorOffsets: " << netMonitorOffsets;
    EXPECT_EQ(
        "netMonitorOffsets: NetMonitorOffsets(powerDistributionBoardCurrent: "
        "1, lowVoltageNet1Current: 2, "
        "lowVoltageNet2Current: 3, highVoltageNetCurrent: 4, lowVoltageState: "
        "5, highVoltageOvercurrentTrigger: 6, "
        "highVoltageEnabled: 7, highVoltageState: 8)",
        ss.str());
}
