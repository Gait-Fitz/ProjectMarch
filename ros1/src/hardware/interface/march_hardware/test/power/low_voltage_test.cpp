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
#include "../mocks/mock_pdo_interface.h"
#include "march_hardware/power/low_voltage.h"

#include <sstream>

#include <gtest/gtest.h>

class LowVoltageTest : public ::testing::Test {
protected:
    MockPdoInterfacePtr mock_pdo;
    march::PdoSlaveInterface pdo
        = march::PdoSlaveInterface(/*slave_index=*/1, this->mock_pdo);
};

TEST_F(LowVoltageTest, Equals)
{
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    march::LowVoltage lowVoltage1(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    march::LowVoltage lowVoltage2(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    EXPECT_TRUE(lowVoltage1 == lowVoltage2);
}

TEST_F(LowVoltageTest, NotEquals)
{
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    NetDriverOffsets netDriverOffsets2(/*lowVoltageNetOnOff=*/1,
        /*highVoltageNetOnOff=*/2, /*highVoltageNetEnableDisable=*/3);
    march::LowVoltage lowVoltage1(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    march::LowVoltage lowVoltage2(
        this->pdo, netMonitoringOffsets, netDriverOffsets2);

    EXPECT_FALSE(lowVoltage1 == lowVoltage2);
}

TEST_F(LowVoltageTest, Stream)
{
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    march::LowVoltage lowVoltage1(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    std::stringstream ss;
    ss << lowVoltage1;
    EXPECT_EQ("LowVoltage(netMonitoringOffsets: "
              "NetMonitorOffsets(powerDistributionBoardCurrent: -1, "
              "lowVoltageNet1Current: -1, lowVoltageNet2Current: -1, "
              "highVoltageNetCurrent: -1, lowVoltageState: -1, "
              "highVoltageOvercurrentTrigger: -1, highVoltageEnabled: -1, "
              "highVoltageState: -1), netDriverOffsets: "
              "NetDriverOffsets(lowVoltageNetOnOff: -1, highVoltageNetOnOff: "
              "-1, highVoltageNetEnableDisable: -1))",
        ss.str());
}
