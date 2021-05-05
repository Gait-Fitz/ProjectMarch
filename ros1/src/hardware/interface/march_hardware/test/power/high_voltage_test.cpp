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
#include "march_hardware/power/high_voltage.h"

#include <gtest/gtest.h>
#include <sstream>

class HighVoltageTest : public ::testing::Test {
protected:
    MockPdoInterfacePtr mock_pdo;
    march::PdoSlaveInterface pdo
        = march::PdoSlaveInterface(/*slave_index=*/1, this->mock_pdo);
};

TEST_F(HighVoltageTest, Equals)
{
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    march::HighVoltage highVoltage1(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    march::HighVoltage highVoltage2(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    EXPECT_TRUE(highVoltage1 == highVoltage2);
}

TEST_F(HighVoltageTest, UnEqual)
{
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets1;
    NetDriverOffsets netDriverOffsets2(/*lowVoltageNetOnOff=*/1,
        /*highVoltageNetOnOff=*/2, /*highVoltageNetEnableDisable=*/3);
    march::HighVoltage highVoltage1(
        this->pdo, netMonitoringOffsets, netDriverOffsets1);
    march::HighVoltage highVoltage2(
        this->pdo, netMonitoringOffsets, netDriverOffsets2);
    EXPECT_FALSE(highVoltage1 == highVoltage2);
}

TEST_F(HighVoltageTest, Stream)
{
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    march::HighVoltage highVoltage(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    std::stringstream ss;
    ss << highVoltage;
    EXPECT_EQ("HighVoltage(netMonitoringOffsets: "
              "NetMonitorOffsets(powerDistributionBoardCurrent: -1, "
              "lowVoltageNet1Current: -1, lowVoltageNet2Current: -1, "
              "highVoltageNetCurrent: -1, lowVoltageState: -1, "
              "highVoltageOvercurrentTrigger: -1, highVoltageEnabled: -1, "
              "highVoltageState: -1), netDriverOffsets: "
              "NetDriverOffsets(lowVoltageNetOnOff: -1, highVoltageNetOnOff: "
              "-1, highVoltageNetEnableDisable: -1))",
        ss.str());
}
