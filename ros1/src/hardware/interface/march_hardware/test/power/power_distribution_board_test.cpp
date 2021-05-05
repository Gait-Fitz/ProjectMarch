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
#include "../mocks/mock_slave.h"
#include "march_hardware/power/power_distribution_board.h"

#include <sstream>

#include <gtest/gtest.h>

class PowerDistributionBoardTest : public ::testing::Test {
protected:
    MockPdoInterfacePtr mock_pdo = std::make_shared<MockPdoInterface>();
    MockSdoInterfacePtr mock_sdo = std::make_shared<MockSdoInterface>();
    MockSlave mock_slave = MockSlave(this->mock_pdo, this->mock_sdo);
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    BootShutdownOffsets bootShutdownOffsets;
    int slaveIndex = 2;
};

TEST_F(PowerDistributionBoardTest, Unequals)
{
    NetDriverOffsets netDriverOffsets2(/*lowVoltageNetOnOff=*/1,
        /*highVoltageNetOnOff=*/2, /*highVoltageNetEnableDisable=*/3);
    NetMonitorOffsets netMonitorOffsets2(
        /*powerDistributionBoardCurrentByteOffset=*/1,
        /*lowVoltageNet1CurrentByteOffset=*/1,
        /*lowVoltageNet2CurrentByteOffset=*/1,
        /*highVoltageNetCurrentByteOffset=*/1, /*lowVoltageStateByteOffset=*/1,
        /*highVoltageOvercurrentTriggerByteOffset=*/1, /*highVoltageEnabled=*/1,
        /*highVoltageStateByteOffset=*/1);
    march::PowerDistributionBoard powerDistributionBoard1(this->mock_slave,
        netMonitoringOffsets, netDriverOffsets, bootShutdownOffsets);
    march::PowerDistributionBoard powerDistributionBoard2(this->mock_slave,
        netMonitoringOffsets, netDriverOffsets2, bootShutdownOffsets);
    march::PowerDistributionBoard powerDistributionBoard3(this->mock_slave,
        netMonitorOffsets2, netDriverOffsets, bootShutdownOffsets);

    EXPECT_FALSE(powerDistributionBoard1 == powerDistributionBoard2);
    EXPECT_FALSE(powerDistributionBoard1 == powerDistributionBoard3);
    EXPECT_FALSE(powerDistributionBoard2 == powerDistributionBoard3);
}

TEST_F(PowerDistributionBoardTest, Equals)
{
    march::PowerDistributionBoard powerDistributionBoard1(this->mock_slave,
        netMonitoringOffsets, netDriverOffsets, bootShutdownOffsets);
    march::PowerDistributionBoard powerDistributionBoard2(this->mock_slave,
        netMonitoringOffsets, netDriverOffsets, bootShutdownOffsets);
    EXPECT_TRUE(powerDistributionBoard1 == powerDistributionBoard2);
}
