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
#include "march_hardware_interface/march_pdb_state_interface.h"

#include <memory>

#include <gtest/gtest.h>

class PdbStateInterfaceTest : public ::testing::Test {
protected:
    std::unique_ptr<march::PowerDistributionBoard>
        power_distribution_board_read_;
    bool master_shutdown_allowed_command = false;
    bool all_high_voltage_on_off_command = true;
    PowerNetOnOffCommand power_net_on_off_command_;

    void SetUp() override
    {
        power_distribution_board_read_
            = std::make_unique<march::PowerDistributionBoard>(
                march::Slave(/*slave_index=*/1,
                    march::PdoInterfaceImpl::create(),
                    march::SdoInterfaceImpl::create()),
                NetMonitorOffsets(), NetDriverOffsets(), BootShutdownOffsets());
        master_shutdown_allowed_command = false;
        all_high_voltage_on_off_command = true;
        power_net_on_off_command_ = PowerNetOnOffCommand();
    }
};

TEST_F(PdbStateInterfaceTest, GeName)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);
    EXPECT_EQ("PDBhandle", marchPdbStateHandle.getName());
}

TEST_F(PdbStateInterfaceTest, GetPowerDistributionBoardEquals)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);
    EXPECT_TRUE(*power_distribution_board_read_
        == *marchPdbStateHandle.getPowerDistributionBoard());
}

TEST_F(PdbStateInterfaceTest, GetPowerDistributionBoardNotEqual)
{
    NetMonitorOffsets currentOffsets = NetMonitorOffsets(
        /*powerDistributionBoardCurrentByteOffset=*/5,
        /*lowVoltageNet1CurrentByteOffset=*/9,
        /*lowVoltageNet2CurrentByteOffset=*/13,
        /*highVoltageNetCurrentByteOffset=*/17, /*lowVoltageStateByteOffset=*/3,
        /*highVoltageOvercurrentTriggerByteOffset=*/2, /*highVoltageEnabled=*/1,
        /*highVoltageStateByteOffset=*/4);
    NetDriverOffsets netDriverOffsets
        = NetDriverOffsets(/*lowVoltageNetOnOff=*/4, /*highVoltageNetOnOff=*/3,
            /*highVoltageNetEnableDisable=*/2);
    BootShutdownOffsets stateOffsets
        = BootShutdownOffsets(/*masterOkByteOffset=*/0,
            /*shutdownByteOffset=*/0, /*shutdownAllowedByteOffset=*/1);
    march::PowerDistributionBoard pdb = march::PowerDistributionBoard(
        march::Slave(/*slave_index=*/1, march::PdoInterfaceImpl::create(),
            march::SdoInterfaceImpl::create()),
        currentOffsets, netDriverOffsets, stateOffsets);
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);
    EXPECT_FALSE(pdb == *marchPdbStateHandle.getPowerDistributionBoard());
}

TEST_F(PdbStateInterfaceTest, SetMasterShutdownAllowedTrue)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    EXPECT_FALSE(master_shutdown_allowed_command);
    marchPdbStateHandle.setMasterShutdownAllowed(/*is_allowed=*/true);
    EXPECT_TRUE(master_shutdown_allowed_command);
}
TEST_F(PdbStateInterfaceTest, SetMasterShutdownAllowedFalse)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    EXPECT_FALSE(master_shutdown_allowed_command);
    marchPdbStateHandle.setMasterShutdownAllowed(/*is_allowed=*/false);
    EXPECT_FALSE(master_shutdown_allowed_command);
}

TEST_F(PdbStateInterfaceTest, HighVoltageNetEnableDisable)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    EXPECT_TRUE(all_high_voltage_on_off_command);
    marchPdbStateHandle.setMasterShutdownAllowed(/*is_allowed=*/true);
    EXPECT_TRUE(all_high_voltage_on_off_command);
}
TEST_F(PdbStateInterfaceTest, TurnLowNetOn)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    // Check if the power net type is undefined
    EXPECT_EQ(PowerNetType(), power_net_on_off_command_.getType());
    marchPdbStateHandle.turnNetOnOrOff(
        PowerNetType("low_voltage"), /*on_or_off=*/true, /*net_number=*/1);
    EXPECT_EQ(PowerNetType("low_voltage"), power_net_on_off_command_.getType());
    EXPECT_EQ(1, power_net_on_off_command_.getNetNumber());
    EXPECT_TRUE(power_net_on_off_command_.isOnOrOff());
}

TEST_F(PdbStateInterfaceTest, TurnLowNetOff)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    // Check if the power net type is undefined
    EXPECT_EQ(PowerNetType(), power_net_on_off_command_.getType());
    marchPdbStateHandle.turnNetOnOrOff(
        PowerNetType("low_voltage"), /*on_or_off=*/false, /*net_number=*/1);
    EXPECT_EQ(PowerNetType("low_voltage"), power_net_on_off_command_.getType());
    EXPECT_EQ(1, power_net_on_off_command_.getNetNumber());
    EXPECT_FALSE(power_net_on_off_command_.isOnOrOff());
}
TEST_F(PdbStateInterfaceTest, TurnHighNetOn)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    // Check if the power net type is undefined
    EXPECT_EQ(PowerNetType(), power_net_on_off_command_.getType());
    marchPdbStateHandle.turnNetOnOrOff(
        PowerNetType("high_voltage"), /*on_or_off=*/true, /*net_number=*/1);
    EXPECT_EQ(
        PowerNetType("high_voltage"), power_net_on_off_command_.getType());
    EXPECT_EQ(1, power_net_on_off_command_.getNetNumber());
    EXPECT_TRUE(power_net_on_off_command_.isOnOrOff());
}

TEST_F(PdbStateInterfaceTest, TurnHighNetOff)
{
    MarchPdbStateHandle marchPdbStateHandle("PDBhandle",
        power_distribution_board_read_.get(), &master_shutdown_allowed_command,
        &all_high_voltage_on_off_command, &power_net_on_off_command_);

    // Check if the power net type is undefined
    EXPECT_EQ(PowerNetType(), power_net_on_off_command_.getType());
    marchPdbStateHandle.turnNetOnOrOff(
        PowerNetType("high_voltage"), /*on_or_off=*/false, /*net_number=*/1);
    EXPECT_EQ(
        PowerNetType("high_voltage"), power_net_on_off_command_.getType());
    EXPECT_EQ(1, power_net_on_off_command_.getNetNumber());
    EXPECT_FALSE(power_net_on_off_command_.isOnOrOff());
}
