/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
 * Copyright (C) 2019 Olav de Haas, Tim Buckers
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
#ifndef MARCH_HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H
#define MARCH_HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H

#include <string>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <march_hardware/power/power_distribution_board.h>
#include <march_hardware_interface/power_net_on_off_command.h>

class MarchPdbStateHandle {
public:
    MarchPdbStateHandle(
        /// Pointers to the storage location
        const std::string& name,
        march::PowerDistributionBoard* powerDistributionBoard,
        bool* master_shutdown_allowed_command,
        bool* all_high_voltage_on_off_command,
        PowerNetOnOffCommand* power_net_on_off_command)
        : name_(name)
        , master_shutdown_allowed_command_(master_shutdown_allowed_command)
        , all_high_voltage_on_off_command_(all_high_voltage_on_off_command)
        , power_net_on_off_command_(power_net_on_off_command)
        , powerDistributionBoard_(powerDistributionBoard)
    {
    }

    MarchPdbStateHandle() = default;

    std::string getName() const
    {
        return name_;
    }

    march::PowerDistributionBoard* getPowerDistributionBoard()
    {
        return const_cast<march::PowerDistributionBoard*>(
            powerDistributionBoard_);
    }

    void setMasterShutdownAllowed(bool is_allowed)
    {
        assert(master_shutdown_allowed_command_);
        ROS_INFO("setMasterShutdownAllowed %d", is_allowed);
        *master_shutdown_allowed_command_ = is_allowed;
    }

    void highVoltageNetEnableDisable(bool enable)
    {
        assert(all_high_voltage_on_off_command_);
        ROS_INFO("highVoltageNetEnableDisable %d", enable);
        *all_high_voltage_on_off_command_ = enable;
    }

    void turnNetOnOrOff(PowerNetType type, bool on_or_off, int net_number)
    {
        assert(power_net_on_off_command_);
        PowerNetOnOffCommand power_net_on_off_command(
            type, on_or_off, net_number);
        ROS_INFO_STREAM("turnNetOnOrOff: " << power_net_on_off_command);
        *power_net_on_off_command_ = power_net_on_off_command;
    }

    bool getHighVoltageEnabled()
    {
        return powerDistributionBoard_->getHighVoltage()
            .getHighVoltageEnabled();
    }

private:
    std::string name_;
    bool* master_shutdown_allowed_command_ {};
    bool* all_high_voltage_on_off_command_ {};
    PowerNetOnOffCommand* power_net_on_off_command_ {};
    march::PowerDistributionBoard* powerDistributionBoard_ {};
};

class MarchPdbStateInterface
    : public hardware_interface::HardwareResourceManager<MarchPdbStateHandle> {
};

#endif // MARCH_HARDWARE_INTERFACE_MARCH_PDB_STATE_INTERFACE_H
