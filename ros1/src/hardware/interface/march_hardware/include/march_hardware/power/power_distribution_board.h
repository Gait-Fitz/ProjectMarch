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

// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_POWER_DISTRIBUTION_BOARD_H
#define MARCH_HARDWARE_POWER_DISTRIBUTION_BOARD_H
#include "boot_shutdown_offsets.h"
#include "high_voltage.h"
#include "low_voltage.h"
#include "march_hardware/ethercat/slave.h"
#include "net_driver_offsets.h"
#include "net_monitor_offsets.h"

namespace march {
class PowerDistributionBoard : public Slave {
private:
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    BootShutdownOffsets bootShutdownOffsets;
    HighVoltage highVoltage;
    LowVoltage lowVoltage;
    bool masterOnlineToggle;

public:
    PowerDistributionBoard(const Slave& slave,
        NetMonitorOffsets netMonitoringOffsets,
        NetDriverOffsets netDriverOffsets,
        BootShutdownOffsets bootShutdownOffsets);

    float getPowerDistributionBoardCurrent();
    bool getMasterShutdownRequested();
    void setMasterOnline();
    void setMasterShutDownAllowed(bool isAllowed);

    HighVoltage getHighVoltage();
    LowVoltage getLowVoltage();

    /** @brief Override comparison operator */
    friend bool operator==(
        const PowerDistributionBoard& lhs, const PowerDistributionBoard& rhs)
    {
        return lhs.getSlaveIndex() == rhs.getSlaveIndex()
            && lhs.netMonitoringOffsets == rhs.netMonitoringOffsets
            && lhs.netDriverOffsets == rhs.netDriverOffsets
            && lhs.lowVoltage == rhs.lowVoltage
            && lhs.highVoltage == rhs.highVoltage;
    }
    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(
        std::ostream& os, const PowerDistributionBoard& powerDistributionBoard)
    {
        return os << "PowerDistributionBoard(slaveIndex: "
                  << powerDistributionBoard.getSlaveIndex() << ", "
                  << "masterOnlineToggle: "
                  << powerDistributionBoard.masterOnlineToggle << ", "
                  << powerDistributionBoard.highVoltage << ", "
                  << powerDistributionBoard.lowVoltage << ")";
    }
};
} // namespace march
#endif // MARCH_HARDWARE_POWER_DISTRIBUTION_BOARD_H
