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
#include "march_hardware/power/power_distribution_board.h"
#include "march_hardware/ethercat/pdo_types.h"

namespace march {
PowerDistributionBoard::PowerDistributionBoard(const Slave& slave,
    NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets,
    BootShutdownOffsets bootShutdownOffsets)
    : Slave(slave)
    , netMonitoringOffsets(netMonitoringOffsets)
    , netDriverOffsets(netDriverOffsets)
    , bootShutdownOffsets(bootShutdownOffsets)
    , highVoltage(*this, netMonitoringOffsets, netDriverOffsets)
    , lowVoltage(*this, netMonitoringOffsets, netDriverOffsets)
    , masterOnlineToggle(false)
{
}

float PowerDistributionBoard::getPowerDistributionBoardCurrent()
{
    bit32 current = this->read32(
        this->netMonitoringOffsets.getPowerDistributionBoardCurrent());
    return current.f;
}

void PowerDistributionBoard::setMasterOnline()
{
    bit8 isOkBit {};
    // By continuously flipping the master online toggle we let the pdb know we
    // are still connected.
    this->masterOnlineToggle = !this->masterOnlineToggle;
    isOkBit.ui = this->masterOnlineToggle;
    this->write8(this->bootShutdownOffsets.getMasterOkByteOffset(), isOkBit);
}

void PowerDistributionBoard::setMasterShutDownAllowed(bool isAllowed)
{
    bit8 isAllowedBit {};
    isAllowedBit.ui = isAllowed;
    this->write8(
        this->bootShutdownOffsets.getShutdownAllowedByteOffset(), isAllowedBit);
}

bool PowerDistributionBoard::getMasterShutdownRequested()
{
    bit8 masterShutdownRequestedBit
        = this->read8(this->bootShutdownOffsets.getShutdownByteOffset());
    return masterShutdownRequestedBit.ui;
}

HighVoltage PowerDistributionBoard::getHighVoltage()
{
    return highVoltage;
}

LowVoltage PowerDistributionBoard::getLowVoltage()
{
    return lowVoltage;
}

} // namespace march
