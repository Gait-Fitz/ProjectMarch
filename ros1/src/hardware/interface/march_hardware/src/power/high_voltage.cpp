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
#include "march_hardware/power/high_voltage.h"
#include "march_hardware/ethercat/pdo_interface.h"
#include "march_hardware/ethercat/pdo_types.h"

namespace march {
HighVoltage::HighVoltage(PdoSlaveInterface& pdo,
    NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets)
    : pdo_(pdo)
    , netMonitoringOffsets(netMonitoringOffsets)
    , netDriverOffsets(netDriverOffsets)
{
}

bool HighVoltage::isValidHighVoltageNetNumber(uint8_t netNumber)
{
    return netNumber >= MIN_NET_NUMBER && netNumber <= MAX_NET_NUMBER;
}

void HighVoltage::throwInvalidNetArgument(
    uint8_t netNumber, const char* caller_name)
{
    ROS_FATAL_THROTTLE(2,
        "Can't execute call at %s for net %d because there are only %d high "
        "voltage nets.",
        caller_name, netNumber, MAX_NET_NUMBER);

    throw std::invalid_argument("High voltage net number should be between "
        + std::to_string(MIN_NET_NUMBER) + " and "
        + std::to_string(MAX_NET_NUMBER) + ".");
}

float HighVoltage::getNetCurrent()
{
    bit32 current = this->pdo_.read32(
        this->netMonitoringOffsets.getHighVoltageNetCurrent());
    return current.f;
}

bool HighVoltage::getNetOperational(uint8_t netNumber)
{
    if (not isValidHighVoltageNetNumber(netNumber)) {
        throwInvalidNetArgument(netNumber);
    }

    bit8 operational
        = this->pdo_.read8(this->netMonitoringOffsets.getHighVoltageState());
    // The first bit of the 8 bits represents net 1 and so on till the last 8th
    // bit which represents net 8.
    return ((uint8_t)(operational.ui >> (netNumber - 1U)) & 1U);
}

bool HighVoltage::getOvercurrentTrigger(uint8_t netNumber)
{
    if (not isValidHighVoltageNetNumber(netNumber)) {
        throwInvalidNetArgument(netNumber);
    }
    bit8 overcurrent = this->pdo_.read8(
        this->netMonitoringOffsets.getHighVoltageOvercurrentTrigger());
    return ((uint8_t)(overcurrent.ui >> (netNumber - 1U)) & 1U);
}

bool HighVoltage::getHighVoltageEnabled()
{
    bit8 highVoltageEnabled
        = this->pdo_.read8(this->netMonitoringOffsets.getHighVoltageEnabled());
    return highVoltageEnabled.ui;
}

void HighVoltage::setNetOnOff(bool on, uint8_t netNumber)
{
    if (not isValidHighVoltageNetNumber(netNumber)) {
        throwInvalidNetArgument(netNumber);
    }

    if (on && getNetOperational(netNumber)) {
        ROS_WARN_THROTTLE(2, "High voltage net %d is already on", netNumber);
    }
    uint8_t currentStateHighVoltageNets = getNetsOperational();
    bit8 highVoltageNets {};
    highVoltageNets.ui = 1U << (netNumber - 1U);
    if (on) {
        // Force bit of the respective net to one.
        highVoltageNets.ui |= currentStateHighVoltageNets;
    } else {
        // Force bit of the respective net to zero.
        highVoltageNets.ui = ~highVoltageNets.ui;
        highVoltageNets.ui &= currentStateHighVoltageNets;
    }
    this->pdo_.write8(
        this->netDriverOffsets.getHighVoltageNetOnOff(), highVoltageNets);
}

void HighVoltage::enableDisableHighVoltage(bool enable)
{
    if (enable && getHighVoltageEnabled()) {
        ROS_ERROR_THROTTLE(2, "High voltage already enabled");
        throw std::runtime_error("High voltage already enabled");
    } else if (!enable && !getHighVoltageEnabled()) {
        ROS_ERROR_THROTTLE(2, "High voltage already disabled");
        throw std::runtime_error("High voltage already disabled");
    }
    if (enable) {
        ROS_DEBUG_THROTTLE(2, "Trying to enable high voltage from software");
    } else {
        ROS_DEBUG_THROTTLE(2, "Trying to disable high voltage from software");
    }

    bit8 isEnabled {};
    isEnabled.ui = enable;
    this->pdo_.write8(
        this->netDriverOffsets.getHighVoltageEnableDisable(), isEnabled);
}

uint8_t HighVoltage::getNetsOperational()
{
    bit8 operational
        = this->pdo_.read8(this->netMonitoringOffsets.getHighVoltageState());
    return operational.ui;
}

} // namespace march
