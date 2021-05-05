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
#ifndef MARCH_HARDWARE_LOW_VOLTAGE_H
#define MARCH_HARDWARE_LOW_VOLTAGE_H
#include "march_hardware/ethercat/pdo_interface.h"
#include "net_driver_offsets.h"
#include "net_monitor_offsets.h"

#include <cstdint>
#include <iostream>

namespace march {
class LowVoltage {
private:
    PdoSlaveInterface& pdo_;
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;

    uint8_t getNetsOperational();

    bool isValidLowVoltageNetNumber(uint8_t netNumber);
    void throwInvalidNetArgument(
        uint8_t netNumber, const char* caller_name = __builtin_FUNCTION());
    const uint8_t MIN_NET_NUMBER = 1;
    const uint8_t MAX_NET_NUMBER = 2;

public:
    LowVoltage(PdoSlaveInterface& pdo, NetMonitorOffsets netMonitoringOffsets,
        NetDriverOffsets netDriverOffsets);

    float getNetCurrent(uint8_t netNumber);
    bool getNetOperational(uint8_t netNumber);
    void setNetOnOff(bool on, uint8_t netNumber);

    /** @brief Override comparison operator */
    friend bool operator==(const LowVoltage& lhs, const LowVoltage& rhs)
    {
        return lhs.netDriverOffsets == rhs.netDriverOffsets
            && lhs.netMonitoringOffsets == rhs.netMonitoringOffsets;
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(
        std::ostream& os, const LowVoltage& lowVoltage)
    {
        return os << "LowVoltage(netMonitoringOffsets: "
                  << lowVoltage.netMonitoringOffsets << ", "
                  << "netDriverOffsets: " << lowVoltage.netDriverOffsets << ")";
    }
};

} // namespace march
#endif // MARCH_HARDWARE_LOW_VOLTAGE_H
