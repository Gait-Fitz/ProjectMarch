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

// Copyright 2019 Project March
#ifndef MARCH_HARDWARE_INTERFACE_POWERNETONOFFCOMMAND_H
#define MARCH_HARDWARE_INTERFACE_POWERNETONOFFCOMMAND_H

#include <march_hardware_interface/power_net_type.h>
#include <ostream>

class PowerNetOnOffCommand {
    PowerNetType type_;
    bool on_or_off_ {};
    int net_number_ {};

public:
    PowerNetOnOffCommand()
    {
        reset();
    }

    void reset()
    {
        type_ = PowerNetType();
        on_or_off_ = false;
        net_number_ = -1;
    }

    PowerNetOnOffCommand(
        const PowerNetType type, bool on_or_off, int net_number)
        : type_(type)
        , on_or_off_(on_or_off)
        , net_number_(net_number)
    {
    }

    const PowerNetType getType() const
    {
        return type_;
    }
    bool isOnOrOff() const
    {
        return on_or_off_;
    }
    int getNetNumber() const
    {
        return net_number_;
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(
        std::ostream& os, const PowerNetOnOffCommand& powerNetOnOffCommand)
    {
        return os << "PowerNetOnOffCommand(isOnOrOff: "
                  << powerNetOnOffCommand.isOnOrOff()
                  << ", netNumber:" << powerNetOnOffCommand.getNetNumber()
                  << ", type: " << powerNetOnOffCommand.getType() << ")";
    }
};

#endif // MARCH_HARDWARE_INTERFACE_POWERNETONOFFCOMMAND_H
