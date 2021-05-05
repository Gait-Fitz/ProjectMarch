/*
 * Copyright (C) 2021 Thijs Raymakers
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
#ifndef MARCH_HARDWARE_INTERFACE_POWERNETTYPE_H
#define MARCH_HARDWARE_INTERFACE_POWERNETTYPE_H
#include <ostream>
#include <ros/assert.h>
#include <ros/package.h>
#include <string>

class PowerNetType {
public:
    enum Value : int { undefined = 0, high_voltage = 1, low_voltage = 2 };

    PowerNetType()
    {
        value = undefined;
    }

    explicit PowerNetType(const std::string& name)
    {
        if (name == "high_voltage") {
            this->value = high_voltage;
        } else if (name == "low_voltage") {
            this->value = low_voltage;
        } else {
            // NOLINTNEXTLINE(hicpp-no-assembler): violation in macro of ROS
            ROS_ASSERT_MSG(false, "Unknown power net type %s", name.c_str());
            this->value = undefined;
        }
    }

    bool operator==(PowerNetType a) const
    {
        return value == a.value;
    }
    bool operator!=(PowerNetType a) const
    {
        return value != a.value;
    }

    bool operator==(int a) const
    {
        return value == a;
    }
    bool operator!=(int a) const
    {
        return value != a;
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(
        std::ostream& os, const PowerNetType& powerNetType)
    {
        if (powerNetType.value == high_voltage) {
            return os << "PowerNetType(type:HighVoltage)";
        } else if (powerNetType.value == low_voltage) {
            return os << "PowerNetType(type:LowVoltage)";
        } else {
            return os << "PowerNetType(type:Undefined)";
        }
    }

private:
    Value value;
};

#endif // MARCH_HARDWARE_INTERFACE_POWERNETTYPE_H
