/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
 * Copyright (C) 2019 Isha Dijks, Jitske de Vries
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

#ifndef MARCH_HARDWARE_ACTUATION_MODE_H
#define MARCH_HARDWARE_ACTUATION_MODE_H
#include <ros/console.h>
#include <string>

namespace march {
class ActuationMode {
public:
    enum Value : int {
        position,
        torque,
        unknown,
    };

    ActuationMode()
        : value_(unknown)
    {
    }

    // NOLINTNEXTLINE(hicpp-explicit-conversions)
    ActuationMode(Value value)
        : value_(value)
    {
    }

    // NOLINTNEXTLINE(hicpp-explicit-conversions)
    ActuationMode(const std::string& actuationMode)
    {
        if (actuationMode == "position") {
            this->value_ = position;
        } else if (actuationMode == "unknown") {
            this->value_ = unknown;
        } else if (actuationMode == "torque") {
            this->value_ = torque;
        } else {
            ROS_WARN("Actuation mode (%s) is not recognized, setting to "
                     "unknown mode",
                actuationMode.c_str());
            this->value_ = ActuationMode::unknown;
        }
    }

    uint8_t toModeNumber()
    {
        switch (this->value_) {
            case position:
                return 8;
            case torque:
                return 10;
            default:
                return 0;
        }
    }

    int getValue() const
    {
        return this->value_;
    }

    bool operator==(ActuationMode::Value a) const
    {
        return this->value_ == a;
    }

    bool operator!=(ActuationMode::Value a) const
    {
        return this->value_ != a;
    }

    std::string toString() const
    {
        switch (this->value_) {
            case position:
                return "position";
            case torque:
                return "torque";
            default:
                ROS_WARN("Actuationmode (%i) is neither 'torque' or 'position",
                    this->value_);
                return "unknown";
        }
    }

private:
    Value value_ = unknown;
};
} // namespace march

#endif // MARCH_HARDWARE_ACTUATION_MODE_H
