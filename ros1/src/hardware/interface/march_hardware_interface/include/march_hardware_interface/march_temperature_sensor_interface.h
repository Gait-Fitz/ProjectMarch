/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2019 Isha Dijks, Olav de Haas
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

#ifndef MARCH_HARDWARE_INTERFACE_MARCH_TEMPERATURE_SENSOR_INTERFACE_H
#define MARCH_HARDWARE_INTERFACE_MARCH_TEMPERATURE_SENSOR_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

class MarchTemperatureSensorHandle {
public:
    MarchTemperatureSensorHandle(const std::string& name, ///< The name of joint
        const double*
            temperature, ///< A pointer of the temperature in degrees celsius.
        const double* variance ///< A pointer to the storage of the temperature
                               ///< covariance.
        )
        : name_(name)
        , temperature_(temperature)
        , variance(variance)
    {
    }

    std::string getName() const
    {
        return name_;
    }

    const double* getTemperature() const
    {
        return temperature_;
    }

    const double* getVariance() const
    {
        return variance;
    }

private:
    std::string name_;

    const double* temperature_;
    const double* variance;
};

class MarchTemperatureSensorInterface
    : public hardware_interface::HardwareResourceManager<
          MarchTemperatureSensorHandle> {
};

#endif // MARCH_HARDWARE_INTERFACE_MARCH_TEMPERATURE_SENSOR_INTERFACE_H
