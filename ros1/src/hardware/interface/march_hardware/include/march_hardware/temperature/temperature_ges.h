/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
 * Copyright (C) 2019 Isha Dijks, Tim Buckers
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
#ifndef MARCH_HARDWARE_TEMPERATURE_GES_H
#define MARCH_HARDWARE_TEMPERATURE_GES_H
#include "march_hardware/ethercat/slave.h"
#include "temperature_sensor.h"

#include <cstddef>

namespace march {
class TemperatureGES : public Slave, TemperatureSensor {
public:
    TemperatureGES(const Slave& slave, uint8_t byte_offset);

    ~TemperatureGES() noexcept override = default;

    float getTemperature() const override;

    /** @brief Override comparison operator */
    friend bool operator==(const TemperatureGES& lhs, const TemperatureGES& rhs)
    {
        return lhs.getSlaveIndex() == rhs.getSlaveIndex()
            && lhs.temperature_byte_offset_ == rhs.temperature_byte_offset_;
    }
    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(
        std::ostream& os, const TemperatureGES& temperatureGes)
    {
        return os << "slaveIndex: " << temperatureGes.getSlaveIndex() << ", "
                  << "temperatureByteOffset: "
                  << temperatureGes.temperature_byte_offset_;
    }

private:
    const uint8_t temperature_byte_offset_;
};
} // namespace march
#endif // MARCH_HARDWARE_TEMPERATURE_GES_H
