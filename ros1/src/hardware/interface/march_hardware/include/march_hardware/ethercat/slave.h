/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
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
#ifndef MARCH_HARDWARE_SLAVE_H
#define MARCH_HARDWARE_SLAVE_H
#include "march_hardware/error/hardware_exception.h"
#include "pdo_interface.h"
#include "sdo_interface.h"

#include <memory>
#include <stdexcept>

#include <ros/ros.h>

namespace march {
class Slave : public PdoSlaveInterface {
public:
    Slave(uint16_t slave_index, const PdoInterfacePtr& pdo_interface,
        const SdoInterfacePtr& sdo_interface)
        : PdoSlaveInterface(slave_index, pdo_interface)
        , slave_index_(slave_index)
        , sdo_interface_(sdo_interface)
    {
        if (pdo_interface == nullptr || sdo_interface == nullptr) {
            throw std::invalid_argument(
                "SDO and PDO interface cannot be nullptr");
        }
        if (this->slave_index_ < 1) {
            throw error::HardwareException(
                error::ErrorType::INVALID_SLAVE_INDEX,
                "Slave index %d is smaller than 1", this->slave_index_);
        }
    };

    ~Slave() noexcept override = default;

    uint16_t getSlaveIndex() const
    {
        return this->slave_index_;
    }

    bool initSdo(int cycle_time)
    {
        SdoSlaveInterface sdo_slave_interface(
            this->slave_index_, this->sdo_interface_);
        return this->initSdo(sdo_slave_interface, cycle_time);
    }

    void reset()
    {
        SdoSlaveInterface sdo_slave_interface(
            this->slave_index_, this->sdo_interface_);
        this->reset(sdo_slave_interface);
    }

protected:
    virtual bool initSdo(SdoSlaveInterface& /* sdo */, int /* cycle_time */)
    {
        return false;
    }

    virtual void reset(SdoSlaveInterface& /* sdo */)
    {
    }

private:
    const uint16_t slave_index_;
    SdoInterfacePtr sdo_interface_;
};
} // namespace march

#endif // MARCH_HARDWARE_SLAVE_H
