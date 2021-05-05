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

#include "march_hardware/ethercat/sdo_interface.h"

#include <cstdint>

#include <ros/ros.h>
#include <soem/ethercat.h>

namespace march {
int SdoInterfaceImpl::write(
    uint16_t slave, uint16_t index, uint8_t sub, std::size_t size, void* value)
{
    ROS_DEBUG("sdo_write: slave %i, reg 0x%X, sub index %i", slave, index, sub);
    const int working_counter
        = ec_SDOwrite(slave, index, sub, FALSE, size, value, EC_TIMEOUTRXM);
    if (working_counter == 0) {
        ROS_FATAL("sdo_write: Error occurred when writing: slave %i, reg 0x%X, "
                  "sub index %i",
            slave, index, sub);
    }
    return working_counter;
}

int SdoInterfaceImpl::read(uint16_t slave, uint16_t index, uint8_t sub,
    int& val_size, void* value) const
{
    ROS_DEBUG("sdo_read: slave %i, reg 0x%X, sub index %i", slave, index, sub);
    const int working_counter
        = ec_SDOread(slave, index, sub, FALSE, &val_size, value, EC_TIMEOUTRXM);
    if (working_counter == 0) {
        ROS_FATAL("sdo_read: Error occurred when reading: slave %i, reg 0x%X, "
                  "sub index %i",
            slave, index, sub);
    }
    return working_counter;
}
} // namespace march
