/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
 * Copyright (C) 2020 Joris Weeda, Olav de Haas
 * Copyright (C) 2019 Isha Dijks, Michael Treffers, Olav de Haas,
 *                    Roy Arriëns
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
#ifndef MARCH_HARDWARE_IMOTIONCUBE_PDOMAP_H
#define MARCH_HARDWARE_IMOTIONCUBE_PDOMAP_H
#include "march_hardware/ethercat/sdo_interface.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"

#include <array>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace march {
/** Store IMC data as a struct to prevent data overlap.*/
struct IMCObject {
    uint16_t address {}; // in IMC memory (see IMC manual)
    uint8_t sub_index {}; // sub index corresponding to PDO register (see IMC
                          // manual)
    uint8_t length {}; // bits (see IMC manual)
    uint32_t combined_address {}; // combine the address(hex), sub-index(hex)
                                  // and length(hex)

    IMCObject(uint16_t _address, uint8_t _sub_index, uint8_t _length)
        : address(_address)
        , sub_index(_sub_index)
        , length(_length)
    {
        uint32_t MSword = (address << 16U);
        uint32_t LSword = ((uint32_t)(sub_index << 8U)) | length;

        combined_address = (MSword | LSword);
    }

    IMCObject() = default;
    ;
};

/** The data direction to which the PDO is specified is restricted to master in
 * slave out and slave out master in.*/
enum class DataDirection {
    MISO,
    MOSI,
};

/** All the available IMC object names divided over the PDO maps. make sure to
 * also add it to IMCPDOmap constructor.*/
enum class IMCObjectName {
    StatusWord,
    ActualPosition,
    ActualVelocity,
    MotionErrorRegister,
    DetailedErrorRegister,
    SecondDetailedErrorRegister,
    DCLinkVoltage,
    DriveTemperature,
    ActualTorque,
    CurrentLimit,
    MotorPosition,
    MotorVelocity,
    ControlWord,
    TargetPosition,
    TargetTorque,
    QuickStopDeceleration,
    QuickStopOption,
    MotorVoltage
};

class IMCPDOmap {
public:
    /**
     * Initiate all the entered IMC objects to prepare the PDO.
     *
     * @param object_name enum of the object to be added.
     * @throws HardwareException when the object to be added is not defined or
     * the registers overflow.
     */
    void addObject(IMCObjectName object_name);

    std::unordered_map<IMCObjectName, uint8_t> map(
        SdoSlaveInterface& sdo, DataDirection direction);

    static std::unordered_map<IMCObjectName, IMCObject> all_objects;

private:
    /** Used to sort the objects in the all_objects according to data length.
     * @return list of pairs <IMCObjectName, IMCObjects> according from object
     * sizes */
    std::vector<std::pair<IMCObjectName, IMCObject>> sortPDOObjects();

    /** Configures the PDO in the IMC using the given base register address and
     * sync manager address.
     * @return map of the IMC PDO object name in combination with the
     * byte-offset in the PDO register */
    std::unordered_map<IMCObjectName, uint8_t> configurePDO(
        SdoSlaveInterface& sdo, int base_register, uint16_t base_sync_manager);

    std::unordered_map<IMCObjectName, IMCObject> PDO_objects;
    int total_used_bits = 0;

    const int bits_per_register = 64; // Maximum amount of bits that can be
                                      // constructed in one PDO message.
    const int nr_of_regs = 4; // Amount of registers available.
    const std::array<int, 3> object_sizes { 32, 16, 8 }; // Available sizes.
};
} // namespace march

#endif // MARCH_HARDWARE_IMOTIONCUBE_PDOMAP_H