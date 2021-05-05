/*
 * Copyright (C) 2021 Bas Volkers
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
#include "march_hardware/ethercat/odrive_pdo_map.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/sdo_interface.h"

#include <cstdint>
#include <map>
#include <utility>

namespace march {
/* The offset and length values were retrieved from
 * https://confluence.projectmarch.nl:8443/display/62tech/GES+Implementation
 */
ODrivePDOmap::ObjectMap ODrivePDOmap::miso_objects_axis_0 = {
    { ODriveObjectName::ActualPosition,
        ODriveObject(/*offset=*/0, /*length=*/32) },
    { ODriveObjectName::ActualTorque,
        ODriveObject(/*offset=*/4, /*length=*/32) },
    { ODriveObjectName::ActualVelocity,
        ODriveObject(/*offset=*/8, /*length=*/32) },
    { ODriveObjectName::AxisError, ODriveObject(/*offset=*/12, /*length=*/32) },
    { ODriveObjectName::MotorError,
        ODriveObject(/*offset=*/16, /*length=*/16) },
    { ODriveObjectName::EncoderManagerError,
        ODriveObject(/*offset=*/20, /*length=*/32) },
    { ODriveObjectName::EncoderError,
        ODriveObject(/*offset=*/24, /*length=*/16) },
    { ODriveObjectName::ControllerError,
        ODriveObject(/*offset=*/28, /*length=*/8) },
};

ODrivePDOmap::ObjectMap ODrivePDOmap::miso_objects_axis_1 = {
    { ODriveObjectName::ActualPosition,
        ODriveObject(/*offset=*/32, /*length=*/32) },
    { ODriveObjectName::ActualTorque,
        ODriveObject(/*offset=*/36, /*length=*/32) },
    { ODriveObjectName::ActualVelocity,
        ODriveObject(/*offset=*/40, /*length=*/32) },
    { ODriveObjectName::AxisError, ODriveObject(/*offset=*/44, /*length=*/32) },
    { ODriveObjectName::MotorError,
        ODriveObject(/*offset=*/48, /*length=*/16) },
    { ODriveObjectName::EncoderManagerError,
        ODriveObject(/*offset=*/52, /*length=*/32) },
    { ODriveObjectName::EncoderError,
        ODriveObject(/*offset=*/56, /*length=*/16) },
    { ODriveObjectName::ControllerError,
        ODriveObject(/*offset=*/60, /*length=*/8) }
};

ODrivePDOmap::ObjectMap ODrivePDOmap::mosi_objects_axis_0 = {
    { ODriveObjectName::TargetTorque,
        ODriveObject(/*offset=*/0, /*length=*/32) },
};

ODrivePDOmap::ObjectMap ODrivePDOmap::mosi_objects_axis_1 = {
    { ODriveObjectName::TargetTorque,
        ODriveObject(/*offset=*/0, /*length=*/32) },
};

int8_t ODrivePDOmap::getMISOByteOffset(
    ODriveObjectName object_name, ODriveAxis axis)
{
    switch (axis) {
        case ODriveAxis::Zero:
            return miso_objects_axis_0.at(object_name).offset;
        case ODriveAxis::One:
            return miso_objects_axis_1.at(object_name).offset;
        default:
            throw error::HardwareException(
                error::ErrorType::ODRIVE_WRONG_AXIS_NUMBER,
                "Cannot get MISO Byte offset.");
    }
}

int8_t ODrivePDOmap::getMOSIByteOffset(
    ODriveObjectName object_name, ODriveAxis axis)
{
    switch (axis) {
        case ODriveAxis::Zero:
            return mosi_objects_axis_0.at(object_name).offset;
        case ODriveAxis::One:
            return mosi_objects_axis_1.at(object_name).offset;
        default:
            throw error::HardwareException(
                error::ErrorType::ODRIVE_WRONG_AXIS_NUMBER,
                "Cannot get MOSI Byte offset.");
    }
}

} // namespace march
