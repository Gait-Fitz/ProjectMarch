/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
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

// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_ERROR_TYPE_H
#define MARCH_HARDWARE_ERROR_TYPE_H
#include <ostream>

namespace march {
namespace error {
    enum class ErrorType {
        INVALID_ACTUATION_MODE = 100,
        INVALID_ACTUATE_POSITION = 101,
        ENCODER_RESET = 102,
        OUTSIDE_HARD_LIMITS = 103,
        TARGET_EXCEEDS_MAX_DIFFERENCE = 104,
        TARGET_TORQUE_EXCEEDS_MAX_TORQUE = 105,
        PDO_OBJECT_NOT_DEFINED = 106,
        PDO_REGISTER_OVERFLOW = 107,
        WRITING_INITIAL_SETTINGS_FAILED = 108,
        NO_SOCKET_CONNECTION = 109,
        NOT_ALL_SLAVES_FOUND = 110,
        FAILED_TO_REACH_OPERATIONAL_STATE = 111,
        INVALID_ENCODER_RESOLUTION = 112,
        INVALID_RANGE_OF_MOTION = 114,
        INVALID_SLAVE_CONFIGURATION = 115,
        NOT_ALLOWED_TO_ACTUATE = 116,
        INVALID_SLAVE_INDEX = 117,
        MISSING_URDF_JOINT = 118,
        MISSING_REQUIRED_KEY = 119,
        INIT_URDF_FAILED = 120,
        INVALID_SW_STRING = 121,
        SLAVE_LOST_TIMOUT = 122,
        MISSING_ENCODER = 123,
        MISSING_MOTOR_CONTROLLER = 124,
        MISSING_TEMPERATURE_GES = 125,
        ODRIVE_WRONG_AXIS_NUMBER = 126,
        INVALID_MOTOR_CONTROLLER = 127,
        UNKNOWN = 999,
    };

    const char* getErrorDescription(ErrorType type);

    std::ostream& operator<<(std::ostream& s, ErrorType type);
} // namespace error
} // namespace march

#endif // MARCH_HARDWARE_ERROR_TYPE_H
