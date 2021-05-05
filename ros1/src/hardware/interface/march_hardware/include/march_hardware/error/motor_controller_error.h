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

// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_ERROR_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_ERROR_H
#include <array>
#include <bitset>
#include <string>

namespace march {
namespace error {
    enum class ErrorRegister {
        IMOTIONCUBE_MOTION_ERROR,
        IMOTIONCUBE_DETAILED_MOTION_ERROR,
        IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR,
        ODRIVE_AXIS_ERROR,
        ODRIVE_MOTOR_ERROR,
        ODRIVE_ENCODER_ERROR,
        ODRIVE_ENCODER_MANAGER_ERROR,
        ODRIVE_CONTROLLER_ERROR,
    };

    const size_t IMOTIONCUBE_MOTION_ERRORS_SIZE = 16;
    extern const std::array<std::string, IMOTIONCUBE_MOTION_ERRORS_SIZE>
        IMOTIONCUBE_MOTION_ERRORS;

    const size_t IMOTIONCUBE_DETAILED_MOTION_ERRORS_SIZE = 9;
    extern const std::array<std::string,
        IMOTIONCUBE_DETAILED_MOTION_ERRORS_SIZE>
        IMOTIONCUBE_DETAILED_MOTION_ERRORS;

    const size_t IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS_SIZE = 7;
    extern const std::array<std::string,
        IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS_SIZE>
        IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS;

    const size_t ODRIVE_AXIS_ERRORS_SIZE = 8;
    extern const std::array<std::string, ODRIVE_AXIS_ERRORS_SIZE>
        ODRIVE_AXIS_ERRORS;

    const size_t ODRIVE_MOTOR_ERRORS_SIZE = 26;
    extern const std::array<std::string, ODRIVE_MOTOR_ERRORS_SIZE>
        ODRIVE_MOTOR_ERRORS;

    const size_t ODRIVE_ENCODER_ERRORS_SIZE = 11;
    extern const std::array<std::string, ODRIVE_ENCODER_ERRORS_SIZE>
        ODRIVE_ENCODER_ERRORS;

    const size_t ODRIVE_ENCODER_MANAGER_ERRORS_SIZE = 1;
    extern const std::array<std::string, ODRIVE_ENCODER_MANAGER_ERRORS_SIZE>
        ODRIVE_ENCODER_MANAGER_ERRORS;

    const size_t ODRIVE_CONTROLLER_ERRORS_SIZE = 6;
    extern const std::array<std::string, ODRIVE_CONTROLLER_ERRORS_SIZE>
        ODRIVE_CONTROLLER_ERRORS;

    // Add an error type to the description
    void addErrorToDescription(
        size_t index, ErrorRegister error_register, std::string& description);

    template <typename T>
    std::string parseError(T error, ErrorRegister error_register)
    {
        std::string description;
        const auto size = sizeof(error) * 8;
        const std::bitset<size> bitset(error);

        for (size_t i = 0; i < size; i++) {
            if (bitset.test(i)) {
                addErrorToDescription(i, error_register, description);
            }
        }
        return description;
    }

} // namespace error
} // namespace march
#endif // MARCH_HARDWARE_MOTOR_CONTROLLER_ERROR_H
