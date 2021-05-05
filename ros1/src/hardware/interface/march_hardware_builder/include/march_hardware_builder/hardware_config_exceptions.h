/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
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

// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
#define MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
#include <sstream>
#include <string>

#include <march_hardware/error/error_type.h>
#include <march_hardware/error/hardware_exception.h>

class MissingKeyException : public march::error::HardwareException {
public:
    MissingKeyException(
        const std::string& key_name, const std::string& object_name)
        : march::error::HardwareException(
            march::error::ErrorType::MISSING_REQUIRED_KEY,
            createDescription(key_name, object_name))
    {
    }

private:
    static std::string createDescription(
        const std::string& key_name, const std::string& object_name)
    {
        std::ostringstream ss;
        ss << "Missing required key '" << key_name
           << "' while creating object '" << object_name << "'";
        return ss.str();
    }
};
#endif // MARCH_HARDWARE_BUILDER_HARDWARE_CONFIG_EXCEPTIONS_H
