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

#pragma once
#include "march_hardware/motor_controller/motor_controller_state.h"
#include <gmock/gmock.h>
#include <optional>

class MockMotorControllerState : public march::MotorControllerState {
public:
    explicit MockMotorControllerState()
        : MotorControllerState()
    {
    }

    bool isOperational() override
    {
        return true;
    };

    bool hasError() override
    {
        return false;
    }

    std::optional<std::string> getErrorStatus() override
    {
        return "";
    };
    std::string getOperationalState() override
    {
        return "";
    };
};
