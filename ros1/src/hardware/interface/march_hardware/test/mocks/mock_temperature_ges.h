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

#pragma once
#include "mock_slave.h"

#include "march_hardware/temperature/temperature_ges.h"

#include <gmock/gmock.h>

class MockTemperatureGES : public march::TemperatureGES {
public:
    MockTemperatureGES()
        : TemperatureGES(MockSlave(), /*byte_offset=*/0)
    {
    }

    MOCK_CONST_METHOD0(getTemperature, float());

    MOCK_METHOD2(initSdo, bool(march::SdoSlaveInterface& sdo, int cycle_time));
};
