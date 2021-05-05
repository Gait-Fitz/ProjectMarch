/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas, Rutger van Beek
 * Copyright (C) 2019 Tim Buckers
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
#include "march_hardware/encoder/encoder.h"

#include <gmock/gmock.h>

class MockEncoder : public march::Encoder {
public:
    explicit MockEncoder(size_t number_of_bits)
        : Encoder(number_of_bits)
    {
    }

    MOCK_CONST_METHOD0(getRadiansPerBit, double());
    MOCK_CONST_METHOD2(toRadians, double(double, bool));
    MOCK_CONST_METHOD2(toIU, double(double, bool));
};
