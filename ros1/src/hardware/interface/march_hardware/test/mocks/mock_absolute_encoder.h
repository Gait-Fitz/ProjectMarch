/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas, Rutger van Beek
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
#include "march_hardware/encoder/absolute_encoder.h"

#include <gmock/gmock.h>

class MockAbsoluteEncoder : public march::AbsoluteEncoder {
public:
    MockAbsoluteEncoder()
        : AbsoluteEncoder(/*number_of_bits=*/10, /*lower_limit_iu=*/0,
            /*upper_limit_iu=*/162, /*lower_limit_rad=*/0,
            /*upper_limit_rad=*/1, /*lower_soft_limit_rad=*/
            0.1, /*upper_soft_limit_rad=*/0.9)
    {
    }
};
