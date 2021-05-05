/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas, Roel Vos, Rutger van Beek
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
#include "march_hardware/encoder/incremental_encoder.h"

namespace march {
IncrementalEncoder::IncrementalEncoder(
    size_t number_of_bits, double transmission)
    : Encoder(number_of_bits)
    , transmission_(transmission)
{
}

double IncrementalEncoder::getRadiansPerBit() const
{
    return PI_2 / (getTotalPositions() * this->transmission_);
}

double IncrementalEncoder::toIU(
    double radians, bool /*use_zero_position*/) const
{
    return radians / getRadiansPerBit();
}

double IncrementalEncoder::toRadians(
    double iu, bool /*use_zero_position*/) const
{
    return iu * getRadiansPerBit();
}

double IncrementalEncoder::getTransmission() const
{
    return this->transmission_;
}
} //  namespace march
