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

// Copyright 2019 Project March.
#include "march_hardware/encoder/encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/pdo_types.h"

namespace march {
Encoder::Encoder(size_t number_of_bits)
    : total_positions_(Encoder::calculateTotalPositions(number_of_bits))
{
}

size_t Encoder::getTotalPositions() const
{
    return this->total_positions_;
}

size_t Encoder::calculateTotalPositions(size_t number_of_bits)
{
    if (number_of_bits < Encoder::MIN_RESOLUTION
        || number_of_bits > Encoder::MAX_RESOLUTION) {
        throw error::HardwareException(
            error::ErrorType::INVALID_ENCODER_RESOLUTION,
            "Encoder resolution of %d is not within range [%ld, %ld]",
            number_of_bits, Encoder::MIN_RESOLUTION, Encoder::MAX_RESOLUTION);
    }
    return (size_t)1 << number_of_bits;
}

} // namespace march
