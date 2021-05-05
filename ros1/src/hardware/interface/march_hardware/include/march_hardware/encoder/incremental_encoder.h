/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
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

#ifndef MARCH_HARDWARE_INCREMENTAL_ENCODER_H
#define MARCH_HARDWARE_INCREMENTAL_ENCODER_H
#include "march_hardware/encoder/encoder.h"

#include <ostream>

namespace march {
class IncrementalEncoder : public Encoder {
public:
    IncrementalEncoder(size_t number_of_bits, double transmission);

    ~IncrementalEncoder() noexcept override = default;

    // Inherited methods
    double getRadiansPerBit() const final;
    double toRadians(double iu, bool /* use_zero_position */) const final;
    double toIU(double radians, bool /* use_zero_position */) const final;

    double getTransmission() const;

    /** @brief Override comparison operator */
    friend bool operator==(
        const IncrementalEncoder& lhs, const IncrementalEncoder& rhs)
    {
        return lhs.getTotalPositions() == rhs.getTotalPositions()
            && lhs.transmission_ == rhs.transmission_;
    }
    /** @brief Override stream operator for clean printing */
    friend std::ostream& operator<<(
        std::ostream& os, const IncrementalEncoder& encoder)
    {
        return os << "totalPositions: " << encoder.getTotalPositions() << ", "
                  << "transmission: " << encoder.transmission_;
    }

private:
    const double transmission_;
};
} // namespace march

#endif // MARCH_HARDWARE_INCREMENTAL_ENCODER_H
