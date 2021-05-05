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

#ifndef MARCH_HARDWARE_ABSOLUTE_ENCODER_H
#define MARCH_HARDWARE_ABSOLUTE_ENCODER_H
#include "march_hardware/encoder/encoder.h"

#include <ostream>

namespace march {
class AbsoluteEncoder : public Encoder {
public:
    AbsoluteEncoder(size_t number_of_bits, int32_t lower_limit_iu,
        int32_t upper_limit_iu, double lower_limit_rad, double upper_limit_rad,
        double lower_soft_limit_rad, double upper_soft_limit_rad);

    ~AbsoluteEncoder() noexcept override = default;

    // Inherited methods
    double getRadiansPerBit() const final;
    double toRadians(double iu, bool use_zero_position) const final;
    double toIU(double radians, bool use_zero_position) const final;

    bool isWithinHardLimitsIU(int32_t iu) const;
    bool isWithinSoftLimitsIU(int32_t iu) const;
    bool isValidTargetIU(int32_t current_iu, int32_t target_iu) const;

    int32_t getUpperSoftLimitIU() const;
    int32_t getLowerSoftLimitIU() const;
    int32_t getUpperHardLimitIU() const;
    int32_t getLowerHardLimitIU() const;

    /** @brief Override comparison operator */
    friend bool operator==(
        const AbsoluteEncoder& lhs, const AbsoluteEncoder& rhs)
    {
        return lhs.getTotalPositions() == rhs.getTotalPositions()
            && lhs.upper_soft_limit_iu_ == rhs.upper_soft_limit_iu_
            && lhs.lower_soft_limit_iu_ == rhs.lower_soft_limit_iu_
            && lhs.upper_limit_iu_ == rhs.upper_limit_iu_
            && lhs.lower_limit_iu_ == rhs.lower_limit_iu_
            && lhs.zero_position_iu_ == rhs.zero_position_iu_;
    }
    /** @brief Override stream operator for clean printing */
    friend std::ostream& operator<<(
        std::ostream& os, const AbsoluteEncoder& encoder)
    {
        return os << "totalPositions: " << encoder.getTotalPositions() << ", "
                  << "upperHardLimit: " << encoder.upper_limit_iu_ << ", "
                  << "lowerHardLimit: " << encoder.lower_limit_iu_ << ", "
                  << "upperSoftLimit: " << encoder.upper_soft_limit_iu_ << ", "
                  << "lowerSoftLimit: " << encoder.lower_soft_limit_iu_ << ", "
                  << "zeroPositionIU: " << encoder.zero_position_iu_;
    }

    static constexpr double MAX_RANGE_DIFFERENCE = 0.05;

private:
    int32_t zero_position_iu_ = 0;
    int32_t lower_limit_iu_ = 0;
    int32_t upper_limit_iu_ = 0;
    int32_t lower_soft_limit_iu_ = 0;
    int32_t upper_soft_limit_iu_ = 0;
};
} // namespace march

#endif // MARCH_HARDWARE_ABSOLUTE_ENCODER_H
