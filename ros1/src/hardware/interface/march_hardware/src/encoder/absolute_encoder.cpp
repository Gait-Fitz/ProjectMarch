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
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/error/hardware_exception.h"

#include <ros/ros.h>

namespace march {
AbsoluteEncoder::AbsoluteEncoder(size_t number_of_bits, int32_t lower_limit_iu,
    int32_t upper_limit_iu, double lower_limit_rad, double upper_limit_rad,
    double lower_soft_limit_rad, double upper_soft_limit_rad)
    : Encoder(number_of_bits)
    , lower_limit_iu_(lower_limit_iu)
    , upper_limit_iu_(upper_limit_iu)
{
    this->zero_position_iu_ = this->lower_limit_iu_
        - lower_limit_rad * this->getTotalPositions() / PI_2;
    this->lower_soft_limit_iu_
        = this->toIU(lower_soft_limit_rad, /*use_zero_position=*/true);
    this->upper_soft_limit_iu_
        = this->toIU(upper_soft_limit_rad, /*use_zero_position=*/true);

    if (this->lower_limit_iu_ >= this->upper_limit_iu_
        || this->lower_soft_limit_iu_ >= this->upper_soft_limit_iu_
        || this->lower_soft_limit_iu_ < this->lower_limit_iu_
        || this->upper_soft_limit_iu_ > this->upper_limit_iu_) {
        throw error::HardwareException(
            error::ErrorType::INVALID_RANGE_OF_MOTION,
            "lower_soft_limit: %d IU, upper_soft_limit: %d IU\n"
            "lower_hard_limit: %d IU, upper_hard_limit: %d IU",
            this->lower_soft_limit_iu_, this->upper_soft_limit_iu_,
            this->lower_limit_iu_, this->upper_limit_iu_);
    }

    const double range_of_motion = upper_limit_rad - lower_limit_rad;
    const double encoder_range_of_motion
        = this->toRadians(this->upper_limit_iu_, /*use_zero_position=*/true)
        - this->toRadians(this->lower_limit_iu_, /*use_zero_position=*/true);
    const double difference
        = std::abs(encoder_range_of_motion - range_of_motion)
        / encoder_range_of_motion;
    if (difference > AbsoluteEncoder::MAX_RANGE_DIFFERENCE) {
        ROS_WARN("Difference in range of motion of %.2f%% exceeds %.2f%%\n"
                 "Absolute encoder range of motion = %f rad\n"
                 "Limits range of motion = %f rad",
            difference * 100, AbsoluteEncoder::MAX_RANGE_DIFFERENCE * 100,
            encoder_range_of_motion, range_of_motion);
    }
}

double AbsoluteEncoder::getRadiansPerBit() const
{
    return PI_2 / getTotalPositions();
}

double AbsoluteEncoder::toRadians(double iu, bool use_zero_position) const
{
    if (use_zero_position) {
        return (iu - zero_position_iu_) * getRadiansPerBit();
    } else {
        return iu * getRadiansPerBit();
    }
}

double AbsoluteEncoder::toIU(double radians, bool use_zero_position) const
{
    if (use_zero_position) {
        return (radians / getRadiansPerBit()) + zero_position_iu_;
    } else {
        return radians / getRadiansPerBit();
    }
}

bool AbsoluteEncoder::isWithinHardLimitsIU(int32_t iu) const
{
    return (iu > this->lower_limit_iu_ && iu < this->upper_limit_iu_);
}

bool AbsoluteEncoder::isWithinSoftLimitsIU(int32_t iu) const
{
    return (iu > this->lower_soft_limit_iu_ && iu < this->upper_soft_limit_iu_);
}

bool AbsoluteEncoder::isValidTargetIU(
    int32_t current_iu, int32_t target_iu) const
{
    if (target_iu <= this->lower_soft_limit_iu_) {
        return target_iu >= current_iu;
    }

    if (target_iu >= this->upper_soft_limit_iu_) {
        return target_iu <= current_iu;
    }

    return true;
}

int32_t AbsoluteEncoder::getUpperSoftLimitIU() const
{
    return this->upper_soft_limit_iu_;
}

int32_t AbsoluteEncoder::getLowerSoftLimitIU() const
{
    return this->lower_soft_limit_iu_;
}

int32_t AbsoluteEncoder::getUpperHardLimitIU() const
{
    return this->upper_limit_iu_;
}

int32_t AbsoluteEncoder::getLowerHardLimitIU() const
{
    return this->lower_limit_iu_;
}

} // namespace march
