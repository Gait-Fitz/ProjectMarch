/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Thijs Raymakers
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

// Copyright 2020 Project March.

#include "march_fake_sensor_data/UniformDistribution.hpp"

/**
 * @file UniformDistribution.cpp
 * @brief Provide a wrapper around the random functions in the standard library.
 * @details The wrapper makes sure that the same distribution will be used,
 * unless the range of the distribution is changed. In that case, a new
 * distribution is created. This is necessary to ensure that the generated
 * pseudo random distribution has pseude random properties.
 */

/**
 * @brief Create a random uniform distribution between start and end
 * @param start The lower bound (inclusive)
 * @param end The upper bound (inclusive)
 * @return An object that can generate random distributions.
 */
UniformDistribution::UniformDistribution(const int start, const int end)
    : generator(random_device())
{
    set_range(start, end);
}

/**
 * @brief Set the range of an existing distribution to a new range.
 * @param start The lower bound (inclusive)
 * @param end The upper bound (inclusive)
 */
void UniformDistribution::set_range(int start, int end)
{
    // If the end is smaller than the start, swap them and treat the
    // end as the start.
    if (end < start) {
        std::swap(start, end);
    }

    std::uniform_int_distribution<int> new_distribution(start, end);
    distribution = new_distribution;
}

/**
 * @brief Get a random number from the uniform distribution.
 */
int UniformDistribution::get_random_number()
{
    return distribution(generator);
}
