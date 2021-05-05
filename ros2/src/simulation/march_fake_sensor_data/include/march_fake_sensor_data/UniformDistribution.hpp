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

// Copyright Project MARCH 2020
#pragma once

#include <random>

class UniformDistribution {
private:
    // Variables that are required for random number generation.
    std::random_device random_device;
    std::mt19937 generator;
    std::uniform_int_distribution<int> distribution;

public:
    UniformDistribution(const int start, const int end);

    // Assume the start to be 0 if only the end is specified.
    explicit UniformDistribution(const int end)
        : UniformDistribution { /*start=*/0, end } {};

    // While the constructor does also create the random device and the random
    // generator, there should also be a possibility to change the range of the
    // generator without replacing it fully. In order to ensure that the pseudo
    // random numbers have random characteristics, it is necessary that the
    // random device and generator are kept the same.
    void set_range(const int start, const int end);

    // Get a uniformly distributed pseudo random number in the specified range.
    int get_random_number();
};
