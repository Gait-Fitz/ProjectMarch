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

// Copyright 2020 Project MARCH

#include "march_fake_sensor_data/UniformDistribution.hpp"
#include <gtest/gtest.h>

TEST(UniformDistributionTest, generate_within_bounds)
{
    int lower_bound = -5;
    int upper_bound = 5;
    UniformDistribution distribution { lower_bound, upper_bound };
    for (int i { 0 }; i < 30; ++i) {
        auto number = distribution.get_random_number();
        ASSERT_LE(lower_bound, number);
        ASSERT_GE(upper_bound, number);
    }
}

TEST(UniformDistributionTest, valid_change_range)
{
    UniformDistribution distribution { /*start=*/-100, /*end=*/100 };
    distribution.set_range(/*start=*/-1, /*end=*/1);
    for (int i { 0 }; i < 10; ++i) {
        auto number = distribution.get_random_number();
        ASSERT_LE(-1, number);
        ASSERT_GE(1, number);
    }
}

TEST(UniformDistributionTest, inverted_range)
{
    int lower_bound = -3;
    int upper_bound = 3;
    UniformDistribution distribution { upper_bound, lower_bound };
    for (int i { 0 }; i < 30; ++i) {
        auto number = distribution.get_random_number();
        ASSERT_LE(lower_bound, number);
        ASSERT_GE(upper_bound, number);
    }
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
