/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2019 Olav de Haas
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

// Copyright 2019 Project March
#include <gtest/gtest.h>

#include "march_imu_manager/wireless_master.h"

TEST(WirelessMasterTest, emptyRates)
{
    XsIntArray rates(/*sz=*/0, /*src=*/nullptr);
    const int rate = WirelessMaster::findClosestUpdateRate(
        rates, /*desired_update_rate=*/0);
    ASSERT_EQ(rate, 0);
}

TEST(WirelessMasterTest, oneRate)
{
    const int supportedRate = 60;
    XsIntArray rates(/*sz=*/1, &supportedRate);
    const int rate = WirelessMaster::findClosestUpdateRate(
        rates, /*desired_update_rate=*/0);
    ASSERT_EQ(rate, supportedRate);
}

TEST(WirelessMasterTest, matchingRate)
{
    const std::array<int, 3> supportedRates { 60, 80, 100 };
    XsIntArray rates(/*sz=*/supportedRates.size(), supportedRates.data());
    const int rate
        = WirelessMaster::findClosestUpdateRate(rates, supportedRates[1]);
    ASSERT_EQ(rate, supportedRates[1]);
}

TEST(WirelessMasterTest, twoClosestRates)
{
    const std::array<int, 2> supportedRates { 10, 20 };
    XsIntArray rates(/*sz=*/supportedRates.size(), supportedRates.data());
    const int rate = WirelessMaster::findClosestUpdateRate(
        rates, /*desired_update_rate=*/15);
    ASSERT_EQ(rate, supportedRates[0]);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
