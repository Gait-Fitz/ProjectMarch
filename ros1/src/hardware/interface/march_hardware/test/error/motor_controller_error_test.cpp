/*
 * Copyright (C) 2021 Bas Volkers
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
#include <march_hardware/error/motor_controller_error.h>

#include <gtest/gtest.h>

TEST(MotionErrorTest, ParseNoMotionError)
{
    ASSERT_EQ(march::error::parseError((uint16_t)0,
                  march::error::ErrorRegister::IMOTIONCUBE_MOTION_ERROR),
        "");
}

TEST(MotionErrorTest, ParseCorrectMotionError)
{
    const uint16_t error = 1;
    ASSERT_EQ(march::error::parseError(
                  error, march::error::ErrorRegister::IMOTIONCUBE_MOTION_ERROR),
        march::error::IMOTIONCUBE_MOTION_ERRORS[0]);
}

TEST(MotionErrorTest, ParseMultipleErrors)
{
    const uint16_t error = 0b1000000000001100;
    std::string expected;
    expected += march::error::IMOTIONCUBE_MOTION_ERRORS[2];
    expected += march::error::IMOTIONCUBE_MOTION_ERRORS[3];
    expected += march::error::IMOTIONCUBE_MOTION_ERRORS[15];
    ASSERT_EQ(march::error::parseError(
                  error, march::error::ErrorRegister::IMOTIONCUBE_MOTION_ERROR),
        expected);
}

TEST(TestDetailedMotionError, ParseNoDetailedMotionError)
{
    ASSERT_EQ(
        march::error::parseError((uint16_t)0,
            march::error::ErrorRegister::IMOTIONCUBE_DETAILED_MOTION_ERROR),
        "");
}

TEST(TestDetailedMotionError, ParseCorrectDetailedMotionError)
{
    const uint16_t error = 1;
    ASSERT_EQ(
        march::error::parseError(error,
            march::error::ErrorRegister::IMOTIONCUBE_DETAILED_MOTION_ERROR),
        march::error::IMOTIONCUBE_DETAILED_MOTION_ERRORS[0]);
}

TEST(TestDetailedMotionError, ParseMultipleDetailedErrors)
{
    const uint16_t error = 0b100001100;
    std::string expected;
    expected += march::error::IMOTIONCUBE_DETAILED_MOTION_ERRORS[2];
    expected += march::error::IMOTIONCUBE_DETAILED_MOTION_ERRORS[3];
    expected += march::error::IMOTIONCUBE_DETAILED_MOTION_ERRORS[8];
    ASSERT_EQ(
        march::error::parseError(error,
            march::error::ErrorRegister::IMOTIONCUBE_DETAILED_MOTION_ERROR),
        expected);
}

TEST(TestSecondDetailedMotionError, ParseNoSecondDetailedMotionError)
{
    ASSERT_EQ(march::error::parseError((uint16_t)0,
                  march::error::ErrorRegister::
                      IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR),
        "");
}

TEST(TestSecondDetailedMotionError, ParseCorrectSecondDetailedMotionError)
{
    const uint16_t error = 1;
    ASSERT_EQ(march::error::parseError(error,
                  march::error::ErrorRegister::
                      IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR),
        march::error::IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS[0]);
}

TEST(TestSecondDetailedMotionError, ParseMultipleSecondDetailedErrors)
{
    const uint16_t error = 0b0001100;
    std::string expected;
    expected += march::error::IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS[2];
    expected += march::error::IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS[3];
    ASSERT_EQ(march::error::parseError(error,
                  march::error::ErrorRegister::
                      IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR),
        expected);
}