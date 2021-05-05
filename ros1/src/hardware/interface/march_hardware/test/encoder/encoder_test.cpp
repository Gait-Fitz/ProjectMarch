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

// Copyright 2020 Project March.
#include "../mocks/mock_encoder.h"
#include "../mocks/mock_pdo_interface.h"
#include "march_hardware/error/hardware_exception.h"

#include <cmath>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::Eq;
using testing::Return;

/**
 * This test fixture uses the MockEncoder to test non virtual methods
 * of the Encoder abstract class. Otherwise it is not possible to
 * instantiate a pure abstract class. So as long as the non virtual
 * methods are tested all is ok.
 */
class EncoderTest : public testing::Test {
protected:
    const uint16_t slave_index = 1;
    const size_t resolution = 12;
};

TEST_F(EncoderTest, ResolutionBelowRange)
{
    ASSERT_THROW(MockEncoder(0), march::error::HardwareException);
}

TEST_F(EncoderTest, ResolutionAboveRange)
{
    ASSERT_THROW(MockEncoder(50), march::error::HardwareException);
}

TEST_F(EncoderTest, CorrectTotalPositions)
{
    MockEncoder encoder(this->resolution);
    ASSERT_EQ(std::pow(2, this->resolution), encoder.getTotalPositions());
}
