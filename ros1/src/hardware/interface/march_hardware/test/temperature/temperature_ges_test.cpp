/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
 * Copyright (C) 2019 Isha Dijks, Martijn van der Marel
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

// Copyright 2018 Project March.
#include "../mocks/mock_slave.h"
#include "march_hardware/temperature/temperature_ges.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::Eq;
using testing::Return;

class TemperatureGESTest : public ::testing::Test {
protected:
    MockPdoInterfacePtr mock_pdo = std::make_shared<MockPdoInterface>();
    MockSdoInterfacePtr mock_sdo = std::make_shared<MockSdoInterface>();
    MockSlave mock_slave = MockSlave(this->mock_pdo, this->mock_sdo);
};

TEST_F(TemperatureGESTest, ByteOffsetOne)
{
    ASSERT_NO_THROW(march::TemperatureGES(this->mock_slave, 1));
}

TEST_F(TemperatureGESTest, ByteOffsetZero)
{
    ASSERT_NO_THROW(march::TemperatureGES(this->mock_slave, 0));
}

TEST_F(TemperatureGESTest, GetTemperature)
{
    const uint8_t expected_offset = 3;
    const float temperature = 1.0;
    EXPECT_CALL(*this->mock_pdo,
        read32(Eq(this->mock_slave.getSlaveIndex()), Eq(expected_offset)))
        .WillOnce(Return(march::bit32 { .f = temperature }));

    const march::TemperatureGES ges(this->mock_slave, expected_offset);

    ASSERT_FLOAT_EQ(ges.getTemperature(), temperature);
}
