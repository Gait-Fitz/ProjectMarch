/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
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
#include "../mocks/mock_pdo_interface.h"
#include "../mocks/mock_sdo_interface.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/slave.h"

#include <stdexcept>

#include <gtest/gtest.h>

class SlaveTest : public testing::Test {
protected:
    MockPdoInterfacePtr mock_pdo = std::make_shared<MockPdoInterface>();
    MockSdoInterfacePtr mock_sdo = std::make_shared<MockSdoInterface>();
};

TEST_F(SlaveTest, CorrectSlaveIndex)
{
    march::Slave slave(/*slave_index=*/1, this->mock_pdo, this->mock_sdo);
    ASSERT_EQ(slave.getSlaveIndex(), 1);
}

TEST_F(SlaveTest, NoPdoInterface)
{
    ASSERT_THROW(
        march::Slave(1, nullptr, this->mock_sdo), std::invalid_argument);
}

TEST_F(SlaveTest, NoSdoInterface)
{
    ASSERT_THROW(
        march::Slave(1, this->mock_pdo, nullptr), std::invalid_argument);
}

TEST_F(SlaveTest, InvalidSlaveIndex)
{
    ASSERT_THROW(march::Slave(0, this->mock_pdo, this->mock_sdo),
        march::error::HardwareException);
}
