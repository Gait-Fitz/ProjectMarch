/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
 * Copyright (C) 2019 Isha Dijks, Jitske de Vries,
 *                    Martijn van der Marel, Tim Buckers
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
#include "../mocks/mock_absolute_encoder.h"
#include "../mocks/mock_incremental_encoder.h"
#include "../mocks/mock_slave.h"

#include <march_hardware/error/hardware_exception.h>
#include <march_hardware/motor_controller/imotioncube/imotioncube.h>

#include <memory>
#include <stdexcept>
#include <utility>

#include <gtest/gtest.h>

class IMotionCubeTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        this->mock_absolute_encoder = std::make_unique<MockAbsoluteEncoder>();
        this->mock_incremental_encoder
            = std::make_unique<MockIncrementalEncoder>();
    }

    MockPdoInterfacePtr mock_pdo = std::make_shared<MockPdoInterface>();
    MockSdoInterfacePtr mock_sdo = std::make_shared<MockSdoInterface>();
    MockSlave mock_slave = MockSlave(this->mock_pdo, this->mock_sdo);
    std::unique_ptr<MockAbsoluteEncoder> mock_absolute_encoder;
    std::unique_ptr<MockIncrementalEncoder> mock_incremental_encoder;
};

TEST_F(IMotionCubeTest, NoAbsoluteEncoder)
{
    ASSERT_THROW(march::IMotionCube(mock_slave, nullptr,
                     std::move(this->mock_incremental_encoder),
                     march::ActuationMode::unknown),
        march::error::HardwareException);
}

TEST_F(IMotionCubeTest, NoIncrementalEncoder)
{
    ASSERT_THROW(
        march::IMotionCube(mock_slave, std::move(this->mock_absolute_encoder),
            nullptr, march::ActuationMode::unknown),
        march::error::HardwareException);
}

TEST_F(IMotionCubeTest, SlaveIndexOne)
{
    march::IMotionCube imc(mock_slave, std::move(this->mock_absolute_encoder),
        std::move(this->mock_incremental_encoder),
        march::ActuationMode::unknown);
    ASSERT_EQ(1, imc.getSlaveIndex());
}

TEST_F(IMotionCubeTest, NoActuationMode)
{
    march::IMotionCube imc(mock_slave, std::move(this->mock_absolute_encoder),
        std::move(this->mock_incremental_encoder),
        march::ActuationMode::unknown);
    ASSERT_THROW(imc.actuateRadians(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, ActuationModeTorqueActuateRadians)
{
    march::IMotionCube imc(mock_slave, std::move(this->mock_absolute_encoder),
        std::move(this->mock_incremental_encoder),
        march::ActuationMode::torque);

    ASSERT_EQ(march::ActuationMode::torque, imc.getActuationMode().getValue());
    ASSERT_THROW(imc.actuateRadians(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, ActuationModePositionActuateTorque)
{
    march::IMotionCube imc(mock_slave, std::move(this->mock_absolute_encoder),
        std::move(this->mock_incremental_encoder),
        march::ActuationMode::position);

    ASSERT_THROW(imc.actuateTorque(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, OperationEnabledWithoutActuationMode)
{
    march::IMotionCube imc(mock_slave, std::move(this->mock_absolute_encoder),
        std::move(this->mock_incremental_encoder),
        march::ActuationMode::unknown);
    ASSERT_THROW(imc.prepareActuation(), march::error::HardwareException);
}
