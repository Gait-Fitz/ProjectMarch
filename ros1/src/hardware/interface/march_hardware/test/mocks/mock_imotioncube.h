/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas, Roel Vos, Rutger van Beek
 * Copyright (C) 2019 Tim Buckers
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

#pragma once
#include "mock_absolute_encoder.h"
#include "mock_incremental_encoder.h"
#include "mock_slave.h"

#include "march_hardware/ethercat/sdo_interface.h"
#include "march_hardware/motor_controller/imotioncube/imotioncube.h"
#include "march_hardware/motor_controller/imotioncube/imotioncube_state.h"

#include <memory>

#include <gmock/gmock.h>

class MockIMotionCube : public march::IMotionCube {
public:
    MockIMotionCube()
        : IMotionCube(MockSlave(), std::make_unique<MockAbsoluteEncoder>(),
            std::make_unique<MockIncrementalEncoder>(),
            march::ActuationMode::unknown)
    {
    }

    MOCK_METHOD0(getState, std::unique_ptr<march::MotorControllerState>());

    MOCK_METHOD0(prepareActuation, void());

    MOCK_METHOD0(isIncrementalEncoderMorePrecise, bool());

    MOCK_METHOD0(getIncrementalPositionUnchecked, float());
    MOCK_METHOD0(getAbsolutePositionUnchecked, float());
    MOCK_METHOD0(getIncrementalVelocityUnchecked, float());
    MOCK_METHOD0(getAbsoluteVelocityUnchecked, float());

    MOCK_METHOD0(getMotorControllerVoltage, float());
    MOCK_METHOD0(getMotorVoltage, float());
    MOCK_METHOD0(getMotorCurrent, float());

    MOCK_METHOD1(actuateRadians, void(float));
    MOCK_METHOD1(actuateTorque, void(float));

    MOCK_METHOD2(initSdo, bool(march::SdoSlaveInterface& sdo, int cycle_time));
    MOCK_METHOD1(reset, void(march::SdoSlaveInterface&));
};
