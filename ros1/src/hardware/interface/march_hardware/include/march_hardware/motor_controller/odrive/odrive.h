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

// Copyright 2021 Project March.

#ifndef MARCH_HARDWARE_ODRIVE_H
#define MARCH_HARDWARE_ODRIVE_H

#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/ethercat/odrive_pdo_map.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/ethercat/sdo_interface.h"
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"

#include <memory>
#include <string>
#include <unordered_map>

namespace march {
class ODrive : public MotorController {
public:
    /**
     * Constructs an IMotionCube with an incremental and absolute encoder.
     *
     * @param slave slave of the IMotionCube
     * @param absolute_encoder pointer to absolute encoder, required so cannot
     * be nullptr
     * @param incremental_encoder pointer to incremental encoder, required so
     * cannot be nullptr
     * @param actuation_mode actuation mode in which the IMotionCube must
     * operate
     * @throws std::invalid_argument When an absolute or incremental encoder is
     * nullptr.
     */
    ODrive(const Slave& slave, ODriveAxis axis,
        std::unique_ptr<AbsoluteEncoder> absolute_encoder,
        std::unique_ptr<IncrementalEncoder> incremental_encoder,
        ActuationMode actuation_mode);
    ODrive(const Slave& slave, ODriveAxis axis,
        std::unique_ptr<AbsoluteEncoder> absolute_encoder,
        ActuationMode actuation_mode);

    ~ODrive() noexcept override = default;

    // Override functions for actuating the ODrive
    void prepareActuation() override;
    void actuateTorque(float target_torque) override;
    void actuateRadians(float target_position) override;

    // Transform the ActuationMode to a number that is understood by the ODrive
    int getActuationModeNumber() const override;

    // Get a full description of the state of the ODrive
    std::unique_ptr<MotorControllerState> getState() override;

    // Getters for specific information about the state of the motor and the
    // ODrive
    float getTorque() override;
    float getMotorCurrent() override;
    float getMotorControllerVoltage() override;
    float getMotorVoltage() override;

protected:
    // Override protected functions from Slave class
    bool initSdo(SdoSlaveInterface& sdo, int cycle_time) override;
    void reset(SdoSlaveInterface& sdo) override;

    // Override protected functions from MotorController class
    float getAbsolutePositionUnchecked() override;
    float getIncrementalPositionUnchecked() override;
    float getAbsoluteVelocityUnchecked() override;
    float getIncrementalVelocityUnchecked() override;

private:
    // Set the ODrive in a certain axis state
    //  void goToAxisState(ODriveAxisState target_state);

    ODriveAxisState getAxisState();

    float getAbsolutePositionIU();
    float getAbsoluteVelocityIU();

    uint32_t getAxisError();
    uint32_t getMotorError();
    uint32_t getEncoderManagerError();
    uint32_t getEncoderError();
    uint32_t getControllerError();

    ODriveAxis axis_;
};

} // namespace march

#endif // MARCH_HARDWARE_ODRIVE_H
