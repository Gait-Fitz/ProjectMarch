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

#include "march_hardware/motor_controller/motor_controller.h"
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/slave.h"
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/motor_controller_state.h"
#include <memory>
namespace march {
MotorController::MotorController(const Slave& slave,
    std::unique_ptr<AbsoluteEncoder> absolute_encoder,
    std::unique_ptr<IncrementalEncoder> incremental_encoder,
    ActuationMode actuation_mode)
    : Slave(slave)
    , absolute_encoder_(std::move(absolute_encoder))
    , incremental_encoder_(std::move(incremental_encoder))
    , actuation_mode_(actuation_mode)
{
    if (!absolute_encoder_ && !incremental_encoder_) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "A MotorController needs at least an incremental or an absolute "
            "encoder");
    }
}

MotorController::MotorController(const Slave& slave,
    std::unique_ptr<AbsoluteEncoder> absolute_encoder,
    ActuationMode actuation_mode)
    : MotorController(
        slave, std::move(absolute_encoder), nullptr, actuation_mode)
{
}

MotorController::MotorController(const Slave& slave,
    std::unique_ptr<IncrementalEncoder> incremental_encoder,
    ActuationMode actuation_mode)
    : MotorController(
        slave, nullptr, std::move(incremental_encoder), actuation_mode)
{
}

bool MotorController::isIncrementalEncoderMorePrecise() const
{
    if (!hasIncrementalEncoder()) {
        return false;
    }
    if (!hasAbsoluteEncoder()) {
        return true;
    }
    return incremental_encoder_->getRadiansPerBit()
        < absolute_encoder_->getRadiansPerBit();
}

float MotorController::getPosition()
{
    if (isIncrementalEncoderMorePrecise()) {
        return getIncrementalPosition();
    }
    return getAbsolutePosition();
}

float MotorController::getVelocity()
{
    if (isIncrementalEncoderMorePrecise()) {
        return getIncrementalVelocity();
    }
    return getAbsoluteVelocity();
}

float MotorController::getAbsolutePosition()
{
    if (!hasAbsoluteEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "Cannot get absolute position,"
            "the motor controller has no absolute encoder");
    }
    return getAbsolutePositionUnchecked();
}

float MotorController::getAbsoluteVelocity()
{
    if (!hasAbsoluteEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "Cannot get absolute velocity,"
            "the motor controller has no absolute encoder");
    }
    return getAbsoluteVelocityUnchecked();
}

float MotorController::getIncrementalPosition()
{
    if (!hasIncrementalEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "Cannot get incremental position,"
            "the motor controller has no incremental encoder");
    }
    return getIncrementalPositionUnchecked();
}

float MotorController::getIncrementalVelocity()
{
    if (!hasIncrementalEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "Cannot get incremental velocity,"
            "the motor controller has no incremental encoder");
    }
    return getIncrementalVelocityUnchecked();
}

ActuationMode MotorController::getActuationMode() const
{
    return actuation_mode_;
}

void MotorController::setActuationMode(ActuationMode actuation_mode)
{
    actuation_mode_ = actuation_mode;
};

bool MotorController::hasAbsoluteEncoder() const
{
    return absolute_encoder_ != nullptr;
}

bool MotorController::hasIncrementalEncoder() const
{
    return incremental_encoder_ != nullptr;
}

std::unique_ptr<AbsoluteEncoder>& MotorController::getAbsoluteEncoder()
{
    if (!hasAbsoluteEncoder()) {
        throw error::HardwareException(
            error::ErrorType::MISSING_ENCODER, "Cannot get absolute encoder");
    }
    return absolute_encoder_;
}

std::unique_ptr<IncrementalEncoder>& MotorController::getIncrementalEncoder()
{
    if (!hasIncrementalEncoder()) {
        throw error::HardwareException(error::ErrorType::MISSING_ENCODER,
            "Cannot get incremental encoder");
    }
    return incremental_encoder_;
}

void MotorController::actuate(float target)
{
    if (actuation_mode_ == march::ActuationMode::position) {
        actuateRadians(target);
    } else if (actuation_mode_ == march::ActuationMode::torque) {
        actuateTorque(target);
    } else {
        throw error::HardwareException(error::ErrorType::INVALID_ACTUATION_MODE,
            "Actuation mode %s is not supported",
            actuation_mode_.toString().c_str());
    }
}
} // namespace march