// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_H
#include "march_hardware/motor_controller/actuation_mode.h"
#include "march_hardware/motor_controller/motor_controller_state.h"
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/ethercat/slave.h"
#include <string>
#include <memory>

namespace march
{
class MotorController : public Slave
{
public:
  MotorController(const Slave& slave, std::unique_ptr<AbsoluteEncoder> absolute_encoder,
                  std::unique_ptr<IncrementalEncoder> incremental_encoder, ActuationMode actuation_mode);

  // Get the most precise position or velocity
  float getPosition();
  float getVelocity();

  // Get the position, either absolute or incremental
  float getPosition(bool absolute);
  float getVelocity(bool absolute);

  // A MotorController should support both actuating by position (radians) or by torque
  virtual void actuateRadians(float target_position) = 0;
  virtual void actuateTorque(float target_effort) = 0;

  // Getter and setter for the ActuationMode
  ActuationMode getActuationMode() const;
  void setActuationMode(ActuationMode actuation_mode);

  // Actuate based on the actuation mode of the motor controller
  void actuate(float target);

  // Prepare the MotorController for actuation, move it into its 'ready' state
  virtual void prepareActuation() = 0;

  // Transform the ActuationMode to a number that is understood by the MotorController
  virtual unsigned int getActuationModeNumber() const = 0;

  // Get whether the incremental encoder is more precise than the absolute encoder
  bool isIncrementalEncoderMorePrecise() const;

  // A MotorController doesn't necessarily have an AbsoluteEncoder and an IncrementalEncoder, but will have
  // at least one of the two
  bool hasAbsoluteEncoder() const;
  std::unique_ptr<AbsoluteEncoder>& getAbsoluteEncoder();
  bool hasIncrementalEncoder() const;
  std::unique_ptr<IncrementalEncoder>& getIncrementalEncoder();

  // Getters for specific information about the state of the motor and its controller
  virtual float getTorque() = 0;
  virtual float getMotorCurrent() = 0;
  virtual float getMotorControllerVoltage() = 0;
  virtual float getMotorVoltage() = 0;

  // Get a full description of the state of the MotorController
  virtual std::unique_ptr<MotorControllerState> getState() = 0;

  ~MotorController() {};

  // Override comparison operator
  friend bool operator==(const MotorController& lhs, const MotorController& rhs)
  {
    return lhs.getSlaveIndex() == rhs.getSlaveIndex() && *lhs.absolute_encoder_ == *rhs.absolute_encoder_ &&
           *lhs.incremental_encoder_ == *rhs.incremental_encoder_ &&
           lhs.actuation_mode_.getValue() == rhs.actuation_mode_.getValue();
  }
  // Override stream operator for clean printing
  friend std::ostream& operator<<(std::ostream& os, const MotorController& motor_controller)
  {
    return os << "slaveIndex: " << motor_controller.getSlaveIndex() << ", "
              << "incrementalEncoder: " << *motor_controller.incremental_encoder_ << ", "
              << "absoluteEncoder: " << *motor_controller.absolute_encoder_
              << "actuationMode" << motor_controller.actuation_mode_.toString();
  }

  // Watchdog base time = 1 / 25 MHz * (2498 + 2) = 0.0001 seconds=100 µs
  static const uint16_t WATCHDOG_DIVIDER = 2498;
  // 500 * 100us = 50 ms = watchdog timer
  static const uint16_t WATCHDOG_TIME = 500;

protected:
  // Getters for absolute and incremental position and velocity.
  // These will throw an error where the encoder is not available.
  virtual float getAbsolutePosition() = 0;
  virtual float getIncrementalPosition() = 0;
  virtual float getAbsoluteVelocity() = 0;
  virtual float getIncrementalVelocity() = 0;

  // A MotorController doesn't necessarily have an AbsoluteEncoder and an IncrementalEncoder, but will have
  // at least one of the two
  std::unique_ptr<AbsoluteEncoder> absolute_encoder_ = nullptr;
  std::unique_ptr<IncrementalEncoder> incremental_encoder_ = nullptr;
  ActuationMode actuation_mode_;
};

}  // namespace march
#endif  // MARCH_HARDWARE_MOTOR_CONTROLLER_H
