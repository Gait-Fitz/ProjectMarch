// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/MotorControllerState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__MOTOR_CONTROLLER_STATE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__MOTOR_CONTROLLER_STATE__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/motor_controller_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorControllerState_incremental_velocity
{
public:
  explicit Init_MotorControllerState_incremental_velocity(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::MotorControllerState incremental_velocity(::march_shared_msgs::msg::MotorControllerState::_incremental_velocity_type arg)
  {
    msg_.incremental_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_absolute_velocity
{
public:
  explicit Init_MotorControllerState_absolute_velocity(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_incremental_velocity absolute_velocity(::march_shared_msgs::msg::MotorControllerState::_absolute_velocity_type arg)
  {
    msg_.absolute_velocity = std::move(arg);
    return Init_MotorControllerState_incremental_velocity(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_incremental_position
{
public:
  explicit Init_MotorControllerState_incremental_position(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_absolute_velocity incremental_position(::march_shared_msgs::msg::MotorControllerState::_incremental_position_type arg)
  {
    msg_.incremental_position = std::move(arg);
    return Init_MotorControllerState_absolute_velocity(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_absolute_position
{
public:
  explicit Init_MotorControllerState_absolute_position(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_incremental_position absolute_position(::march_shared_msgs::msg::MotorControllerState::_absolute_position_type arg)
  {
    msg_.absolute_position = std::move(arg);
    return Init_MotorControllerState_incremental_position(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_incremental_velocity_iu
{
public:
  explicit Init_MotorControllerState_incremental_velocity_iu(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_absolute_position incremental_velocity_iu(::march_shared_msgs::msg::MotorControllerState::_incremental_velocity_iu_type arg)
  {
    msg_.incremental_velocity_iu = std::move(arg);
    return Init_MotorControllerState_absolute_position(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_absolute_velocity_iu
{
public:
  explicit Init_MotorControllerState_absolute_velocity_iu(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_incremental_velocity_iu absolute_velocity_iu(::march_shared_msgs::msg::MotorControllerState::_absolute_velocity_iu_type arg)
  {
    msg_.absolute_velocity_iu = std::move(arg);
    return Init_MotorControllerState_incremental_velocity_iu(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_incremental_position_iu
{
public:
  explicit Init_MotorControllerState_incremental_position_iu(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_absolute_velocity_iu incremental_position_iu(::march_shared_msgs::msg::MotorControllerState::_incremental_position_iu_type arg)
  {
    msg_.incremental_position_iu = std::move(arg);
    return Init_MotorControllerState_absolute_velocity_iu(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_absolute_position_iu
{
public:
  explicit Init_MotorControllerState_absolute_position_iu(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_incremental_position_iu absolute_position_iu(::march_shared_msgs::msg::MotorControllerState::_absolute_position_iu_type arg)
  {
    msg_.absolute_position_iu = std::move(arg);
    return Init_MotorControllerState_incremental_position_iu(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_temperature
{
public:
  explicit Init_MotorControllerState_temperature(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_absolute_position_iu temperature(::march_shared_msgs::msg::MotorControllerState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_MotorControllerState_absolute_position_iu(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_motor_voltage
{
public:
  explicit Init_MotorControllerState_motor_voltage(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_temperature motor_voltage(::march_shared_msgs::msg::MotorControllerState::_motor_voltage_type arg)
  {
    msg_.motor_voltage = std::move(arg);
    return Init_MotorControllerState_temperature(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_motor_current
{
public:
  explicit Init_MotorControllerState_motor_current(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_motor_voltage motor_current(::march_shared_msgs::msg::MotorControllerState::_motor_current_type arg)
  {
    msg_.motor_current = std::move(arg);
    return Init_MotorControllerState_motor_voltage(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_operational_state
{
public:
  explicit Init_MotorControllerState_operational_state(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_motor_current operational_state(::march_shared_msgs::msg::MotorControllerState::_operational_state_type arg)
  {
    msg_.operational_state = std::move(arg);
    return Init_MotorControllerState_motor_current(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_error_status
{
public:
  explicit Init_MotorControllerState_error_status(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_operational_state error_status(::march_shared_msgs::msg::MotorControllerState::_error_status_type arg)
  {
    msg_.error_status = std::move(arg);
    return Init_MotorControllerState_operational_state(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_joint_names
{
public:
  explicit Init_MotorControllerState_joint_names(::march_shared_msgs::msg::MotorControllerState & msg)
  : msg_(msg)
  {}
  Init_MotorControllerState_error_status joint_names(::march_shared_msgs::msg::MotorControllerState::_joint_names_type arg)
  {
    msg_.joint_names = std::move(arg);
    return Init_MotorControllerState_error_status(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

class Init_MotorControllerState_header
{
public:
  Init_MotorControllerState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorControllerState_joint_names header(::march_shared_msgs::msg::MotorControllerState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorControllerState_joint_names(msg_);
  }

private:
  ::march_shared_msgs::msg::MotorControllerState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::MotorControllerState>()
{
  return march_shared_msgs::msg::builder::Init_MotorControllerState_header();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__MOTOR_CONTROLLER_STATE__BUILDER_HPP_
