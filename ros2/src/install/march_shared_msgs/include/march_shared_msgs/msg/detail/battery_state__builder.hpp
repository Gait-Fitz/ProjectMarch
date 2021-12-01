// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/BatteryState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__BATTERY_STATE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__BATTERY_STATE__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/battery_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_BatteryState_temperature
{
public:
  explicit Init_BatteryState_temperature(::march_shared_msgs::msg::BatteryState & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::BatteryState temperature(::march_shared_msgs::msg::BatteryState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::BatteryState msg_;
};

class Init_BatteryState_voltage
{
public:
  explicit Init_BatteryState_voltage(::march_shared_msgs::msg::BatteryState & msg)
  : msg_(msg)
  {}
  Init_BatteryState_temperature voltage(::march_shared_msgs::msg::BatteryState::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_BatteryState_temperature(msg_);
  }

private:
  ::march_shared_msgs::msg::BatteryState msg_;
};

class Init_BatteryState_percentage
{
public:
  Init_BatteryState_percentage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatteryState_voltage percentage(::march_shared_msgs::msg::BatteryState::_percentage_type arg)
  {
    msg_.percentage = std::move(arg);
    return Init_BatteryState_voltage(msg_);
  }

private:
  ::march_shared_msgs::msg::BatteryState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::BatteryState>()
{
  return march_shared_msgs::msg::builder::Init_BatteryState_percentage();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__BATTERY_STATE__BUILDER_HPP_
