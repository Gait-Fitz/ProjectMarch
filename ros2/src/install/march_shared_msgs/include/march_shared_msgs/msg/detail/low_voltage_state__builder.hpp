// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from march_shared_msgs:msg/LowVoltageState.idl
// generated code does not contain a copyright notice

#ifndef MARCH_SHARED_MSGS__MSG__DETAIL__LOW_VOLTAGE_STATE__BUILDER_HPP_
#define MARCH_SHARED_MSGS__MSG__DETAIL__LOW_VOLTAGE_STATE__BUILDER_HPP_

#include "march_shared_msgs/msg/detail/low_voltage_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace march_shared_msgs
{

namespace msg
{

namespace builder
{

class Init_LowVoltageState_lv2_ok
{
public:
  explicit Init_LowVoltageState_lv2_ok(::march_shared_msgs::msg::LowVoltageState & msg)
  : msg_(msg)
  {}
  ::march_shared_msgs::msg::LowVoltageState lv2_ok(::march_shared_msgs::msg::LowVoltageState::_lv2_ok_type arg)
  {
    msg_.lv2_ok = std::move(arg);
    return std::move(msg_);
  }

private:
  ::march_shared_msgs::msg::LowVoltageState msg_;
};

class Init_LowVoltageState_lv1_ok
{
public:
  explicit Init_LowVoltageState_lv1_ok(::march_shared_msgs::msg::LowVoltageState & msg)
  : msg_(msg)
  {}
  Init_LowVoltageState_lv2_ok lv1_ok(::march_shared_msgs::msg::LowVoltageState::_lv1_ok_type arg)
  {
    msg_.lv1_ok = std::move(arg);
    return Init_LowVoltageState_lv2_ok(msg_);
  }

private:
  ::march_shared_msgs::msg::LowVoltageState msg_;
};

class Init_LowVoltageState_lv2_current
{
public:
  explicit Init_LowVoltageState_lv2_current(::march_shared_msgs::msg::LowVoltageState & msg)
  : msg_(msg)
  {}
  Init_LowVoltageState_lv1_ok lv2_current(::march_shared_msgs::msg::LowVoltageState::_lv2_current_type arg)
  {
    msg_.lv2_current = std::move(arg);
    return Init_LowVoltageState_lv1_ok(msg_);
  }

private:
  ::march_shared_msgs::msg::LowVoltageState msg_;
};

class Init_LowVoltageState_lv1_current
{
public:
  Init_LowVoltageState_lv1_current()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LowVoltageState_lv2_current lv1_current(::march_shared_msgs::msg::LowVoltageState::_lv1_current_type arg)
  {
    msg_.lv1_current = std::move(arg);
    return Init_LowVoltageState_lv2_current(msg_);
  }

private:
  ::march_shared_msgs::msg::LowVoltageState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::march_shared_msgs::msg::LowVoltageState>()
{
  return march_shared_msgs::msg::builder::Init_LowVoltageState_lv1_current();
}

}  // namespace march_shared_msgs

#endif  // MARCH_SHARED_MSGS__MSG__DETAIL__LOW_VOLTAGE_STATE__BUILDER_HPP_
